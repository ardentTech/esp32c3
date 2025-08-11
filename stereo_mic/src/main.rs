#![no_std]
#![no_main]

use esp_hal;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config, Error, I2c};
use esp_hal::i2s::master::{DataFormat, I2s, Standard};
use esp_hal::interrupt::status;
use esp_hal::riscv::asm::delay;
use esp_hal::riscv::interrupt::enable;
use esp_hal::time::Rate;
use esp_println::println;
use tlv320dac3100::error::TLV320DAC3100Error;
use tlv320dac3100::registers::{DAC_INTERRUPT_FLAGS_STICKY_BITS, HEADSET_DETECTION};
use tlv320dac3100::TLV320DAC3100;
use tlv320dac3100::typedefs::{CodecClkin, CodecInterface, CodecInterfaceWordLength, DacLeftOutputMixerRouting, DacRightOutputMixerRouting, Gpio1Mode, HeadsetButtonPressDebounce, HeadsetDetected, HeadsetDetectionDebounce, HpOutputVoltage, HpPowerOn, HpRampUp, LeftDataPath, MicBiasOutput, OutputStage, PllClkin, RightDataPath, SoftStepping, VolumeControl, VolumeControlHysteresis, VolumeControlThroughput};

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[esp_hal::main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let dma_channel = peripherals.DMA_CH0;
    // why 4092 ("the default chunk size used for DMA transfers.") instead of 4096?
    let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(4 * 4092, 0);
    // i2s
    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16, // TODO not sure about these widths...
        Rate::from_hz(44100), // sample rate
        dma_channel,
    );
    let mut i2s_tx = i2s
        .i2s_tx
        .with_bclk(peripherals.GPIO4)
        .with_ws(peripherals.GPIO5)
        .with_dout(peripherals.GPIO6)
        .build(tx_descriptors);

    let mut delay = Delay::new();
    // reset dac before use
    let mut reset = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());
    delay.delay_millis(100);
    reset.set_high();

    let i2c = I2c::new(peripherals.I2C0, Config::default()).unwrap()
        .with_sda(peripherals.GPIO7)
        .with_scl(peripherals.GPIO9);
    delay.delay_millis(10);

    let mut dac = TLV320DAC3100::new(i2c);

    dac.set_codec_interface_control_1(CodecInterface::I2S, CodecInterfaceWordLength::Word16Bits, false, false).expect("Error setting codec interface control 1");
    dac.set_clock_gen_muxing(PllClkin::Bclk, CodecClkin::PllClk).expect("Error setting clock gen muxing");
    // TODO PLL is powered later in ref example
    dac.set_pll_p_and_r_values(true, 2, 2).expect("");
    dac.set_pll_j_value(32).expect("Error setting pll j");
    dac.set_pll_d_value(0).expect("Error setting pll d");
    dac.set_dac_ndac_val(true, 8).expect("Error setting dac NDAC val");
    dac.set_dac_mdac_val(true, 2).expect("Error setting dac MDAC val");
    dac.set_dac_data_path_setup(true, true, LeftDataPath::Left, RightDataPath::Right, SoftStepping::OneStepPerPeriod).expect("Error setting dac DataPath setup");

    dac.set_dac_l_and_dac_r_output_mixer_routing(DacLeftOutputMixerRouting::LeftChannelMixerAmplifier, false, false, DacRightOutputMixerRouting::RightChannelMixerAmplifier, false, false).expect("Error setting dac L and R mixer routing");
    dac.set_dac_volume_control(false, false, VolumeControl::IndependentChannels).expect("Error setting dac Volume control");

    dac.set_dac_left_volume_control(18.0).expect("Error setting dac Left volume control");
    dac.set_dac_right_volume_control(18.0).expect("Error setting dac Right volume control");
    dac.set_headphone_drivers(true, true, HpOutputVoltage::Common1_35V, false).expect("Error setting dac Headphone drivers");

    dac.set_hpl_driver(0, true).expect("Error setting dac hpl driver");
    dac.set_hpr_driver(0, true).expect("Error setting dac hpr driver");
    dac.set_left_analog_volume_to_hpl(true, 6).expect("Error setting left analog volume to hpl");

    dac.set_right_analog_volume_to_hpr(true, 6).expect("Error setting right analog volume to hpr");
    dac.set_class_d_spk_amp(true).expect("Error setting class D spk amp");
    dac.set_class_d_spk_driver(OutputStage::Gain6dB, true).expect("Error setting class D spk driver");
    dac.set_left_analog_volume_to_spk(true, 0).expect("Error setting left analog volume to spk");
    dac.set_micbias(false, true, MicBiasOutput::PoweredAVDD).expect("Error setting micbias");
    dac.set_headset_detection(true, HeadsetDetectionDebounce::Debounce16ms, HeadsetButtonPressDebounce::Debounce0ms).expect("Error setting headset detection");
    dac.set_int1_control_register(true, true, false, false, false, false).expect("Error setting int1 control register");
    dac.set_gpio1_io_pin_control(Gpio1Mode::Int1).expect("Error setting gpio1 io pin");

    let mut enabled = false;
    let mut detected = HeadsetDetected::None;
    let mut headset_debounce = HeadsetDetectionDebounce::Debounce16ms;
    let mut button_debounce = HeadsetButtonPressDebounce::Debounce0ms;
    dac.get_headset_detection(&mut enabled, &mut detected, &mut headset_debounce, &mut button_debounce).unwrap();

    let mut sticky_bits = dac.read_reg(0, DAC_INTERRUPT_FLAGS_STICKY_BITS).unwrap();
    let mut transfer = i2s_tx.write_dma_circular(&tx_buffer)?;

    loop {

        // if enabled {
        //     println!("Headset detection enabled");
        // } else {
        //     println!("Headset detection disabled");
        // }
        // 
        // match detected {
        //     HeadsetDetected::None => println!("No headset detected"),
        //     HeadsetDetected::WithoutMic => println!("headset without mic detected"),
        //     HeadsetDetected::WithMic => println!("headset with mic detected"),
        // }
        // 
        // if ((sticky_bits >> 4) & 0b1) == 1 {
        //     println!("headset insertion/removal detected!");
        // }
        // 
        // dac.get_headset_detection(&mut enabled, &mut detected, &mut headset_debounce, &mut button_debounce).unwrap();
        // delay.delay_millis(3000);
    }
}