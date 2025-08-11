#![no_std]
#![no_main]

use esp_hal;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::i2s::master::{DataFormat, I2s, Standard};
use esp_hal::riscv::interrupt::enable;
use esp_hal::time::Rate;
use esp_println::println;
use tlv320dac3100::driver::TLV320DAC3100;
use tlv320dac3100::typedefs::{CodecClkin, HeadsetButtonPressDebounce, HeadsetDetected, HeadsetDetectionDebounce, PllClkin};

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[esp_hal::main]
fn main() -> ! {
    println!("let's go");
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    println!("peripherals ready :)");
    // let dma_channel = peripherals.DMA_CH0;
    // // why 4092 ("the default chunk size used for DMA transfers.") instead of 4096?
    // let (mut rx_buffer, rx_descriptors, mut tx_buffer, tx_descriptors) = dma_buffers!(4 * 4092, 0);

    // I2S
    // let i2s = I2s::new(
    //     peripherals.I2S0,
    //     Standard::Philips,
    //     DataFormat::Data32Channel32, // TODO not sure about these widths...
    //     Rate::from_hz(44100), // sample rate
    //     dma_channel,
    // );
    //let i2s = i2s.with_mclk(peripherals.GPIO0);
    // let mut i2s_rx = i2s
    //     .i2s_rx
    //     .with_bclk(peripherals.GPIO1) // 2_822_400 Hz (44_100 * 2 * 32)
    //     .with_ws(peripherals.GPIO2)
    //     .with_din(peripherals.GPIO3)
    //     .build(rx_descriptors);

    // let mut i2s_rx = i2s
    //     .i2s_rx
    //     .with_bclk(peripherals.GPIO5)
    //     .with_ws(peripherals.GPIO6)
    //     .with_dout(peripherals.GPIO7)
    //     .build(tx_descriptors);

    // I2C
    let i2c = I2c::new(peripherals.I2C0, Config::default()).unwrap()
        .with_sda(peripherals.GPIO8)
        .with_scl(peripherals.GPIO9);
    println!("i2c configured :)");
    let mut dac = TLV320DAC3100::new(Delay::new(), i2c);
    println!("dac configured :)");

    // let mut dac = TLV320DAC3100::new(Delay::new(), i2c);
    //
    // let mut transfer = i2s_rx.read_dma_circular(&mut rx_buffer).unwrap();
    // loop {
    //     let avail = transfer.available().unwrap();
    //
    //     if avail > 0 {
    //         let mut rcv = [0u8; 5000];
    //         transfer.pop(&mut rcv[..avail]).unwrap();
    //         // TODO write
    //     }
    // }

    let mut enabled = true;
    let mut detected = HeadsetDetected::None;
    let mut debounce = HeadsetDetectionDebounce::Debounce16ms;
    let mut button_debounce = HeadsetButtonPressDebounce::Debounce16ms;
    match dac.set_headset_detection(enabled, HeadsetDetectionDebounce::Debounce16ms, HeadsetButtonPressDebounce::Debounce16ms) {
        Ok(_) => println!("Headset detected enabled :)"),
        Err(e) => println!("Failed to enable headset detection :( {:?}", e),
    }

    let delay = Delay::new();

    loop {
        match dac.get_headset_detection(&mut enabled, &mut detected, &mut debounce, &mut button_debounce) {
            Ok(_) => {
                if detected != HeadsetDetected::None {
                    println!("Headset detected!");
                } else {
                    println!("Headset not detected :(");
                }
            }
            Err(e) => {
                println!("err: {:?}", e);
            }
        }
        delay.delay_millis(3000);
    }
}