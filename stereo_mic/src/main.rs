#![no_std]
#![no_main]

use esp_hal;
use esp_hal::clock::CpuClock;
use esp_hal::dma_buffers;
use esp_hal::i2s::master::{DataFormat, I2s, Standard};
use esp_hal::time::Rate;
use esp_println::println;

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
    let (mut rx_buffer, rx_descriptors, _, _) = dma_buffers!(4 * 4092, 0);

    // driver
    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data32Channel32, // TODO not sure about these widths...
        Rate::from_hz(44100), // sample rate
        dma_channel,
    );
    //let i2s = i2s.with_mclk(peripherals.GPIO0);
    let mut i2s_rx = i2s
        .i2s_rx
        .with_bclk(peripherals.GPIO1) // 2_822_400 Hz (44_100 * 2 * 32)
        .with_ws(peripherals.GPIO0)
        .with_din(peripherals.GPIO3) // GPIO5 is what the example used
        .build(rx_descriptors);

    let mut transfer = i2s_rx.read_dma_circular(&mut rx_buffer).unwrap();

    loop {
        let avail = transfer.available().unwrap();

        if avail > 0 {
            let mut rcv = [0u8; 5000];
            transfer.pop(&mut rcv[..avail]).unwrap();
        }
        //println!("looping...")
    }
}
