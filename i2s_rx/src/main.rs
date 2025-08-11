#![no_std]
#![no_main]

use esp_hal;
use esp_hal::clock::CpuClock;
use esp_hal::dma::DmaError;
use esp_hal::dma_buffers;
use esp_hal::i2s::master::{DataFormat, I2s, Standard};
use esp_hal::time::Rate;
use esp_println::{print, println};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

const SAMPLE_RATE: u32 = 44100;
const BITS_PER_SAMPLE: u8 = 32;
const CHANNELS: u8 = 2;
const CLOCK_CYCLES_PER_BIT: u8 = 2;

#[esp_hal::main]
fn main() -> ! {
    // 160 MHz
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let dma_channel = peripherals.DMA_CH0;
    // why 4092 ("the default chunk size used for DMA transfers.") instead of 4096?
    // tx buffer size: 16368
    let (mut rx_buffer, rx_descriptors, _, _) = dma_buffers!(4 * 4092, 0);
    // i2s
    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        //DataFormat::Data32Channel24, // usable data is 18 bits, 6 0 bits, total frame of 32 bits
        DataFormat::Data32Channel32,
        Rate::from_hz(SAMPLE_RATE),
        dma_channel,
    );
    let mut i2s_rx = i2s
        .i2s_rx
        .with_bclk(peripherals.GPIO0)
        .with_ws(peripherals.GPIO2)
        .with_din(peripherals.GPIO3)
        .build(rx_descriptors);

    // let sawtooth: [u8; 256] = core::array::from_fn(|i| i as u8); // 8 bit sawtooth

    let mut transfer = i2s_rx.read_dma_circular(&mut rx_buffer).unwrap();

    loop {
        match transfer.available() {
            Ok(avail) => {
                if avail > 0 {
                    let mut rcv = [0u8; 5000];
                    match transfer.pop(&mut rcv[..avail]) {
                        Ok(count) => {
                            //rcv[..count].iter().for_each(|x| println!("{:b} ", x));
                            //println!("received {} bytes", count);
                        }
                        Err(_) => {}
                    }
                }
            }
            Err(e) => {
                println!("err: {:?}", e);
            }
        };
    }
}
