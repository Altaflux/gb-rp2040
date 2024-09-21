//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use alloc::boxed::Box;
use alloc::vec::Vec;
use defmt::*;
use defmt_rtt as _;

use embedded_hal::digital::OutputPin;
use embedded_sdmmc::{SdCard, VolumeManager};
use gameboy::display::{GameVideoIter, GameboyLineBufferDisplay};
use ili9341::{DisplaySize, DisplaySize240x320};
use panic_probe as _;

use hal::fugit::RateExtU32;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::{self as hal, pio::PIOExt};
use rp2040_hal::{entry, Clock};

use rp_pico;
extern crate alloc;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.

use embedded_alloc::Heap;

use hal::{clocks::init_clocks_and_plls, pac, sio::Sio, spi, watchdog::Watchdog};

use gb_core::gameboy::GameBoy;
use util::DummyOutputPin;
mod array_scaler;
mod dma_transfer;
mod gameboy;
mod pio_interface;
mod rp_hal;
mod scaler;
mod sdcard;
mod stream_display;
mod util;
//

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 203000;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    }
    info!("--------------------");
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    //  hal::clocks::RtcClock::
    pac.VREG_AND_CHIP_RESET
        .vreg()
        .write(|w| unsafe { w.vsel().bits(0b1101) });
    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    //let mut led_pin = pins.led.into_push_pull_output();

    let rs = pins.gpio28.into_push_pull_output();
    let rw = pins.gpio27.into_function::<hal::gpio::FunctionPio0>();

    let _ = pins.gpio3.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio4.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio5.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio6.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio7.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio8.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio9.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio10.into_function::<hal::gpio::FunctionPio0>();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    ///////////////////////////////SD CARD
    let spi_sclk: hal::gpio::Pin<_, _, hal::gpio::PullDown> =
        pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi: hal::gpio::Pin<_, _, hal::gpio::PullDown> =
        pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
    let spi_cs = pins.gpio17.into_push_pull_output();
    let spi_miso: hal::gpio::Pin<_, _, hal::gpio::PullDown> =
        pins.gpio16.into_function::<hal::gpio::FunctionSpi>();

    // Create the SPI driver instance for the SPI0 device
    let spi = spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        400.kHz(), // card initialization happens at low baud rate
        embedded_hal::spi::MODE_0,
    );
    let exclusive_spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, spi_cs, timer).unwrap();
    let sdcard = SdCard::new(exclusive_spi, timer);
    let mut volume_mgr = VolumeManager::new(sdcard, sdcard::DummyTimesource::default());

    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    let mut root_dir = volume0.open_root_dir().unwrap();

    //Read boot rom
    let mut boot_rom_file = root_dir
        .open_file_in_dir("dmg_boot.bin", embedded_sdmmc::Mode::ReadOnly)
        .unwrap();
    let mut boot_rom_data = Box::new([0u8; 0x100]);
    boot_rom_file.read(&mut *boot_rom_data).unwrap();
    boot_rom_file.close().unwrap();

    let rom_file = root_dir
        .open_file_in_dir("sml.gb", embedded_sdmmc::Mode::ReadOnly)
        .unwrap();

    let roms = gameboy::rom::SdRomManager::new(rom_file);
    let gb_rom = gb_core::hardware::rom::Rom::from_bytes(roms);
    let cartridge = gb_rom.into_cartridge();
    ///////////////////////////////
    let interface =
        pio_interface::PioInterface::new(3, rs, &mut pio, sm0, rw.id().num, (3, 10), endianess);

    let mut display = ili9341::Ili9341::new_orig(
        interface,
        DummyOutputPin,
        &mut timer,
        ili9341::Orientation::Landscape,
        ili9341::DisplaySize240x320,
    )
    .unwrap();

    let boot_rom = gb_core::hardware::boot_rom::Bootrom::new(Some(
        gb_core::hardware::boot_rom::BootromData::from_bytes(&*boot_rom_data),
    ));
    core::mem::drop(boot_rom_data);
    let screen = GameboyLineBufferDisplay::new();
    let mut gameboy = GameBoy::create(
        screen,
        cartridge,
        boot_rom,
        Box::new(gameboy::audio::NullAudioPlayer),
    );

    const SCREEN_WIDTH: usize =
        (<DisplaySize240x320 as DisplaySize>::WIDTH as f32 / 1.0f32) as usize;
    const SCREEN_HEIGHT: usize =
        (<DisplaySize240x320 as DisplaySize>::HEIGHT as f32 / 1.0f32) as usize;

    let spare: &'static mut [u16] =
        cortex_m::singleton!(: Vec<u16>  = alloc::vec![0; SCREEN_WIDTH  ])
            .unwrap()
            .as_mut_slice();

    let dm_spare: &'static mut [u16] =
        cortex_m::singleton!(: Vec<u16>  = alloc::vec![0; SCREEN_WIDTH ])
            .unwrap()
            .as_mut_slice();

    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut streamer = stream_display::Streamer::new(dma.ch0, dm_spare, spare);
    let scaler: scaler::ScreenScaler<144, 160, { SCREEN_WIDTH }, { SCREEN_HEIGHT }> =
        scaler::ScreenScaler::new();

    loop {
        let start_time = timer.get_counter();
        display = display
            .async_transfer_mode(
                0,
                0,
                (SCREEN_HEIGHT - 1) as u16,
                (SCREEN_WIDTH - 1) as u16,
                |iface| {
                    iface.transfer_16bit_mode(|sm| {
                        streamer.stream::<_, _, _, _, 1>(
                            sm,
                            &mut scaler.scale_iterator(GameVideoIter::new(&mut gameboy)),
                            |d| [d],
                        )
                    })
                },
            )
            .unwrap();

        let end_time = timer.get_counter();
        let diff = end_time - start_time;
        let milliseconds = diff.to_millis();
        info!(
            "Time elapsed: {}:{}",
            milliseconds / 1000,
            milliseconds % 1000
        );
        info!("Free Mem: {}", ALLOCATOR.free());
        info!("Used Mem: {}", ALLOCATOR.used());
    }
}
#[inline(always)]
const fn endianess(be: bool, val: u16) -> u16 {
    if be {
        val.to_le()
    } else {
        val.to_be()
    }
}
