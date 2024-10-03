//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::u16;

use alloc::boxed::Box;
use alloc::vec::Vec;
use defmt::*;
use defmt_rtt as _;

use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::{IntoStorage, Point, RgbColor, Size, WebColors};
use embedded_graphics::primitives::Rectangle;
use embedded_hal::delay::DelayNs;
use embedded_sdmmc::{SdCard, VolumeManager};
use gameboy::display::GameboyLineBufferDisplay;
//use i2s::I2sPioInterface;
use embedded_graphics_core::draw_target::DrawTarget;
use gameboy::GameEmulationHandler;
use hal::fugit::RateExtU32;

use hardware::display::ScreenScaler;
use ili9341::{DisplaySize, DisplaySize240x320};
use panic_probe as _;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::{self as hal, pio::PIOExt};
use rp2040_hal::{entry, Clock};
// #[allow(unused_imports)]
// use rp_pico;
extern crate alloc;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.

use embedded_alloc::Heap;

use gb_core::gameboy::GameBoy;
use hal::{pac, sio::Sio, spi, watchdog::Watchdog};

//mod array_scaler;
mod clocks;

mod gameboy;

mod hardware;
mod rp_hal;

mod util;
//

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 160000;
        //const HEAP_SIZE: usize = 220000;
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
        .write(|w| unsafe { w.vsel().bits(0b1110) }); // 1.25v
                                                      // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = clocks::configure_overclock(
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

    let mut timer: rp2040_hal::Timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

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

    let (mut pio_0, sm0_0, sm0_1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let (mut pio_1, sm_1_0, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);

    const SCREEN_WIDTH: usize =
        (<DisplaySize240x320 as DisplaySize>::WIDTH as f32 / 1.0f32) as usize;
    const SCREEN_HEIGHT: usize =
        (<DisplaySize240x320 as DisplaySize>::HEIGHT as f32 / 1.0f32) as usize;

    ///////////////////////////////SD CARD
    /// ///
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
    let mut volume_mgr = VolumeManager::new(sdcard, hardware::sdcard::DummyTimesource::default());

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
        .open_file_in_dir("tetris.gb", embedded_sdmmc::Mode::ReadOnly)
        .unwrap();

    let roms = gameboy::rom::SdRomManager::new(rom_file);
    let gb_rom = gb_core::hardware::rom::Rom::from_bytes(roms);
    let cartridge = gb_rom.into_cartridge();

    ///////////////////////////////
    // let spi_sclk: hal::gpio::Pin<_, _, hal::gpio::PullDown> =
    //     pins.gpio14.into_function::<hal::gpio::FunctionSpi>();
    // let spi_mosi: hal::gpio::Pin<_, _, hal::gpio::PullDown> =
    //     pins.gpio15.into_function::<hal::gpio::FunctionSpi>(); //tx
    // let spi_miso: hal::gpio::Pin<_, _, hal::gpio::PullDown> =
    //     pins.gpio12.into_function::<hal::gpio::FunctionSpi>(); //rx
    // let spi_screen = spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
    let screen_dc: hal::gpio::Pin<
        hal::gpio::bank0::Gpio11,
        hal::gpio::FunctionSio<hal::gpio::SioOutput>,
        hal::gpio::PullDown,
    > = pins.gpio11.into_push_pull_output();
    // let spi_screen = spi_screen.init(
    //     &mut pac.RESETS,
    //     clocks.peripheral_clock.freq(),
    //     80.MHz(), // card initialization happens at low baud rate
    //     embedded_hal::spi::MODE_0,
    // );

    // let exclusive_screen_spi =
    //     spi_device::ExclusiveDevice::new(spi_screen, DummyOutputPin, timer).unwrap();
    // let spi_display_interface =
    //     display_interface_spi::SPIInterface::new(exclusive_screen_spi, screen_dc);

    let spi_sclk: hal::gpio::Pin<_, _, hal::gpio::PullDown> =
        pins.gpio14.into_function::<hal::gpio::FunctionPio0>();
    let spi_mosi: hal::gpio::Pin<_, _, hal::gpio::PullDown> =
        pins.gpio15.into_function::<hal::gpio::FunctionPio0>();

    // /////////////
    //////////DMA

    let spare: &'static mut [u16] =
        cortex_m::singleton!(: [u16;(SCREEN_WIDTH * 3) * 3]  = [0u16; (SCREEN_WIDTH * 3) * 3 ])
            .unwrap()
            .as_mut_slice();

    let streamer = hardware::display::DmaStreamer::new(dma.ch0, dma.ch1, spare);

    let display_interface = hardware::display::SpiPioDmaInterface::new(
        (3, 0),
        screen_dc,
        &mut pio_0,
        sm0_1,
        sm0_0,
        spi_sclk.id().num,
        spi_mosi.id().num,
        streamer,
    );
    // let display_interface = hardware::display::PioInterfaceStreamer::new(
    //     (3, 0),
    //     rs,
    //     &mut pio_0,
    //     sm0_0,
    //     rw.id().num,
    //     (3, 10),
    //     streamer,
    // );
    let display_reset = pins.gpio2.into_push_pull_output();
    let mut display = ili9341::Ili9341::new_orig(
        display_interface,
        display_reset,
        &mut timer,
        ili9341::Orientation::Landscape,
        ili9341::DisplaySize240x320,
    )
    .unwrap();

    let boot_rom = gb_core::hardware::boot_rom::Bootrom::new(Some(
        gb_core::hardware::boot_rom::BootromData::from_bytes(&*boot_rom_data),
    ));
    core::mem::drop(boot_rom_data);

    //////////////////////AUDIO SETUP
    ///
    let sample_rate: u32 = 16000;
    let clock_divider: u32 = 369_000_000 * 4 / sample_rate;

    let int_divider = (clock_divider >> 8) as u16;
    let frak_divider = (clock_divider & 0xFF) as u8;
    info!(
        "Suggested dividers: int: {} frac: {}",
        int_divider, frak_divider
    );

    let clock_frequency = 16000 * 16 * 2;
    let clock_divider = 369_000_000. / clock_frequency as f64 / 2.;
    let int_divider2 = (clock_frequency >> 8) as u16;
    let frak_divider2 = (clock_frequency & 0xFF) as u8;
    info!(
        "Suggested dividers2: int: {} frac: {}",
        int_divider2, frak_divider2
    );
    info!("Suggested dividers: clock_divider: {}", clock_divider);

    let _ = pins.gpio20.into_function::<hal::gpio::FunctionPio1>();
    let _ = pins.gpio21.into_function::<hal::gpio::FunctionPio1>();
    let _ = pins.gpio22.into_function::<hal::gpio::FunctionPio1>();
    let audio_buffer: &'static mut [u16] =
        cortex_m::singleton!(: [u16; (2000 * 3) * 3]  = [0u16;  (2000 * 3) * 3 ])
            .unwrap()
            .as_mut_slice();
    let i2s_interface = hardware::sound::I2sPioInterface::new(
        sample_rate,
        dma.ch2,
        dma.ch3,
        (int_divider as u16, frak_divider as u8),
        &mut pio_1,
        sm_1_0,
        (21, 22),
        20,
        audio_buffer,
    );
    //////////////////////
    let screen = GameboyLineBufferDisplay::new(timer);
    let mut gameboy = GameBoy::create(screen, cartridge, boot_rom, Box::new(i2s_interface));

    let scaler: ScreenScaler<144, 160, { SCREEN_WIDTH }, { SCREEN_HEIGHT }> = ScreenScaler::new();

    let mut loop_counter: usize = 0;
    loop {
        let start_time = timer.get_counter();

        display
            .draw_raw_iter(
                0,
                0,
                (SCREEN_HEIGHT - 1) as u16,
                (SCREEN_WIDTH - 1) as u16,
                scaler.scale_iterator(GameEmulationHandler::new(&mut gameboy)),
            )
            .unwrap();

        let end_time: rp2040_hal::fugit::Instant<u64, 1, 1000000> = timer.get_counter();
        let diff = end_time - start_time;
        let milliseconds = diff.to_millis();
        info!(
            "Loop: {}, Time elapsed: {}:{}",
            loop_counter,
            milliseconds / 1000,
            milliseconds % 1000
        );
        info!("Free Mem: {}", ALLOCATOR.free());
        info!("Used Mem: {}", ALLOCATOR.used());
        loop_counter += 1;
    }
}

struct InfiniteIter;
impl Iterator for InfiniteIter {
    type Item = u16;

    fn next(&mut self) -> Option<Self::Item> {
        Some(Rgb565::GREEN.into_storage())
    }
}
