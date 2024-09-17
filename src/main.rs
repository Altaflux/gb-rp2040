//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
use core::ops;

use alloc::boxed::Box;
use alloc::vec::{self, Vec};
use defmt::*;
use defmt_rtt as _;

use embedded_hal::digital::OutputPin;
use embedded_sdmmc::{sdcard, SdCard, VolumeManager};
use gb_core::hardware::rom::Rom;
use gb_core::hardware::rom::RomManager;
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

use hal::{clocks::init_clocks_and_plls, pac, sio::Sio, spi, spi::Spi, watchdog::Watchdog};

use gb_core::{gameboy::GameBoy, hardware::Screen};
mod array_scaler;
mod dma_transfer;
mod pio_interface;
mod rp_hal;
mod scaler;
mod spi_device;
mod stream_display;
//

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 180000;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    }

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
    let reset = pins.gpio2.into_push_pull_output();
    let mut cs = pins.gpio27.into_push_pull_output();
    let rs = pins.gpio28.into_push_pull_output();
    let rw = pins.gpio22.into_function::<hal::gpio::FunctionPio0>();
    let mut rd = pins.gpio26.into_push_pull_output();

    let _ = pins.gpio6.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio7.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio8.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio9.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio10.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio11.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio12.into_function::<hal::gpio::FunctionPio0>();
    let _ = pins.gpio13.into_function::<hal::gpio::FunctionPio0>();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    rd.set_high().unwrap();
    cs.set_low().unwrap();

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
    let exclusive_spi = spi_device::ExclusiveDevice::new(spi, spi_cs, timer).unwrap();
    let sdcard = SdCard::new(exclusive_spi, timer);
    let mut volume_mgr = VolumeManager::new(sdcard, DummyTimesource::default());

    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    let mut root_dir = volume0.open_root_dir().unwrap();
    let mut my_file = root_dir
        .open_file_in_dir("tetris.gb", embedded_sdmmc::Mode::ReadOnly)
        .unwrap();

    // let gb_rom = gb_core::hardware::rom::Rom::from_bytes_two(my_vec.as_slice());
    let roms = SdRomManager::new(my_file);
    let gb_rom = gb_core::hardware::rom::Rom::from_bytes(roms);
    let fk = gb_rom.into_cartridge();
    ///////////////////////////////
    let interface =
        pio_interface::PioInterface::new(3, rs, &mut pio, sm0, rw.id().num, (6, 13), endianess);

    let mut display = ili9341::Ili9341::new_orig(
        interface,
        reset,
        &mut timer,
        ili9341::Orientation::Landscape,
        ili9341::DisplaySize240x320,
    )
    .unwrap();

    //let gb_rom = load_rom_from_path();
    //   let cart = gb_rom.into_cartridge();
    let boot_rom = gb_core::hardware::boot_rom::Bootrom::new(Some(
        gb_core::hardware::boot_rom::BootromData::from_bytes(include_bytes!(
            "C:\\roms\\dmg_boot.bin"
        )),
    ));
    let screen = GameboyLineBufferDisplay::new();
    let mut gameboy = GameBoy::create(screen, fk, boot_rom);

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
                        streamer.stream::<_, _>(
                            sm,
                            &mut scaler.scale_iterator(GameVideoIter::new(&mut gameboy)),
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

pub struct GameVideoIter<'a, 'b> {
    gameboy: &'a mut GameBoy<'b, GameboyLineBufferDisplay>,
    current_line_index: usize,
}
impl<'a, 'b> GameVideoIter<'a, 'b> {
    fn new(gameboy: &'a mut GameBoy<'b, GameboyLineBufferDisplay>) -> Self {
        Self {
            gameboy: gameboy,
            current_line_index: 0,
        }
    }
}

impl<'a, 'b> Iterator for GameVideoIter<'a, 'b> {
    type Item = u16;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.gameboy.get_screen().turn_off {
                self.gameboy.get_screen().turn_off = false;
                return None;
            }
            if self.gameboy.get_screen().line_complete {
                let pixel = self.gameboy.get_screen().line_buffer[self.current_line_index];
                if self.current_line_index + 1 >= 160 {
                    self.current_line_index = 0;
                    self.gameboy.get_screen().line_complete = false;
                } else {
                    self.current_line_index = self.current_line_index + 1;
                }

                return Some(pixel);
            } else {
                self.gameboy.tick();
            }
        }
    }
}

struct GameboyLineBufferDisplay {
    line_buffer: Box<[u16; 160]>,
    line_complete: bool,
    turn_off: bool,
}

impl GameboyLineBufferDisplay {
    fn new() -> Self {
        Self {
            line_buffer: Box::new([0; 160]),
            line_complete: false,
            turn_off: false,
        }
    }
}

impl Screen for GameboyLineBufferDisplay {
    fn turn_on(&mut self) {
        self.turn_off = true;
    }

    fn turn_off(&mut self) {
        //todo!()
    }

    fn set_pixel(&mut self, x: u8, _y: u8, color: gb_core::hardware::color_palette::Color) {
        let encoded_color = ((color.red as u16 & 0b11111000) << 8)
            + ((color.green as u16 & 0b11111100) << 3)
            + (color.blue as u16 >> 3);

        self.line_buffer[x as usize] = encoded_color;
    }
    fn scanline_complete(&mut self, _y: u8, _skip: bool) {
        self.line_complete = true;
    }

    fn draw(&mut self, _: bool) {}
}

// pub fn load_rom_from_path<'a>() -> gb_core::hardware::rom::Rom<'a> {
//     let rom_f = include_bytes!("C:\\roms\\tetris.gb");
//     gb_core::hardware::rom::Rom::from_bytes(rom_f)
// }

// pub fn load_rom<
//     'a,
//     D: embedded_sdmmc::BlockDevice,
//     T: embedded_sdmmc::TimeSource,
//     const MAX_DIRS: usize,
//     const MAX_FILES: usize,
//     const MAX_VOLUMES: usize,
// >(file: ) -> impl gb_core::hardware::rom::RomManager {
// }

// End of file

#[derive(Default)]
pub struct DummyTimesource();

impl embedded_sdmmc::TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

struct SdRomManager<
    'a,
    D: embedded_sdmmc::BlockDevice,
    T: embedded_sdmmc::TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> {
    file: embedded_sdmmc::File<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    bank_0: Box<[u8; 0x4000]>,
    active_bank: Box<[u8; 0x4000]>,
    current_bank: u8,
}
impl<
        'a,
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    > SdRomManager<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    fn new(mut file: embedded_sdmmc::File<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>) -> Self {
        let mut bank_0 = Box::new([0u8; 0x4000]);
        file.seek_from_start(0u32).unwrap();
        file.read(&mut *bank_0).unwrap();

        let mut result = Self {
            active_bank: Box::new([0u8; 0x4000]),
            bank_0: bank_0,
            current_bank: 1,
            file: file,
        };

        result.set_active_bank(1);
        result
    }

    pub fn compare(value: u16, from: u16, to: u16) -> isize {
        if value < from {
            return value as isize - from as isize;
        } else if value > to {
            return value as isize - to as isize;
        }
        return 0;
    }
}
impl<
        'a,
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    > gb_core::hardware::rom::RomManager
    for SdRomManager<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    fn set_active_bank(&mut self, bank: u8) {
        let start = 0x4000 + ((bank as u16 - 1) * (0x7FFF - 0x4000 + 1));

        self.file.seek_from_start(start as u32).unwrap();
        self.file.read(&mut *self.active_bank).unwrap();
        self.current_bank = bank;
    }
}
impl<
        'a,
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    > core::ops::Index<usize> for SdRomManager<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    type Output = u8;

    fn index(&self, index: usize) -> &Self::Output {
        if Self::compare(index as u16, 0x4000, 0x7FFF) == 0 {
            info!("Read from bank 1: {}", index);
            return &self.active_bank[index - 0x4000];
        }
        //info!("Read from bank 0");
        &self.bank_0[index as usize]
    }
}
impl<
        'a,
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    > core::ops::Index<core::ops::Range<usize>>
    for SdRomManager<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    type Output = [u8];

    fn index(&self, index: core::ops::Range<usize>) -> &Self::Output {
        return &self.bank_0[index];
    }
}
