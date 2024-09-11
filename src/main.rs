//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
#![feature(const_float_bits_conv)]
use alloc::vec::Vec;
use defmt::*;
use defmt_rtt as _;

use embedded_hal::digital::OutputPin;
use ili9341::{DisplaySize, DisplaySize240x320};
use panic_probe as _;

use rp2040_hal::dma::DMAExt;
use rp2040_hal::entry;
use rp2040_hal::{self as hal, pio::PIOExt};
use rp_pico;
extern crate alloc;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.

use hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};

use embedded_alloc::Heap;

use gb_core::{gameboy::GameBoy, hardware::Screen};
mod array_scaler;
mod const_math;
mod dma_transfer;
mod pio_interface;
mod rp_hal;
mod scaler;
mod stream_display;
//

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 131000;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    }

    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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
    let mut cs = pins.gpio17.into_push_pull_output();
    let rs = pins.gpio28.into_push_pull_output();
    let rw = pins.gpio22.into_function::<hal::gpio::FunctionPio0>();
    let mut rd = pins.gpio16.into_push_pull_output();

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
    let endianess = |be: bool, val: u16| {
        if be {
            val.to_le()
        } else {
            val.to_be()
        }
    };

    let interface =
        pio_interface::PioInterface::new(3, rs, &mut pio, sm0, rw.id().num, (6, 13), endianess);

    let mut display = ili9341::Ili9341::new_orig(
        interface,
        reset,
        &mut timer,
        ili9341::Orientation::LandscapeFlipped,
        ili9341::DisplaySize240x320,
    )
    .unwrap();

    let gb_rom = load_rom_from_path();
    let cart = gb_rom.into_cartridge();
    let boot_rom = gb_core::hardware::boot_rom::Bootrom::new(Some(
        gb_core::hardware::boot_rom::BootromData::from_bytes(include_bytes!(
            "C:\\roms\\dmg_boot.bin"
        )),
    ));
    let screen = GameboyLineBufferDisplay::new();
    let mut gameboy = GameBoy::create(screen, cart, boot_rom);

    // let SCREEN_WIDTH: usize =
    //     num_traits::Float::floor(<DisplaySize240x320 as DisplaySize>::WIDTH as f32 / 1.0f32)
    //         as usize;
    // let SCREEN_HEIGHT: usize =
    //     num_traits::Float::floor(<DisplaySize240x320 as DisplaySize>::HEIGHT as f32 / 1.0f32)
    //         as usize;

    const SCREEN_WIDTH: usize =
        const_math::floorf(<DisplaySize240x320 as DisplaySize>::WIDTH as f32 / 1.0f32) as usize;
    const SCREEN_HEIGHT: usize =
        const_math::floorf(<DisplaySize240x320 as DisplaySize>::HEIGHT as f32 / 1.0f32) as usize;

    let spare: &'static mut [u16] =
        cortex_m::singleton!(: Vec<u16>  = alloc::vec![0; SCREEN_WIDTH ])
            .unwrap()
            .as_mut_slice();

    let dm_spare: &'static mut [u16] =
        cortex_m::singleton!(: Vec<u16>  = alloc::vec![0; SCREEN_WIDTH ])
            .unwrap()
            .as_mut_slice();

    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut streamer = stream_display::Streamer::new(dma.ch0, dm_spare, spare, SCREEN_WIDTH);
    let mut scaler: scaler::ScreenScaler<144, 160, { SCREEN_WIDTH }, { SCREEN_HEIGHT }> =
        scaler::ScreenScaler::new(SCREEN_HEIGHT as u16, SCREEN_WIDTH as u16);

    loop {
        let mut dis = scaler.clone();
        display = display
            .async_transfer_mode(0, 0, SCREEN_HEIGHT as u16, SCREEN_WIDTH as u16, |iface| {
                iface.transfer_16bit_mode(|sm| {
                    let display_iter = GameVideoIter::new(&mut gameboy);
                    let mut foo = dis.get_iter(display_iter);
                    streamer.stream::<SCREEN_WIDTH, _, _>(sm, &mut foo)
                })
            })
            .unwrap();
    }
}

pub struct GameVideoIter<'a> {
    gameboy: &'a mut GameBoy<GameboyLineBufferDisplay>,
    current_line_index: usize,
}
impl<'a> GameVideoIter<'a> {
    fn new(gameboy: &'a mut GameBoy<GameboyLineBufferDisplay>) -> Self {
        Self {
            gameboy: gameboy,
            current_line_index: 0,
        }
    }
}

impl<'a> Iterator for GameVideoIter<'a> {
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
    line_buffer: Vec<u16>,
    line_complete: bool,
    turn_off: bool,
}

impl GameboyLineBufferDisplay {
    fn new() -> Self {
        Self {
            line_buffer: alloc::vec![0; 160],
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

pub fn load_rom_from_path() -> gb_core::hardware::rom::Rom<'static> {
    let rom_f = include_bytes!("C:\\roms\\tetris.gb");
    gb_core::hardware::rom::Rom::from_bytes(rom_f)
}

// End of file
