use crate::hal::timer::Instant;

use gb_core::hardware::Screen;
const NANOS_IN_VSYNC: u64 = ((1.0 / 60.0) * 1000000000.0) as u64;
pub struct GameboyLineBufferDisplay {
    pub line_buffer: [u16; 160],
    pub line_complete: bool,
    pub turn_off: bool,
    time_counter: Instant,
    delay: crate::hal::Timer,
}

impl GameboyLineBufferDisplay {
    pub fn new(delay: crate::hal::Timer) -> Self {
        Self {
            line_buffer: [0; 160],
            line_complete: false,
            turn_off: false,
            time_counter: delay.get_counter(),
            delay: delay,
        }
    }
}

impl Screen for GameboyLineBufferDisplay {
    fn turn_on(&mut self) {
        self.time_counter = self.delay.get_counter();
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

    fn draw(&mut self, _: bool) {
        let current_time = self.delay.get_counter();
        let diff = current_time - self.time_counter;
        let nano_seconds = diff.to_nanos();
        if NANOS_IN_VSYNC > nano_seconds {
            let time_delay = NANOS_IN_VSYNC.saturating_sub(nano_seconds) as u32;
            //self.delay.delay_ns(time_delay);
        }
        self.time_counter = self.delay.get_counter();
    }

    fn frame_rate(&self) -> u8 {
        30
    }
}
