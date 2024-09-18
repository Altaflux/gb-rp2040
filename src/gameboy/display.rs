use alloc::boxed::Box;
use gb_core::{gameboy::GameBoy, hardware::Screen};

pub struct GameboyLineBufferDisplay {
    pub line_buffer: Box<[u16; 160]>,
    pub line_complete: bool,
    pub turn_off: bool,
}

impl GameboyLineBufferDisplay {
    pub fn new() -> Self {
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

pub struct GameVideoIter<'a, 'b> {
    gameboy: &'a mut GameBoy<'b, GameboyLineBufferDisplay>,
    current_line_index: usize,
}
impl<'a, 'b> GameVideoIter<'a, 'b> {
    pub fn new(gameboy: &'a mut GameBoy<'b, GameboyLineBufferDisplay>) -> Self {
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
