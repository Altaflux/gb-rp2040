use display::GameboyLineBufferDisplay;
use gb_core::gameboy::GameBoy;

pub mod display;
pub mod rom;

pub struct GameEmulationHandler<'a, 'b> {
    gameboy: &'a mut GameBoy<'b, GameboyLineBufferDisplay>,
    current_line_index: usize,
}
impl<'a, 'b> GameEmulationHandler<'a, 'b> {
    pub fn new(gameboy: &'a mut GameBoy<'b, GameboyLineBufferDisplay>) -> Self {
        Self {
            gameboy: gameboy,
            current_line_index: 0,
        }
    }
}

impl<'a, 'b> Iterator for GameEmulationHandler<'a, 'b> {
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
