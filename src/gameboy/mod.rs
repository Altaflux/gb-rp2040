use core::{convert::Infallible, marker::PhantomData};

use display::GameboyLineBufferDisplay;
use embedded_hal::digital::InputPin;
use gb_core::{gameboy::GameBoy, hardware::Screen};

pub mod display;
pub mod rom;

pub trait GameboyButtonHandler<'a> {
    fn handle_button_clicks<SC: Screen>(&mut self, gameboy: &mut GameBoy<'a, SC>);
}

pub struct GameEmulationHandler<'a, 'b, 'c, BH: GameboyButtonHandler<'c>> {
    gameboy: &'a mut GameBoy<'b, GameboyLineBufferDisplay>,
    current_line_index: usize,
    button_handler: &'a mut BH,
    _marker: PhantomData<&'c ()>,
}
impl<'a, 'b, 'c, BH: GameboyButtonHandler<'c>> GameEmulationHandler<'a, 'b, 'c, BH> {
    pub fn new(
        gameboy: &'a mut GameBoy<'b, GameboyLineBufferDisplay>,
        button_handler: &'a mut BH,
    ) -> Self {
        Self {
            gameboy: gameboy,
            current_line_index: 0,
            button_handler,
            _marker: PhantomData,
        }
    }
}

impl<'a, 'b, 'c, BH: GameboyButtonHandler<'c>> Iterator for GameEmulationHandler<'a, 'b, 'c, BH>
where
    'b: 'c,
    'c: 'b,
{
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
                    self.button_handler.handle_button_clicks(&mut self.gameboy);
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

pub struct InputButtonMapper<'a> {
    a_button: &'a mut dyn InputPin<Error = Infallible>,
    b_button: &'a mut dyn InputPin<Error = Infallible>,
    start_button: &'a mut dyn InputPin<Error = Infallible>,
    select_button: &'a mut dyn InputPin<Error = Infallible>,
    up_button: &'a mut dyn InputPin<Error = Infallible>,
    down_button: &'a mut dyn InputPin<Error = Infallible>,
    left_button: &'a mut dyn InputPin<Error = Infallible>,
    right_button: &'a mut dyn InputPin<Error = Infallible>,
    a_button_state: bool,
    b_button_state: bool,
    start_button_state: bool,
    select_button_state: bool,
    up_button_state: bool,
    down_button_state: bool,
    left_button_state: bool,
    right_button_state: bool,
}

impl<'a, 'b> GameboyButtonHandler<'b> for InputButtonMapper<'a> {
    #[inline(always)]
    fn handle_button_clicks<SC: Screen>(&mut self, gameboy: &mut GameBoy<'b, SC>) {
        ////
        if self.b_button.is_high().unwrap() {
            if self.b_button_state == false {
                //  info!("B pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::B);
                self.b_button_state = true;
            }
        } else {
            if self.b_button_state == true {
                //   info!("B released");
                gameboy.key_released(gb_core::hardware::input::Button::B);
                self.b_button_state = false;
            }
        }
        ////
        if self.a_button.is_high().unwrap() {
            if self.a_button_state == false {
                //  info!("A pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::A);
                self.a_button_state = true;
            }
        } else {
            if self.a_button_state == true {
                // info!("A released");
                gameboy.key_released(gb_core::hardware::input::Button::A);
                self.a_button_state = false;
            }
        }
        ////
        if self.select_button.is_high().unwrap() {
            if self.select_button_state == false {
                // info!("Select pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::SELECT);
                self.select_button_state = true;
            }
        } else {
            if self.select_button_state == true {
                //  info!("Select released");
                gameboy.key_released(gb_core::hardware::input::Button::SELECT);
                self.select_button_state = false;
            }
        }
        /////
        if self.start_button.is_high().unwrap() {
            if self.start_button_state == false {
                //   info!("Start pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::START);
                self.start_button_state = true;
            }
        } else {
            if self.start_button_state == true {
                //  info!("Start released");
                gameboy.key_released(gb_core::hardware::input::Button::START);
                self.start_button_state = false;
            }
        }
        /////
        if self.up_button.is_high().unwrap() {
            if self.up_button_state == false {
                //  info!("Up pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::UP);
                self.up_button_state = true;
            }
        } else {
            if self.up_button_state == true {
                //  info!("Up released");
                gameboy.key_released(gb_core::hardware::input::Button::UP);
                self.up_button_state = false;
            }
        }
        /////
        if self.down_button.is_high().unwrap() {
            if self.down_button_state == false {
                // info!("Down pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::DOWN);
                self.down_button_state = true;
            }
        } else {
            if self.down_button_state == true {
                // info!("Down released");
                gameboy.key_released(gb_core::hardware::input::Button::DOWN);
                self.down_button_state = false;
            }
        }
        /////
        if self.left_button.is_high().unwrap() {
            if self.left_button_state == false {
                //  info!("Left pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::LEFT);
                self.left_button_state = true;
            }
        } else {
            if self.left_button_state == true {
                //   info!("Left released");
                gameboy.key_released(gb_core::hardware::input::Button::LEFT);
                self.left_button_state = false;
            }
        }
        /////
        if self.right_button.is_high().unwrap() {
            if self.right_button_state == false {
                //   info!("Right pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::RIGHT);
                self.right_button_state = true;
            }
        } else {
            if self.right_button_state == true {
                //   info!("Right released");
                gameboy.key_released(gb_core::hardware::input::Button::RIGHT);
                self.right_button_state = false;
            }
        }
    }
}
impl<'a> InputButtonMapper<'a> {
    pub fn new(
        a_button: &'a mut dyn InputPin<Error = Infallible>,
        b_button: &'a mut dyn InputPin<Error = Infallible>,
        start_button: &'a mut dyn InputPin<Error = Infallible>,
        select_button: &'a mut dyn InputPin<Error = Infallible>,
        up_button: &'a mut dyn InputPin<Error = Infallible>,
        down_button: &'a mut dyn InputPin<Error = Infallible>,
        left_button: &'a mut dyn InputPin<Error = Infallible>,
        right_button: &'a mut dyn InputPin<Error = Infallible>,
    ) -> Self {
        Self {
            b_button,
            a_button,
            select_button,
            start_button,
            up_button,
            down_button,
            left_button,
            right_button,
            a_button_state: false,
            b_button_state: false,
            select_button_state: false,
            start_button_state: false,
            up_button_state: false,
            down_button_state: false,
            left_button_state: false,
            right_button_state: false,
        }
    }
}

//////////////////
///
///

pub struct InputButtonMapper2<
    'a,
    A: InputPin<Error = Infallible>,
    B: InputPin<Error = Infallible>,
    START: InputPin<Error = Infallible>,
    SEL: InputPin<Error = Infallible>,
    UP: InputPin<Error = Infallible>,
    DOWN: InputPin<Error = Infallible>,
    LEFT: InputPin<Error = Infallible>,
    RIGHT: InputPin<Error = Infallible>,
> {
    a_button: &'a mut A,
    b_button: &'a mut B,
    start_button: &'a mut START,
    select_button: &'a mut SEL,
    up_button: &'a mut UP,
    down_button: &'a mut DOWN,
    left_button: &'a mut LEFT,
    right_button: &'a mut RIGHT,
    a_button_state: bool,
    b_button_state: bool,
    start_button_state: bool,
    select_button_state: bool,
    up_button_state: bool,
    down_button_state: bool,
    left_button_state: bool,
    right_button_state: bool,
}
impl<
        'a,
        A: InputPin<Error = Infallible>,
        B: InputPin<Error = Infallible>,
        START: InputPin<Error = Infallible>,
        SEL: InputPin<Error = Infallible>,
        UP: InputPin<Error = Infallible>,
        DOWN: InputPin<Error = Infallible>,
        LEFT: InputPin<Error = Infallible>,
        RIGHT: InputPin<Error = Infallible>,
    > InputButtonMapper2<'a, A, B, START, SEL, UP, DOWN, LEFT, RIGHT>
{
    pub fn new(
        a_button: &'a mut A,
        b_button: &'a mut B,
        start_button: &'a mut START,
        select_button: &'a mut SEL,
        up_button: &'a mut UP,
        down_button: &'a mut DOWN,
        left_button: &'a mut LEFT,
        right_button: &'a mut RIGHT,
    ) -> Self {
        Self {
            b_button,
            a_button,
            select_button,
            start_button,
            up_button,
            down_button,
            left_button,
            right_button,
            a_button_state: false,
            b_button_state: false,
            select_button_state: false,
            start_button_state: false,
            up_button_state: false,
            down_button_state: false,
            left_button_state: false,
            right_button_state: false,
        }
    }
}

impl<
        'a,
        'b,
        A: InputPin<Error = Infallible>,
        B: InputPin<Error = Infallible>,
        START: InputPin<Error = Infallible>,
        SEL: InputPin<Error = Infallible>,
        UP: InputPin<Error = Infallible>,
        DOWN: InputPin<Error = Infallible>,
        LEFT: InputPin<Error = Infallible>,
        RIGHT: InputPin<Error = Infallible>,
    > GameboyButtonHandler<'b> for InputButtonMapper2<'a, A, B, START, SEL, UP, DOWN, LEFT, RIGHT>
{
    #[inline(always)]
    fn handle_button_clicks<SC: Screen>(&mut self, gameboy: &mut GameBoy<'b, SC>) {
        if self.b_button.is_high().unwrap() {
            if self.b_button_state == false {
                //  info!("B pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::B);
                self.b_button_state = true;
            }
        } else {
            if self.b_button_state == true {
                //   info!("B released");
                gameboy.key_released(gb_core::hardware::input::Button::B);
                self.b_button_state = false;
            }
        }
        ////
        if self.a_button.is_high().unwrap() {
            if self.a_button_state == false {
                //  info!("A pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::A);
                self.a_button_state = true;
            }
        } else {
            if self.a_button_state == true {
                // info!("A released");
                gameboy.key_released(gb_core::hardware::input::Button::A);
                self.a_button_state = false;
            }
        }
        ////
        if self.select_button.is_high().unwrap() {
            if self.select_button_state == false {
                // info!("Select pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::SELECT);
                self.select_button_state = true;
            }
        } else {
            if self.select_button_state == true {
                //  info!("Select released");
                gameboy.key_released(gb_core::hardware::input::Button::SELECT);
                self.select_button_state = false;
            }
        }
        /////
        if self.start_button.is_high().unwrap() {
            if self.start_button_state == false {
                //   info!("Start pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::START);
                self.start_button_state = true;
            }
        } else {
            if self.start_button_state == true {
                //  info!("Start released");
                gameboy.key_released(gb_core::hardware::input::Button::START);
                self.start_button_state = false;
            }
        }
        /////
        if self.up_button.is_high().unwrap() {
            if self.up_button_state == false {
                //  info!("Up pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::UP);
                self.up_button_state = true;
            }
        } else {
            if self.up_button_state == true {
                //  info!("Up released");
                gameboy.key_released(gb_core::hardware::input::Button::UP);
                self.up_button_state = false;
            }
        }
        /////
        if self.down_button.is_high().unwrap() {
            if self.down_button_state == false {
                // info!("Down pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::DOWN);
                self.down_button_state = true;
            }
        } else {
            if self.down_button_state == true {
                // info!("Down released");
                gameboy.key_released(gb_core::hardware::input::Button::DOWN);
                self.down_button_state = false;
            }
        }
        /////
        if self.left_button.is_high().unwrap() {
            if self.left_button_state == false {
                //  info!("Left pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::LEFT);
                self.left_button_state = true;
            }
        } else {
            if self.left_button_state == true {
                //   info!("Left released");
                gameboy.key_released(gb_core::hardware::input::Button::LEFT);
                self.left_button_state = false;
            }
        }
        /////
        if self.right_button.is_high().unwrap() {
            if self.right_button_state == false {
                //   info!("Right pressed");
                gameboy.key_pressed(gb_core::hardware::input::Button::RIGHT);
                self.right_button_state = true;
            }
        } else {
            if self.right_button_state == true {
                //   info!("Right released");
                gameboy.key_released(gb_core::hardware::input::Button::RIGHT);
                self.right_button_state = false;
            }
        }
    }
}
