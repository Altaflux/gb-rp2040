use crate::rp_hal::hal;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_hal::digital::OutputPin;
use hal::dma::{HalfWord, Word};
use hal::pio::{PIOExt, PIO};
use hal::pio::{Running, StateMachine, StateMachineIndex, Tx};
use hal::pio::{Rx, UninitStateMachine};
type Result = core::result::Result<(), DisplayError>;
use hal::dma::Byte;

pub struct I2sPioInterface<RS, P: PIOExt, SM: StateMachineIndex> {
    sm: StateMachine<(P, SM), Running>,
    tx: Tx<(P, SM), HalfWord>,
    rx: Rx<(P, SM)>,
    rs: RS,
}

impl<RS, P, SM> I2sPioInterface<RS, P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
    RS: OutputPin,
{
    pub fn new(
        clock_divider: u16,
        rs: RS,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        clock_pin: (u8, u8),
        pins: (u8, u8),
    ) -> Self {
        let audio_program = pio_proc::pio_file!("src/audio_i2s.pio");

        let video_program_installed = pio.install(&audio_program.program).unwrap();
        let program_offset = video_program_installed.offset();

        let (mut video_sm, rx, vid_tx) =
            hal::pio::PIOBuilder::from_installed_program(video_program_installed)
                .out_pins(pins.0, (1 - pins.0) + pins.1)
                .side_set_pin_base(clock_pin.0)
                .out_shift_direction(hal::pio::ShiftDirection::Left)
                .autopull(true)
                .pull_threshold(32)
                .buffers(hal::pio::Buffers::OnlyTx)
                .clock_divisor_fixed_point(clock_divider, 0)
                .build(sm);
        video_sm.set_pindirs((pins.0..pins.1 + 1 as u8).map(|n| (n, hal::pio::PinDir::Output)));
        video_sm.set_pindirs(
            (clock_pin.0..clock_pin.1 + 1 as u8).map(|n| (n, hal::pio::PinDir::Output)),
        );

        let mut interface = Self {
            rs: rs,
            sm: video_sm.start(),
            rx: rx,
            tx: vid_tx.transfer_size(HalfWord),
        };

        let instruction = pio::Instruction {
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: program_offset + audio_program.public_defines.entry_point as u8,
            },
            delay: 0,
            side_set: None,
        };
        interface.sm.exec_instruction(instruction);
        interface
    }

    pub fn transfer_16bit_mode<F>(mut self, mut callback: F) -> Self
    where
        F: FnMut(Tx<(P, SM), HalfWord>) -> Tx<(P, SM), HalfWord>,
    {
        let interface = (callback)(self.tx);
        self.tx = interface;
        self
    }

    pub fn transfer_8bit_mode<F>(mut self, mut callback: F) -> Self
    where
        F: FnMut(Tx<(P, SM), Byte>) -> Tx<(P, SM), Byte>,
    {
        let interface = (callback)(self.tx.transfer_size(Byte));
        self.tx = interface.transfer_size(HalfWord);
        self
    }

    #[allow(dead_code)]
    pub fn free(self, pio: &mut PIO<P>) -> (UninitStateMachine<(P, SM)>, RS) {
        let (sm, prg) = self.sm.uninit(self.rx, self.tx);
        pio.uninstall(prg);
        (sm, self.rs)
    }
}
