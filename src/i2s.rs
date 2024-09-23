use crate::rp_hal::hal;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_hal::digital::OutputPin;

use hal::pio::{PIOExt, PIO};
use hal::pio::{Running, StateMachine, StateMachineIndex, Tx};
use hal::pio::{Rx, UninitStateMachine};
type Result = core::result::Result<(), DisplayError>;

use embedded_dma::{ReadBuffer, Word};
use hal::dma::single_buffer::{Config, Transfer};
use hal::dma::Byte;
use hal::dma::SingleChannel;
use hal::dma::{ReadTarget, WriteTarget};
use rp2040_hal::dma::HalfWord;

type ToType<P, SM> = Tx<(P, SM), rp2040_hal::dma::HalfWord>;
enum DmaState<
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = u16>,
    P: PIOExt,
    SM: StateMachineIndex,
> {
    IDLE(CH, FROM, ToType<P, SM>),
    RUNNING(Transfer<CH, FROM, ToType<P, SM>>),
}

pub struct I2sPioInterface<CH: SingleChannel, P: PIOExt, SM: StateMachineIndex> {
    dma_state: Option<DmaState<CH, LimitingArrayReadTarget, P, SM>>,
}

impl<CH, P, SM> I2sPioInterface<CH, P, SM>
where
    CH: SingleChannel,
    P: PIOExt,
    SM: StateMachineIndex,
{
    pub fn new(
        channel: CH,
        clock_divider: u16,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        clock_pin: (u8, u8),
        pins: (u8, u8),
        buffer: &'static mut [u16],
    ) -> ()
    where
        P: PIOExt,
        SM: StateMachineIndex,
    {
        let audio_program = pio_proc::pio_file!("src/audio_i2s.pio");

        let video_program_installed = pio.install(&audio_program.program).unwrap();
        let program_offset = video_program_installed.offset();

        let (mut video_sm, rx, mut vid_tx) =
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
        let mut sm = video_sm.start();
        let to_dest: Tx<(P, SM), rp2040_hal::dma::HalfWord> =
            vid_tx.transfer_size(hal::dma::HalfWord);

        let instruction = pio::Instruction {
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: program_offset + audio_program.public_defines.entry_point as u8,
            },
            delay: 0,
            side_set: None,
        };
        sm.exec_instruction(instruction);
        Self {
            dma_state: Some(DmaState::IDLE(
                channel,
                LimitingArrayReadTarget::new(buffer, buffer.len() as u32),
                to_dest,
            )),
        };
    }

    // #[allow(dead_code)]
    // pub fn free(self, pio: &mut PIO<P>) -> (UninitStateMachine<(P, SM)>, RS) {
    //     let (sm, prg) = self.sm.uninit(self.rx, self.tx);
    //     pio.uninstall(prg);
    //     (sm, self.rs)
    // }
}

impl<CH, P, SM> gb_core::hardware::sound::AudioPlayer for I2sPioInterface<CH, P, SM>
where
    CH: SingleChannel,
    P: PIOExt,
    SM: StateMachineIndex,
{
    fn play(&mut self, output_buffer: &[f32]) {
        let dma_state = core::mem::replace(&mut self.dma_state, None).unwrap();
        let (ch, audio_buffer, tx) = match dma_state {
            DmaState::IDLE(ch, buff, tx) => (ch, buff, tx),
            DmaState::RUNNING(dma) => dma.wait(),
        };
        let output = audio_buffer.new_max_read(output_buffer.len() as u32);
        for (i, v) in output_buffer.iter().enumerate() {
            let clamped = (*v).max(-1.0).min(1.0);
            let clamp = (clamped * i16::MAX as f32) as i16;
            output.array[i] = clamp as u16;
        }

        let sbc = Config::new(ch, output, tx).start();

        self.dma_state = Some(DmaState::RUNNING(sbc));
    }

    fn samples_rate(&self) -> u32 {
        5512
    }

    fn underflowed(&self) -> bool {
        match &self.dma_state {
            Some(dma_state) => match dma_state {
                DmaState::IDLE(..) => true,
                DmaState::RUNNING(transfer) => transfer.is_done(),
            },
            None => false,
        }
    }
}

struct LimitingArrayReadTarget {
    array: &'static mut [u16],
    max_read: u32,
}

impl LimitingArrayReadTarget {
    fn new(array: &'static mut [u16], max_read: u32) -> Self {
        Self { array, max_read }
    }

    fn new_max_read(self, max_read: u32) -> Self {
        Self {
            array: self.array,
            max_read,
        }
    }
}

unsafe impl ReadTarget for LimitingArrayReadTarget {
    type ReceivedWord = u16;

    fn rx_treq() -> Option<u8> {
        None
    }

    fn rx_address_count(&self) -> (u32, u32) {
        let (ptr, _) = unsafe { self.array.read_buffer() };
        (ptr as u32, self.max_read as u32)
    }

    fn rx_increment(&self) -> bool {
        self.array.rx_increment()
    }
}
