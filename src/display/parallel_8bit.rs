use crate::rp_hal::hal;
use crate::stream_display::Streamer;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_hal::digital::OutputPin;
use hal::dma::HalfWord;
use hal::pio::{PIOExt, PIO};
use hal::pio::{Running, StateMachine, StateMachineIndex, Tx};
use hal::pio::{Rx, UninitStateMachine};
type Result = core::result::Result<(), DisplayError>;
use hal::dma::Byte;
use rp2040_hal::dma::SingleChannel;
pub struct PioInterfaceStreamer<RS, P: PIOExt, SM: StateMachineIndex, END, CH1, CH2> {
    sm: StateMachine<(P, SM), Running>,
    tx: Option<Tx<(P, SM), HalfWord>>,
    rx: Rx<(P, SM)>,
    labels: PIOLabelDefines,
    rs: RS,
    streamer: Streamer<CH1, CH2>,
    pub endian_function: END,
}

impl<RS, P, SM, END, CH1, CH2> PioInterfaceStreamer<RS, P, SM, END, CH1, CH2>
where
    P: PIOExt,
    SM: StateMachineIndex,
    RS: OutputPin,
    END: Fn(bool, u16) -> u16,
{
    pub fn new(
        clock_divider: (u16, u8),
        rs: RS,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        rw: u8,
        pins: (u8, u8),
        streamer: Streamer<CH1, CH2>,
        endianess: END,
    ) -> Self {
        let video_program = pio_proc::pio_asm!(
            ".side_set 1 opt",
            "jmp start_8 side 1",
            ".wrap_target"

            "public start_tx:"
            "pull side 1",
            "out pins, 24 side 0  ",
            "nop side 1  ",
            "out pins, 8 side 0  ",
            "jmp start_tx side 1 ",

            "public start_8:"
            "pull side 1 ",
            "out pins, 32 side 0 ",
            "jmp start_8 side 1 ",
            ".wrap"
        );
        let out_pin_offset = ((1i16 - pins.0 as i16) + pins.1 as i16) as u8;
        let video_program_installed = pio.install(&video_program.program).unwrap();
        let program_offset = video_program_installed.offset();
        let (mut video_sm, rx, vid_tx) =
            hal::pio::PIOBuilder::from_installed_program(video_program_installed)
                .out_pins(pins.0, out_pin_offset)
                .side_set_pin_base(rw)
                .out_shift_direction(hal::pio::ShiftDirection::Left)
                .in_shift_direction(hal::pio::ShiftDirection::Left)
                .buffers(hal::pio::Buffers::OnlyTx)
                .clock_divisor_fixed_point(clock_divider.0, clock_divider.1)
                .build(sm);
        video_sm.set_pindirs((pins.0..pins.1 + 1 as u8).map(|n| (n, hal::pio::PinDir::Output)));
        video_sm.set_pindirs([(rw, hal::pio::PinDir::Output)]);

        let labels = PIOLabelDefines {
            program_offset: program_offset,
            bit_16: video_program.public_defines.start_tx,
            bit_8: video_program.public_defines.start_8,
        };

        Self {
            rs: rs,
            sm: video_sm.start(),
            rx: rx,
            tx: Some(vid_tx.transfer_size(HalfWord)),
            labels: labels,
            endian_function: endianess,
            streamer,
        }
    }

    fn set_8bit_mode(&mut self) {
        let instruction = pio::Instruction {
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: self.labels.program_offset as u8 + self.labels.bit_8 as u8,
            },
            delay: 0,
            side_set: None,
        };
        self.sm.exec_instruction(instruction);
    }
    fn set_16bit_mode(&mut self) {
        let instruction = pio::Instruction {
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: self.labels.program_offset as u8 + self.labels.bit_16 as u8,
            },
            delay: 0,
            side_set: None,
        };
        self.sm.exec_instruction(instruction);
    }

    pub fn transfer_16bit_mode<F>(&mut self, mut callback: F)
    where
        F: FnMut(Tx<(P, SM), HalfWord>) -> Tx<(P, SM), HalfWord>,
    {
        self.set_16bit_mode();
        let tx = core::mem::replace(&mut self.tx, None);
        let interface = (callback)(tx.unwrap());
        self.tx = Some(interface);
    }

    pub fn transfer_8bit_mode<F>(mut self, mut callback: F) -> Self
    where
        F: FnMut(Tx<(P, SM), Byte>) -> Tx<(P, SM), Byte>,
    {
        self.set_8bit_mode();
        let tx = core::mem::replace(&mut self.tx, None);
        let interface = (callback)(tx.unwrap().transfer_size(Byte));
        self.tx = Some(interface.transfer_size(HalfWord));
        self
    }

    #[allow(dead_code)]
    pub fn free(self, pio: &mut PIO<P>) -> (UninitStateMachine<(P, SM)>, RS) {
        let (sm, prg) = self.sm.uninit(self.rx, self.tx.unwrap());
        pio.uninstall(prg);
        (sm, self.rs)
    }
}

impl<RS, P, SM, END, CH1, CH2> WriteOnlyDataCommand
    for PioInterfaceStreamer<RS, P, SM, END, CH1, CH2>
where
    P: PIOExt,
    SM: StateMachineIndex,
    RS: OutputPin,
    CH1: SingleChannel,
    CH2: SingleChannel,
    END: Fn(bool, u16) -> u16,
{
    #[inline(always)]
    fn send_commands(&mut self, cmd: display_interface::DataFormat<'_>) -> Result {
        self.rs.set_low().map_err(|_| DisplayError::RSError)?;
        send_data(self, cmd)?;
        Ok(())
    }

    #[inline(always)]
    fn send_data(&mut self, buf: display_interface::DataFormat<'_>) -> Result {
        self.rs.set_high().map_err(|_| DisplayError::RSError)?;
        send_data(self, buf)?;
        Ok(())
    }
}
#[derive(Debug)]
struct PIOLabelDefines {
    pub program_offset: u8,
    pub bit_8: i32,
    pub bit_16: i32,
}

#[inline(always)]
fn send_data<RS, P, SM, END, CH1, CH2>(
    iface: &mut PioInterfaceStreamer<RS, P, SM, END, CH1, CH2>,
    words: DataFormat<'_>,
) -> Result
where
    P: PIOExt,
    SM: StateMachineIndex,
    RS: OutputPin,
    CH1: SingleChannel,
    CH2: SingleChannel,
    END: Fn(bool, u16) -> u16,
{
    match words {
        DataFormat::U8(slice) => {
            iface.set_8bit_mode();
            let tx = iface.tx.as_mut().unwrap();
            for i in slice {
                while !tx.write(*i as u32) {}
            }
            while !iface.tx.as_mut().unwrap().is_empty() {}
            Ok(())
        }
        DataFormat::U16(slice) => {
            iface.set_16bit_mode();
            let tx = iface.tx.as_mut().unwrap();
            for i in slice {
                let tmp = (*i) as u32;
                while !tx.write(tmp) {}
            }
            while !tx.is_empty() {}
            Ok(())
        }
        DataFormat::U16BE(slice) => {
            iface.set_16bit_mode();
            let tx = iface.tx.as_mut().unwrap();
            for i in slice {
                let tmp = (iface.endian_function)(true, *i) as u32;
                while !tx.write(tmp) {}
            }
            while !tx.is_empty() {}
            Ok(())
        }
        DataFormat::U16LE(slice) => {
            iface.set_16bit_mode();
            let tx = iface.tx.as_mut().unwrap();
            for i in slice {
                let tmp = (iface.endian_function)(false, *i) as u32;
                while !tx.write(tmp) {}
            }
            while !tx.is_empty() {}
            Ok(())
        }
        DataFormat::U8Iter(iter) => {
            iface.set_8bit_mode();
            let tx = iface.tx.take().unwrap();
            let tx = iface.streamer.stream_8b(tx.transfer_size(Byte), iter);
            iface.tx = Some(tx.transfer_size(HalfWord));
            Ok(())
        }
        DataFormat::U16BEIter(iter) => {
            iface.set_16bit_mode();
            let tx = iface.tx.take().unwrap();
            let tx = iface.streamer.stream_16b(tx, iter, u16::to_le);
            iface.tx = Some(tx);
            Ok(())
        }
        DataFormat::U16LEIter(iter) => {
            iface.set_16bit_mode();
            let tx = iface.tx.take().unwrap();
            let tx = iface.streamer.stream_16b(tx, iter, u16::to_be);
            iface.tx = Some(tx);
            Ok(())
        }
        _ => Err(DisplayError::DataFormatNotImplemented),
    }
}
