use crate::rp_hal::hal;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_hal::digital::OutputPin;
use hal::dma::HalfWord;
use hal::pio::{PIOExt, PIO};
use hal::pio::{Running, StateMachine, StateMachineIndex, Tx};
use hal::pio::{Rx, UninitStateMachine};
type Result = core::result::Result<(), DisplayError>;
use hal::dma::Byte;
pub struct SpiPioInterface<RS, P: PIOExt, SM: StateMachineIndex> {
    sm: StateMachine<(P, SM), Running>,
    tx: Tx<(P, SM), HalfWord>,
    rx: Rx<(P, SM)>,
    rs: RS,
}

impl<RS, P, SM> SpiPioInterface<RS, P, SM>
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
        clk: u8,
        tx: u8,
    ) -> Self {
        let video_program =
            pio_proc::pio_asm!(".side_set 1 ", "out pins, 1 side 0 [1]", "nop side 1",);

        let video_program_installed = pio.install(&video_program.program).unwrap();
        let (mut video_sm, rx, vid_tx) =
            hal::pio::PIOBuilder::from_installed_program(video_program_installed)
                .out_pins(tx, 1)
                .side_set_pin_base(clk)
                .autopull(true)
                .pull_threshold(8)
                .out_shift_direction(hal::pio::ShiftDirection::Left)
                .in_shift_direction(hal::pio::ShiftDirection::Left)
                .buffers(hal::pio::Buffers::OnlyTx)
                .clock_divisor_fixed_point(clock_divider, 0)
                .build(sm);
        video_sm.set_pindirs([(tx, hal::pio::PinDir::Output)]);
        video_sm.set_pindirs([(clk, hal::pio::PinDir::Output)]);

        Self {
            rs: rs,
            sm: video_sm.start(),
            rx: rx,
            tx: vid_tx.transfer_size(HalfWord),
        }
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

    pub fn free(self, pio: &mut PIO<P>) -> (UninitStateMachine<(P, SM)>, RS) {
        let (sm, prg) = self.sm.uninit(self.rx, self.tx);
        pio.uninstall(prg);
        (sm, self.rs)
    }
}

impl<RS, P, SM> WriteOnlyDataCommand for SpiPioInterface<RS, P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
    RS: OutputPin,
{
    fn send_commands(&mut self, cmd: display_interface::DataFormat<'_>) -> Result {
        self.rs.set_low().map_err(|_| DisplayError::RSError)?;
        send_data(self, cmd)?;
        Ok(())
    }

    fn send_data(&mut self, buf: display_interface::DataFormat<'_>) -> Result {
        self.rs.set_high().map_err(|_| DisplayError::RSError)?;
        send_data(self, buf)?;
        Ok(())
    }
}

fn send_data<RS, P, SM>(iface: &mut SpiPioInterface<RS, P, SM>, words: DataFormat<'_>) -> Result
where
    P: PIOExt,
    SM: StateMachineIndex,
    RS: OutputPin,
{
    match words {
        DataFormat::U8(slice) => {
            for i in slice {
                while !iface.tx.write_u8_replicated(*i) {}
            }
            while !iface.tx.is_empty() {}
            Ok(())
        }
        // DataFormat::U16(slice) => {
        //     for i in slice {
        //         while !iface.tx.write_u16_replicated(*i) {}
        //     }
        //     while !iface.tx.is_empty() {}
        //     Ok(())
        // }
        // DataFormat::U16LE(slice) => {
        //     for i in slice {
        //         let tmp = (iface.endian_function)(false, *i);
        //         while !iface.tx.write_u16_replicated(tmp) {}
        //     }
        //     while !iface.tx.is_empty() {}
        //     Ok(())
        // }
        // DataFormat::U16BE(slice) => {
        //     for i in slice {
        //         let tmp = (iface.endian_function)(true, *i);
        //         while !iface.tx.write_u16_replicated(tmp) {}
        //     }
        //     while !iface.tx.is_empty() {}
        //     Ok(())
        // }
        // DataFormat::U8Iter(iter) => {
        //     for i in iter {
        //         while !iface.tx.write_u8_replicated(i) {}
        //     }
        //     while !iface.tx.is_empty() {}
        //     Ok(())
        // }
        // DataFormat::U16LEIter(iter) => {
        //     for i in iter {
        //         let tmp = (iface.endian_function)(false, i);
        //         while !iface.tx.write_u16_replicated(tmp) {}
        //     }
        //     while !iface.tx.is_empty() {}
        //     Ok(())
        // }
        DataFormat::U16BEIter(iter) => {
            for i in iter {
                let i_split = i.to_be_bytes();
                while !iface.tx.write_u8_replicated(i_split[0]) {}
                while !iface.tx.write_u8_replicated(i_split[0]) {}
            }
            while !iface.tx.is_empty() {}
            Ok(())
        }
        _ => Err(DisplayError::DataFormatNotImplemented),
    }
}
