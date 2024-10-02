use core::cell::RefCell;

use crate::rp_hal::hal;
use crate::stream_display::Streamer;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_dma::Word;
use embedded_hal::digital::OutputPin;
use hal::dma::HalfWord;
use hal::pio::{PIOExt, PIO};
use hal::pio::{Running, StateMachine, StateMachineIndex, Tx};
use hal::pio::{Rx, UninitStateMachine};
type Result = core::result::Result<(), DisplayError>;
use hal::dma::Byte;
use rp2040_hal::dma::SingleChannel;
use rp2040_hal::pio::Stopped;
pub struct SpiPioInterfaceMultiBitDma<
    RS,
    P: PIOExt,
    SM1: StateMachineIndex,
    SM2: StateMachineIndex,
    CH1,
    CH2,
> {
    streamer: Option<Streamer<CH1, CH2, u16>>,
    mode: Option<PioMode<P, SM1, SM2>>,
    rs: RS,
}

enum PioMode<P: PIOExt, SM1: StateMachineIndex, SM2: StateMachineIndex> {
    ByteMode(
        (
            PioContainer<P, SM1, Byte, Running>,
            PioContainer<P, SM2, HalfWord, Stopped>,
        ),
    ),
    HalfWordMode(
        (
            PioContainer<P, SM1, Byte, Stopped>,
            PioContainer<P, SM2, HalfWord, Running>,
        ),
    ),
}

struct PioContainer<P: PIOExt, SM: StateMachineIndex, TxSize, State> {
    sm: StateMachine<(P, SM), State>,
    tx: Tx<(P, SM), TxSize>,
    rx: Rx<(P, SM)>,
}
impl<RS, P, SM1, SM2, CH1, CH2> SpiPioInterfaceMultiBitDma<RS, P, SM1, SM2, CH1, CH2>
where
    P: PIOExt,
    SM1: StateMachineIndex,
    SM2: StateMachineIndex,
    RS: OutputPin,
    CH1: SingleChannel,
    CH2: SingleChannel,
{
    pub fn new(
        clock_divider: u16,
        rs: RS,
        pio: &mut PIO<P>,
        sm1: UninitStateMachine<(P, SM1)>,
        sm2: UninitStateMachine<(P, SM2)>,
        clk: u8,
        tx: u8,
        streamer: Streamer<CH1, CH2, u16>,
    ) -> Self {
        let video_program =
            pio_proc::pio_asm!(".side_set 1 ", "out pins, 1 side 0 [1]", "nop side 1",);

        let video_program_installed = pio.install(&video_program.program).unwrap();
        let (mut sm_8b, rx_8b, tx_8b) =
            hal::pio::PIOBuilder::from_installed_program(video_program_installed)
                .out_pins(tx, 1)
                .side_set_pin_base(clk)
                .autopull(true)
                .pull_threshold(8)
                .out_shift_direction(hal::pio::ShiftDirection::Left)
                .in_shift_direction(hal::pio::ShiftDirection::Left)
                .buffers(hal::pio::Buffers::OnlyTx)
                .clock_divisor_fixed_point(clock_divider, 0)
                .build(sm1);
        sm_8b.set_pindirs([(tx, hal::pio::PinDir::Output)]);
        sm_8b.set_pindirs([(clk, hal::pio::PinDir::Output)]);
        let video_program_installed = pio.install(&video_program.program).unwrap();
        let (mut sm_16b, rx_16b, tx_16b) =
            hal::pio::PIOBuilder::from_installed_program(video_program_installed)
                .out_pins(tx, 1)
                .side_set_pin_base(clk)
                .autopull(true)
                .pull_threshold(16)
                .out_shift_direction(hal::pio::ShiftDirection::Left)
                .in_shift_direction(hal::pio::ShiftDirection::Left)
                .buffers(hal::pio::Buffers::OnlyTx)
                .clock_divisor_fixed_point(clock_divider, 0)
                .build(sm2);

        sm_16b.set_pindirs([(tx, hal::pio::PinDir::Output)]);
        sm_16b.set_pindirs([(clk, hal::pio::PinDir::Output)]);

        let byte_sm = PioContainer {
            sm: sm_8b.start(),
            tx: tx_8b.transfer_size(Byte),
            rx: rx_8b,
        };

        let half_word_sm = PioContainer {
            sm: sm_16b,
            tx: tx_16b.transfer_size(HalfWord),
            rx: rx_16b,
        };
        Self {
            streamer: Some(streamer),
            rs,
            mode: Some(PioMode::ByteMode((byte_sm, half_word_sm))),
        }
    }

    pub fn transfer_16bit_mode<F>(&mut self, mut callback: F)
    where
        F: FnMut(Tx<(P, SM2), HalfWord>, &mut Streamer<CH1, CH2, u16>) -> Tx<(P, SM2), HalfWord>,
    {
        let pio_mode = core::mem::replace(&mut self.mode, None).unwrap();
        let (byte_sm, mut half_byte_sm) = Self::set_16bit_mode(pio_mode);

        let mut streamer = self.streamer.take().unwrap();
        let interface = (callback)(half_byte_sm.tx, &mut streamer);
        half_byte_sm.tx = interface;
        self.streamer = Some(streamer);
        self.mode = Some(PioMode::HalfWordMode((byte_sm, half_byte_sm)));
    }

    pub fn transfer_16bit_mode_two<F>(&mut self, mut callback: F)
    where
        F: FnMut(
            Tx<(P, SM2), HalfWord>,
            Streamer<CH1, CH2, u16>,
        ) -> (Tx<(P, SM2), HalfWord>, Streamer<CH1, CH2, u16>),
    {
        let pio_mode = core::mem::replace(&mut self.mode, None).unwrap();
        let (byte_sm, mut half_byte_sm) = Self::set_16bit_mode(pio_mode);
        let mut streamer = self.streamer.take().unwrap();
        let interface = (callback)(half_byte_sm.tx, streamer);
        half_byte_sm.tx = interface.0;
        self.streamer = Some(interface.1);
        self.mode = Some(PioMode::HalfWordMode((byte_sm, half_byte_sm)));
    }

    pub fn iterator_16bit_mode(&mut self, iterator: &mut dyn Iterator<Item = u16>) {
        let pio_mode = core::mem::replace(&mut self.mode, None).unwrap();
        let (byte_sm, mut half_byte_sm) = Self::set_16bit_mode(pio_mode);

        let mut streamer = self.streamer.take().unwrap();
        half_byte_sm.tx = streamer.stream(half_byte_sm.tx, iterator);
        self.streamer = Some(streamer);
        self.mode = Some(PioMode::HalfWordMode((byte_sm, half_byte_sm)));
    }

    pub fn transfer_8bit_mode<F>(&mut self, mut callback: F)
    where
        F: FnMut(Tx<(P, SM1), Byte>) -> Tx<(P, SM1), Byte>,
    {
        let pio_mode = core::mem::replace(&mut self.mode, None).unwrap();
        let (mut byte_sm, half_byte_sm) = Self::set_8bit_mode(pio_mode);

        let interface = (callback)(byte_sm.tx);
        byte_sm.tx = interface;

        self.mode = Some(PioMode::ByteMode((byte_sm, half_byte_sm)));
    }

    fn set_8bit_mode(
        pio_mode: PioMode<P, SM1, SM2>,
    ) -> (
        PioContainer<P, SM1, Byte, Running>,
        PioContainer<P, SM2, HalfWord, Stopped>,
    ) {
        let new_mode = match pio_mode {
            PioMode::ByteMode(mode) => (mode.0, mode.1),
            PioMode::HalfWordMode(mode) => {
                let half_word_sm = PioContainer {
                    sm: mode.1.sm.stop(),
                    tx: mode.1.tx,
                    rx: mode.1.rx,
                };
                let byte_sm = PioContainer {
                    sm: mode.0.sm.start(),
                    tx: mode.0.tx,
                    rx: mode.0.rx,
                };
                (byte_sm, half_word_sm)
            }
        };
        new_mode
    }

    fn set_16bit_mode(
        pio_mode: PioMode<P, SM1, SM2>,
    ) -> (
        PioContainer<P, SM1, Byte, Stopped>,
        PioContainer<P, SM2, HalfWord, Running>,
    ) {
        let new_mode = match pio_mode {
            PioMode::ByteMode(mode) => {
                let byte_sm = PioContainer {
                    sm: mode.0.sm.stop(),
                    tx: mode.0.tx,
                    rx: mode.0.rx,
                };
                let half_word_sm = PioContainer {
                    sm: mode.1.sm.start(),
                    tx: mode.1.tx,
                    rx: mode.1.rx,
                };
                (byte_sm, half_word_sm)
            }
            PioMode::HalfWordMode(mode) => (mode.0, mode.1),
        };
        new_mode
    }

    // pub fn transfer_16bit_mode<F>(mut self, mut callback: F) -> Self
    // where
    //     F: FnMut(Tx<(P, SM), HalfWord>) -> Tx<(P, SM), HalfWord>,
    // {
    //     let interface = (callback)(self.tx);
    //     self.tx = interface;
    //     self
    // }

    // pub fn transfer_8bit_mode<F>(mut self, mut callback: F) -> Self
    // where
    //     F: FnMut(Tx<(P, SM), Byte>) -> Tx<(P, SM), Byte>,
    // {
    //     let interface = (callback)(self.tx.transfer_size(Byte));
    //     self.tx = interface.transfer_size(HalfWord);
    //     self
    // }

    // pub fn free(self, pio: &mut PIO<P>) -> (UninitStateMachine<(P, SM)>, RS) {
    //     let (sm, prg) = self.sm.uninit(self.rx, self.tx);
    //     pio.uninstall(prg);
    //     (sm, self.rs)
    // }
}

impl<RS, P, SM1, SM2, CH1: SingleChannel, CH2: SingleChannel> WriteOnlyDataCommand
    for SpiPioInterfaceMultiBitDma<RS, P, SM1, SM2, CH1, CH2>
where
    P: PIOExt,
    SM1: StateMachineIndex,
    SM2: StateMachineIndex,
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

fn send_data<RS, P, SM1, SM2, CH1: SingleChannel, CH2: SingleChannel>(
    iface: &mut SpiPioInterfaceMultiBitDma<RS, P, SM1, SM2, CH1, CH2>,
    words: DataFormat<'_>,
) -> Result
where
    P: PIOExt,
    SM1: StateMachineIndex,
    SM2: StateMachineIndex,
    RS: OutputPin,
{
    match words {
        DataFormat::U8(slice) => {
            iface.transfer_8bit_mode(|mut tx| {
                for i in slice {
                    while !tx.write_u8_replicated(*i) {}
                }
                while !tx.is_empty() {}
                return tx;
            });

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
            iface.transfer_16bit_mode(|tx, streamer| {
                let txx: Tx<(P, SM2), HalfWord> = streamer.stream(tx, iter);
                return txx;
            });

            Ok(())
        }
        _ => Err(DisplayError::DataFormatNotImplemented),
    }
}
