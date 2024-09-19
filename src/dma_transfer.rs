use crate::array_scaler::LineTransfer;
use crate::rp_hal::hal;

use hal::dma::{
    single_buffer::{Config, Transfer},
    ReadTarget, SingleChannel, WriteTarget,
};

use embedded_dma::Word;

enum DmaState<
    T: Word,
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = T>,
    TO: WriteTarget<TransmittedWord = T>,
> {
    IDLE(CH, FROM, TO),
    RUNNING(Transfer<CH, FROM, TO>),
}

pub struct DmaTransfer<T: 'static + Word, CH: SingleChannel, TO: WriteTarget<TransmittedWord = T>> {
    dma: Option<DmaState<T, CH, &'static mut [T], TO>>,
}

impl<T, CH, TO> DmaTransfer<T, CH, TO>
where
    T: 'static + Word,
    CH: SingleChannel,
    TO: WriteTarget<TransmittedWord = T>,
{
    pub fn new(dma_channel: CH, tx: TO, buffer: &'static mut [T]) -> Self {
        Self {
            dma: (Some(DmaState::IDLE(dma_channel, buffer, tx))),
        }
    }

    pub fn do_tranfer(&mut self, buffer: &'static mut [T]) -> &'static mut [T] {
        let dma_state = core::mem::replace(&mut self.dma, None).unwrap();

        let (ch, old_buffer, tx) = match dma_state {
            DmaState::IDLE(ch, buff, tx) => (ch, buff, tx),
            DmaState::RUNNING(dma) => dma.wait(),
        };

        let sbc = Config::new(ch, buffer, tx).start();
        self.dma = Some(DmaState::RUNNING(sbc));

        old_buffer
    }

    pub fn free(mut self) -> (CH, TO, &'static mut [T]) {
        let foo = core::mem::replace(&mut self.dma, None).unwrap();
        let (ch, old_buffer, tx) = match foo {
            DmaState::IDLE(ch, buff, tx) => (ch, buff, tx),
            DmaState::RUNNING(dma) => dma.wait(),
        };
        (ch, tx, old_buffer)
    }
}

impl<T, CH, TO> LineTransfer for DmaTransfer<T, CH, TO>
where
    T: Word,
    CH: SingleChannel,
    TO: WriteTarget<TransmittedWord = T>,
{
    type Item = T;
    fn send_scanline(&mut self, line: &'static mut [Self::Item]) -> &'static mut [Self::Item] {
        self.do_tranfer(line)
        //line
    }
}
