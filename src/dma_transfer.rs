use crate::rp_hal::hal;

use hal::dma::{
    single_buffer::{Config, Transfer},
    ReadTarget, SingleChannel, WriteTarget,
};

use crate::array_scaler::LineTransfer;

enum DmaState<
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = u16>,
    TO: WriteTarget<TransmittedWord = u16>,
> {
    IDLE(CH, FROM, TO),
    RUNNING(Transfer<CH, FROM, TO>),
}

pub struct DmaTransfer<CH: SingleChannel, TO: WriteTarget<TransmittedWord = u16>> {
    dma: Option<DmaState<CH, &'static mut [u16], TO>>,
}

impl<CH, TO> DmaTransfer<CH, TO>
where
    CH: SingleChannel,
    TO: WriteTarget<TransmittedWord = u16>,
{
    pub fn new(dma_channel: CH, tx: TO, buffer: &'static mut [u16]) -> Self {
        Self {
            dma: (Some(DmaState::IDLE(dma_channel, buffer, tx))),
        }
    }

    pub fn do_tranfer(&mut self, buffer: &'static mut [u16]) -> &'static mut [u16] {
        let dma_state = core::mem::replace(&mut self.dma, None).unwrap();

        let (ch, old_buffer, tx) = match dma_state {
            DmaState::IDLE(ch, buff, tx) => (ch, buff, tx),
            DmaState::RUNNING(dma) => dma.wait(),
        };

        let sbc = Config::new(ch, buffer, tx).start();
        self.dma = Some(DmaState::RUNNING(sbc));

        old_buffer
    }

    pub fn free(mut self) -> (CH, TO, &'static mut [u16]) {
        let foo = core::mem::replace(&mut self.dma, None).unwrap();
        let (ch, old_buffer, tx) = match foo {
            DmaState::IDLE(ch, buff, tx) => (ch, buff, tx),
            DmaState::RUNNING(dma) => dma.wait(),
        };
        (ch, tx, old_buffer)
    }
}

impl<CH, TO> LineTransfer for DmaTransfer<CH, TO>
where
    CH: SingleChannel,
    TO: WriteTarget<TransmittedWord = u16>,
{
    fn send_scanline(&mut self, line: &'static mut [u16]) -> &'static mut [u16] {
        self.do_tranfer(line)
    }
}
