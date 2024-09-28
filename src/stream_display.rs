use crate::array_scaler::ScreenHandler;
use crate::hal::dma::WriteTarget;

use crate::dma_transfer;
use crate::hal::dma::EndlessWriteTarget;
use crate::rp_hal::hal;
use embedded_dma::Word;
use hal::dma::SingleChannel;

pub struct Streamer<CH1, CH2, DO: 'static> {
    dma_channel1: Option<CH1>,
    dma_channel2: Option<CH2>,
    spare_buffer: Option<&'static mut [DO]>,
    spare_buffer2: Option<&'static mut [DO]>,
    main_buffer: Option<&'static mut [DO]>,
}

impl<CH1, CH2, DO: 'static> Streamer<CH1, CH2, DO>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    DO: Word,
{
    pub fn new(
        channel1: CH1,
        channel2: CH2,
        spare_buffer: &'static mut [DO],
        main_buffer: &'static mut [DO],
        spare_buffer2: &'static mut [DO],
    ) -> Self {
        Self {
            dma_channel1: Some(channel1),
            dma_channel2: Some(channel2),
            spare_buffer: Some(spare_buffer),
            spare_buffer2: Some(spare_buffer2),
            main_buffer: Some(main_buffer),
        }
    }

    pub fn stream<I, TO, F, DI, const TS: usize>(
        &mut self,
        tx: TO,
        iterator: &mut I,
        transformer: F,
    ) -> TO
    where
        DO: Word + Copy,
        TO: WriteTarget<TransmittedWord = DO> + EndlessWriteTarget,
        I: Iterator<Item = DI>,
        F: Fn(DI) -> [DO; TS],
    {
        let channel1 = core::mem::replace(&mut self.dma_channel1, None).unwrap();
        let channel2 = core::mem::replace(&mut self.dma_channel2, None).unwrap();
        let spare_buffer = core::mem::replace(&mut self.spare_buffer, None).unwrap();
        let spare_buffer2 = core::mem::replace(&mut self.spare_buffer2, None).unwrap();
        let main_buffer = core::mem::replace(&mut self.main_buffer, None).unwrap();
        let stream =
            dma_transfer::DmaTransfer::new(channel1, channel2, tx, main_buffer, spare_buffer2);

        let sh = ScreenHandler::new(iterator, stream, spare_buffer, transformer);
        let (stream, spare_buffer) = sh.compute_line();

        let (channel1, channel2, sm, main_buffer, spare_buffer2) = stream.free();

        self.main_buffer = Some(main_buffer);
        self.spare_buffer = Some(spare_buffer);
        self.spare_buffer2 = Some(spare_buffer2);
        self.dma_channel1 = Some(channel1);
        self.dma_channel2 = Some(channel2);

        sm
    }
}
