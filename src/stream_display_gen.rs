use crate::array_scaler::ScreenHandler;
use crate::hal::dma::WriteTarget;

use crate::dma_transfer;
use crate::rp_hal::hal;
use crate::screen_handler::SpiScreenHandler;
use embedded_dma::Word;
use hal::dma::SingleChannel;

pub struct StreamerSpi<CH, DO: 'static> {
    dma_channel: Option<CH>,
    spare_buffer: Option<&'static mut [DO]>,
    main_buffer: Option<&'static mut [DO]>,
}

impl<CH, DO: 'static> StreamerSpi<CH, DO>
where
    CH: SingleChannel,
    DO: Word,
{
    pub fn new(
        channel: CH,
        spare_buffer: &'static mut [DO],
        main_buffer: &'static mut [DO],
    ) -> Self {
        Self {
            dma_channel: Some(channel),
            spare_buffer: Some(spare_buffer),
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
        TO: WriteTarget<TransmittedWord = DO>,
        I: Iterator<Item = DI>,
        F: Fn(DI) -> [DO; TS],
    {
        let channel = core::mem::replace(&mut self.dma_channel, None).unwrap();
        let spare_buffer = core::mem::replace(&mut self.spare_buffer, None).unwrap();
        let main_buffer = core::mem::replace(&mut self.main_buffer, None).unwrap();
        let stream = dma_transfer::DmaTransfer::new(channel, tx, main_buffer);

        let sh = SpiScreenHandler::new(iterator, stream, spare_buffer, transformer);
        let (stream, spare_buffer) = sh.compute_line();

        let (channel, sm, main_buffer) = stream.free();

        self.main_buffer = Some(main_buffer);
        self.spare_buffer = Some(spare_buffer);
        self.dma_channel = Some(channel);

        sm
    }

    pub fn stream_no_conversion<I, TO>(&mut self, tx: TO, iterator: &mut I) -> TO
    where
        DO: Word + Copy,
        TO: WriteTarget<TransmittedWord = DO>,
        I: Iterator<Item = DO>,
    {
        let channel = core::mem::replace(&mut self.dma_channel, None).unwrap();
        let spare_buffer = core::mem::replace(&mut self.spare_buffer, None).unwrap();
        let main_buffer = core::mem::replace(&mut self.main_buffer, None).unwrap();
        let stream = dma_transfer::DmaTransfer::new(channel, tx, main_buffer);

        let sh = ScreenHandler::new(iterator, stream, spare_buffer);
        let (stream, spare_buffer) = sh.compute_line();

        let (channel, sm, main_buffer) = stream.free();

        self.main_buffer = Some(main_buffer);
        self.spare_buffer = Some(spare_buffer);
        self.dma_channel = Some(channel);

        sm
    }
}
