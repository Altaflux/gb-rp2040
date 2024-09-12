use crate::hal::dma::WriteTarget;

use crate::rp_hal::hal;
use hal::dma::SingleChannel;

use crate::array_scaler::ScreenHandler;
use crate::dma_transfer;
use embedded_dma::Word;
pub struct Streamer<T, CH>
where
    T: 'static,
{
    dma_channel: Option<CH>,
    spare_buffer: Option<&'static mut [T]>,
    main_buffer: Option<&'static mut [T]>,
}

impl<T, CH> Streamer<T, CH>
where
    T: Word,
    CH: SingleChannel,
{
    pub fn new(channel: CH, spare_buffer: &'static mut [T], main_buffer: &'static mut [T]) -> Self {
        Self {
            dma_channel: Some(channel),
            spare_buffer: Some(spare_buffer),
            main_buffer: Some(main_buffer),
        }
    }

    pub fn stream<I, TO>(&mut self, tx: TO, iterator: &mut I) -> TO
    where
        TO: WriteTarget<TransmittedWord = T>,
        I: Iterator<Item = T>,
    {
        let channel = core::mem::replace(&mut self.dma_channel, None).unwrap();
        let spare_buffer = core::mem::replace(&mut self.spare_buffer, None).unwrap();
        let main_buffer = core::mem::replace(&mut self.main_buffer, None).unwrap();
        let stream = dma_transfer::DmaTransfer::new(channel, tx, main_buffer);

        let sh: ScreenHandler<T, _, _> = ScreenHandler::new(iterator, stream, spare_buffer);
        let (stream, spare_buffer) = sh.compute_line();

        let (channel, sm, main_buffer) = stream.free();

        self.main_buffer = Some(main_buffer);
        self.spare_buffer = Some(spare_buffer);
        self.dma_channel = Some(channel);

        sm
    }
}
