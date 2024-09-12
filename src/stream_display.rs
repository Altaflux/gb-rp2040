use crate::hal::dma::WriteTarget;

use crate::rp_hal::hal;
use hal::dma::SingleChannel;

use crate::array_scaler::ScreenHandler;
use crate::dma_transfer;

pub struct Streamer<CH> {
    dma_channel: Option<CH>,
    spare_buffer: Option<&'static mut [u16]>,
    main_buffer: Option<&'static mut [u16]>,
}

impl<CH> Streamer<CH>
where
    CH: SingleChannel,
{
    pub fn new(
        channel: CH,
        spare_buffer: &'static mut [u16],
        main_buffer: &'static mut [u16],
    ) -> Self {
        Self {
            dma_channel: Some(channel),
            spare_buffer: Some(spare_buffer),
            main_buffer: Some(main_buffer),
        }
    }

    pub fn stream<I, TO>(&mut self, tx: TO, iterator: &mut I) -> TO
    where
        TO: WriteTarget<TransmittedWord = u16>,
        I: Iterator<Item = u16>,
    {
        let channel = core::mem::replace(&mut self.dma_channel, None).unwrap();
        let spare_buffer = core::mem::replace(&mut self.spare_buffer, None).unwrap();
        let main_buffer = core::mem::replace(&mut self.main_buffer, None).unwrap();
        let stream = dma_transfer::DmaTransfer::new(channel, tx, main_buffer);

        let sh: ScreenHandler<_, _> = ScreenHandler::new(iterator, stream, spare_buffer);
        let (stream, spare_buffer) = sh.compute_line();

        let (channel, sm, main_buffer) = stream.free();

        self.main_buffer = Some(main_buffer);
        self.spare_buffer = Some(spare_buffer);
        self.dma_channel = Some(channel);

        sm
    }
}
