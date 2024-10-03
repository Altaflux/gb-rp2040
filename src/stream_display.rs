use crate::array_scaler::LineTransfer;
use crate::hal::dma::WriteTarget;

use crate::dma_transfer;
use crate::hal::dma::EndlessWriteTarget;
use crate::rp_hal::hal;
use byte_slice_cast::{AsMutSliceOf, ToMutByteSlice};
use hal::dma::SingleChannel;
pub struct Streamer<CH1, CH2> {
    dma_channel1: Option<CH1>,
    dma_channel2: Option<CH2>,
    spare_buffer: Option<&'static mut [u16]>,
    spare_buffer2: Option<&'static mut [u16]>,
    main_buffer: Option<&'static mut [u16]>,
}

impl<CH1, CH2> Streamer<CH1, CH2>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
{
    pub fn new(
        channel1: CH1,
        channel2: CH2,
        spare_buffer: &'static mut [u16],
        main_buffer: &'static mut [u16],
        spare_buffer2: &'static mut [u16],
    ) -> Self {
        Self {
            dma_channel1: Some(channel1),
            dma_channel2: Some(channel2),
            spare_buffer: Some(spare_buffer),
            spare_buffer2: Some(spare_buffer2),
            main_buffer: Some(main_buffer),
        }
    }

    #[inline(always)]
    pub fn stream_8b<TO>(&mut self, tx: TO, iterator: &mut dyn Iterator<Item = u8>) -> TO
    where
        TO: WriteTarget<TransmittedWord = u8> + EndlessWriteTarget,
    {
        let channel1 = core::mem::replace(&mut self.dma_channel1, None).unwrap();
        let channel2 = core::mem::replace(&mut self.dma_channel2, None).unwrap();

        let spare_buffer: &'static mut [u8] =
            ToMutByteSlice::to_mut_byte_slice(self.spare_buffer.take().unwrap());
        let spare_buffer2: &'static mut [u8] =
            ToMutByteSlice::to_mut_byte_slice(self.spare_buffer2.take().unwrap());
        let main_buffer: &'static mut [u8] =
            ToMutByteSlice::to_mut_byte_slice(self.main_buffer.take().unwrap());
        let stream =
            dma_transfer::DmaTransfer::new(channel1, channel2, tx, main_buffer, spare_buffer2);

        let (stream, spare_buffer) = Self::compute_line(stream, spare_buffer, iterator);

        let (channel1, channel2, sm, main_buffer, spare_buffer2) = stream.free();
        self.main_buffer = Some(AsMutSliceOf::as_mut_slice_of::<u16>(main_buffer).unwrap());
        self.spare_buffer = Some(AsMutSliceOf::as_mut_slice_of::<u16>(spare_buffer).unwrap());
        self.spare_buffer2 = Some(AsMutSliceOf::as_mut_slice_of::<u16>(spare_buffer2).unwrap());
        self.dma_channel1 = Some(channel1);
        self.dma_channel2 = Some(channel2);

        sm
    }
    #[inline(always)]
    pub fn stream_16b<TO, F>(&mut self, tx: TO, iterator: &mut dyn Iterator<Item = u16>, f: F) -> TO
    where
        TO: WriteTarget<TransmittedWord = u16> + EndlessWriteTarget,
        F: Fn(u16) -> u16,
    {
        let channel1 = core::mem::replace(&mut self.dma_channel1, None).unwrap();
        let channel2 = core::mem::replace(&mut self.dma_channel2, None).unwrap();
        let spare_buffer = core::mem::replace(&mut self.spare_buffer, None).unwrap();

        let spare_buffer2 = core::mem::replace(&mut self.spare_buffer2, None).unwrap();

        let main_buffer = core::mem::replace(&mut self.main_buffer, None).unwrap();

        let stream =
            dma_transfer::DmaTransfer::new(channel1, channel2, tx, main_buffer, spare_buffer2);

        let (stream, spare_buffer) = Self::compute_line_u16(stream, spare_buffer, iterator, f);

        let (channel1, channel2, sm, main_buffer, spare_buffer2) = stream.free();

        self.main_buffer = Some(main_buffer);
        self.spare_buffer = Some(spare_buffer);
        self.spare_buffer2 = Some(spare_buffer2);
        self.dma_channel1 = Some(channel1);
        self.dma_channel2 = Some(channel2);

        sm
    }
    #[inline(always)]
    pub fn compute_line<DO, T: LineTransfer<Item = DO>>(
        mut transfer: T,
        mut buffer: &'static mut [DO],
        iterator: &mut dyn Iterator<Item = DO>,
    ) -> (T, &'static mut [DO]) {
        let mut width_position = 0;
        for pixel in iterator {
            let out = pixel;
            buffer[width_position] = out;
            width_position += 1;
            if width_position == buffer.len() {
                buffer = transfer.send_scanline(buffer, buffer.len() as u32);
                width_position = 0;
            }
        }

        if width_position > 0 {
            buffer = transfer.send_scanline(buffer, width_position as u32);
        }

        (transfer, buffer)
    }

    #[inline(always)]
    pub fn compute_line_u16<T: LineTransfer<Item = u16>, F>(
        mut transfer: T,
        mut buffer: &'static mut [u16],
        iterator: &mut dyn Iterator<Item = u16>,
        f: F,
    ) -> (T, &'static mut [u16])
    where
        F: Fn(u16) -> u16,
    {
        let mut width_position = 0;
        for pixel in iterator.map(f) {
            let out = pixel;
            buffer[width_position] = out;
            width_position += 1;
            if width_position == buffer.len() {
                buffer = transfer.send_scanline(buffer, buffer.len() as u32);
                width_position = 0;
            }
        }

        if width_position > 0 {
            buffer = transfer.send_scanline(buffer, width_position as u32);
        }

        (transfer, buffer)
    }
}
