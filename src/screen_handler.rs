use crate::array_scaler::LineTransfer;
use byte_slice_cast::*;
pub struct SpiScreenHandler<'a, T: LineTransfer<Item = u8>, I: Iterator<Item = u16>> {
    iterator: &'a mut I,
    scaled_scan_line_buffer: &'static mut [u8],
    line_transfer: T,
}

impl<'a, I, T> SpiScreenHandler<'a, T, I>
where
    I: Iterator<Item = u16>,
    T: LineTransfer<Item = u8>,
{
    pub fn new(iterator: &'a mut I, line_transfer: T, buffer: &'static mut [u8]) -> Self {
        Self {
            iterator: iterator,
            scaled_scan_line_buffer: buffer,
            line_transfer: line_transfer,
        }
    }
}

impl<'a, I, T> SpiScreenHandler<'a, T, I>
where
    I: Iterator<Item = u16>,
    T: LineTransfer<Item = u8>,
{
    pub fn compute_line(self) -> (T, &'static mut [u8]) {
        let mut transfer = self.line_transfer;

        let mut buffer = self.scaled_scan_line_buffer;
        // AsByteSlice::as_byte_slice(buffer);

        let mut width_position = 0;
        for pixel in self.iterator {
            let pixel_parts: [u8; 2] = pixel.to_be_bytes();
            buffer[width_position] = pixel_parts[0];
            buffer[width_position + 1] = pixel_parts[1];
            width_position += 2;
            if width_position == buffer.len() {
                buffer = transfer.send_scanline(buffer);
                width_position = 0;
            }
        }

        (transfer, buffer)
    }
}
