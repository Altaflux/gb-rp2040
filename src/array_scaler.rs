//New Scaler
use byte_slice_cast::AsByteSlice;

pub struct ScreenHandler<'a, DT, T: LineTransfer<Item = DT>, I: Iterator<Item = DT>>
where
    DT: 'static,
{
    iterator: &'a mut I,
    scaled_scan_line_buffer: &'static mut [DT],
    line_transfer: T,
}

impl<'a, DT, I, T> ScreenHandler<'a, DT, T, I>
where
    I: Iterator<Item = DT>,
    T: LineTransfer<Item = DT>,
{
    pub fn new(iterator: &'a mut I, line_transfer: T, buffer: &'static mut [DT]) -> Self {
        Self {
            iterator: iterator,
            scaled_scan_line_buffer: buffer,
            line_transfer: line_transfer,
        }
    }
}

impl<'a, DT, I, T> ScreenHandler<'a, DT, T, I>
where
    I: Iterator<Item = DT>,
    T: LineTransfer<Item = DT>,
{
    pub fn compute_line(self) -> (T, &'static mut [DT]) {
        let mut transfer = self.line_transfer;

        let mut buffer = self.scaled_scan_line_buffer;
        // AsByteSlice::as_byte_slice(buffer);

        let mut width_position = 0;
        for pixel in self.iterator {
            buffer[width_position] = pixel;
            width_position += 1;
            if width_position == buffer.len() {
                buffer = transfer.send_scanline(buffer);
                width_position = 0;
            }
        }

        (transfer, buffer)
    }
}

pub trait LineTransfer {
    type Item;
    fn send_scanline(&mut self, line: &'static mut [Self::Item]) -> &'static mut [Self::Item];
}
