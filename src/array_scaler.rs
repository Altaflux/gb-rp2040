//New Scaler

pub struct ScreenHandler<'a, const OUT_WIDTH: usize, T: LineTransfer, I: Iterator<Item = u16>> {
    iterator: &'a mut I,
    scaled_scan_line_buffer: &'static mut [u16],
    line_transfer: T,
}

impl<'a, const OUT_WIDTH: usize, I, T> ScreenHandler<'a, OUT_WIDTH, T, I>
where
    I: Iterator<Item = u16>,
    T: LineTransfer,
{
    pub fn new(iterator: &'a mut I, line_transfer: T, buffer: &'static mut [u16]) -> Self {
        Self {
            iterator: iterator,
            scaled_scan_line_buffer: buffer,
            line_transfer: line_transfer,
        }
    }
}

impl<'a, I, T, const OUT_WIDTH: usize> ScreenHandler<'a, OUT_WIDTH, T, I>
where
    I: Iterator<Item = u16>,
    T: LineTransfer,
{
    pub fn compute_line(self) -> (T, &'static mut [u16]) {
        let mut transfer = self.line_transfer;
        let mut buffer = self.scaled_scan_line_buffer;

        let mut width_position = 0;
        for pixel in self.iterator {
            buffer[width_position] = pixel;
            width_position += 1;
            if width_position == OUT_WIDTH {
                buffer = transfer.send_scanline(buffer);
                width_position = 0;
            }
        }

        (transfer, buffer)
    }
}

pub trait LineTransfer {
    fn send_scanline(&mut self, line: &'static mut [u16]) -> &'static mut [u16];
}
