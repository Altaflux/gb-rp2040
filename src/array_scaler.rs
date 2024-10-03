//New Scaler

pub struct ScreenHandler<'a, DO, T: LineTransfer<Item = DO>, I: Iterator<Item = DO>>
where
    DO: 'static,
{
    iterator: &'a mut I,
    scaled_scan_line_buffer: &'static mut [DO],
    line_transfer: T,
}

impl<'a, DO, I, T> ScreenHandler<'a, DO, T, I>
where
    I: Iterator<Item = DO>,
    T: LineTransfer<Item = DO>,
    DO: 'static + Copy,
{
    pub fn new(iterator: &'a mut I, line_transfer: T, buffer: &'static mut [DO]) -> Self {
        Self {
            iterator: iterator,
            scaled_scan_line_buffer: buffer,
            line_transfer: line_transfer,
        }
    }
}

impl<'a, DO, I, T> ScreenHandler<'a, DO, T, I>
where
    I: Iterator<Item = DO>,
    T: LineTransfer<Item = DO>,
    DO: 'static + Copy,
{
    pub fn compute_line(self) -> (T, &'static mut [DO]) {
        let mut transfer = self.line_transfer;

        let mut buffer = self.scaled_scan_line_buffer;

        let mut width_position = 0;
        for pixel in self.iterator {
            let out = pixel;
            buffer[width_position] = out;
            width_position += 1;
            if width_position == buffer.len() {
                buffer = transfer.send_scanline(buffer, buffer.len() as u32);
                width_position = 0;
            }
        }

        (transfer, buffer)
    }
}

pub trait LineTransfer {
    type Item;
    fn send_scanline(
        &mut self,
        line: &'static mut [Self::Item],
        size: u32,
    ) -> &'static mut [Self::Item];
}
