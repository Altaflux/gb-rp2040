use crate::array_scaler::LineTransfer;

pub struct SpiScreenHandler<
    'a,
    DI,
    DO,
    T: LineTransfer<Item = DO>,
    I: Iterator<Item = DI>,
    F,
    const TS: usize,
> where
    DO: 'static,
{
    iterator: &'a mut I,
    scaled_scan_line_buffer: &'static mut [DO],
    line_transfer: T,
    type_converter: F,
}

impl<'a, DI, DO, I, T, F, const TS: usize> SpiScreenHandler<'a, DI, DO, T, I, F, TS>
where
    I: Iterator<Item = DI>,
    T: LineTransfer<Item = DO>,
    DO: 'static + Copy,
    F: Fn(DI) -> [DO; TS],
{
    pub fn new(
        iterator: &'a mut I,
        line_transfer: T,
        buffer: &'static mut [DO],
        converter: F,
    ) -> Self {
        Self {
            iterator: iterator,
            scaled_scan_line_buffer: buffer,
            line_transfer: line_transfer,
            type_converter: converter,
        }
    }
}

impl<'a, DI, DO, I, T, F, const TS: usize> SpiScreenHandler<'a, DI, DO, T, I, F, TS>
where
    I: Iterator<Item = DI>,
    T: LineTransfer<Item = DO>,
    DO: 'static + Copy,
    F: Fn(DI) -> [DO; TS],
{
    pub fn compute_line(self) -> (T, &'static mut [DO]) {
        let mut transfer = self.line_transfer;

        let mut buffer = self.scaled_scan_line_buffer;

        let mut width_position = 0;
        for pixel in self.iterator {
            let out = (self.type_converter)(pixel);
            for i in 0..TS {
                buffer[width_position + i] = out[i];
            }
            width_position += TS;
            if width_position == buffer.len() {
                buffer = transfer.send_scanline(buffer);
                width_position = 0;
            }
        }

        (transfer, buffer)
    }
}
