use core::u16;

use alloc::vec::Vec;

pub struct ScreenScaler<
    const IN_HEIGHT: usize,
    const IN_WIDTH: usize,
    const OUT_HEIGHT: usize,
    const OUT_WIDTH: usize,
> {
    width_ceil_calcs: [u16; IN_WIDTH],
    height_ceil_calcs: [u16; IN_HEIGHT],
}

impl<
        const IN_HEIGHT: usize,
        const IN_WIDTH: usize,
        const OUT_HEIGHT: usize,
        const OUT_WIDTH: usize,
    > ScreenScaler<IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH>
{
    pub fn new() -> Self {
        let calc_out_width_frac = OUT_WIDTH as f32 / IN_WIDTH as f32;
        let calc_out_height_frac = OUT_HEIGHT as f32 / IN_HEIGHT as f32;
        let mut width_ceil_calcs: [u16; IN_WIDTH] = [0u16; IN_WIDTH];
        let mut height_ceil_calcs: [u16; IN_HEIGHT] = [0u16; IN_HEIGHT];
        generate_scaling_ratio(calc_out_width_frac, IN_WIDTH, &mut width_ceil_calcs);
        generate_scaling_ratio(calc_out_height_frac, IN_HEIGHT, &mut height_ceil_calcs);
        Self {
            width_ceil_calcs: width_ceil_calcs,
            height_ceil_calcs: height_ceil_calcs,
        }
    }
    #[inline(always)]
    pub fn scale_iterator<'a, T, I>(&'a self, iterator: I) -> impl Iterator<Item = T> + 'a
    where
        I: Iterator<Item = T> + 'a,
        T: Default + Copy + Default + 'static,
    {
        return ScalerIterator::<'a, T, IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH, I>::new(
            iterator,
            &self.width_ceil_calcs,
            &self.height_ceil_calcs,
        );
    }
}

struct ScalerIterator<
    'a,
    T,
    const IN_HEIGHT: usize,
    const IN_WIDTH: usize,
    const OUT_HEIGHT: usize,
    const OUT_WIDTH: usize,
    I: Iterator<Item = T>,
> {
    iterator: I,
    input_current_scan_line: u16,
    output_current_scan_line: u16,
    scaled_scan_line_buffer: Vec<I::Item>,
    width_ceil_calcs: &'a [u16],
    height_ceil_calcs: &'a [u16],
    scaled_line_buffer_repeat: u16,
    current_scaled_line_index: u16,
}

impl<
        'a,
        T,
        const IN_HEIGHT: usize,
        const IN_WIDTH: usize,
        const OUT_HEIGHT: usize,
        const OUT_WIDTH: usize,
        I,
    > ScalerIterator<'a, T, IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH, I>
where
    I: Iterator<Item = T>,
    T: Default + Copy,
{
    pub fn new(iterator: I, width_ceil_calcs: &'a [u16], height_ceil_calcs: &'a [u16]) -> Self {
        Self {
            iterator: iterator,
            input_current_scan_line: 0,
            output_current_scan_line: 0,
            scaled_scan_line_buffer: alloc::vec![T::default(); OUT_WIDTH],
            scaled_line_buffer_repeat: 0,
            current_scaled_line_index: 0,
            width_ceil_calcs,
            height_ceil_calcs,
        }
    }
}

impl<
        'a,
        T,
        I,
        const IN_HEIGHT: usize,
        const IN_WIDTH: usize,
        const OUT_HEIGHT: usize,
        const OUT_WIDTH: usize,
    > Iterator for ScalerIterator<'a, T, IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH, I>
where
    I: Iterator<Item = T>,
    T: Copy,
{
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.scaled_line_buffer_repeat > 0 {
                let pixel = self.scaled_scan_line_buffer[self.current_scaled_line_index as usize];

                let next_current_scaled_line_index = self.current_scaled_line_index + 1;
                if next_current_scaled_line_index < OUT_WIDTH as u16 {
                    self.current_scaled_line_index = next_current_scaled_line_index;
                } else {
                    self.scaled_line_buffer_repeat -= 1;
                    self.current_scaled_line_index = 0;
                }
                return Some(pixel);
            }

            //Collect all pixes from a scan line
            let mut next_x_position = 0;
            for count in 0..IN_WIDTH {
                let pixel = self.iterator.next();
                if pixel.is_none() {
                    return None;
                }

                let last_pixel = self.width_ceil_calcs[count] as u16;
                self.scaled_scan_line_buffer[(next_x_position as usize)..last_pixel as usize]
                    .fill(pixel.unwrap());

                next_x_position = last_pixel;
            }
            //Calculate y position of the next scan line
            let next_scan_line_start =
                self.height_ceil_calcs[(self.input_current_scan_line) as usize] as u16;
            //How many scan lines are in bewteen the previous last scan line and the next, this is the amount of scan line repetitions needed for Y scaling

            self.scaled_line_buffer_repeat = next_scan_line_start - self.output_current_scan_line;
            self.output_current_scan_line =
                self.output_current_scan_line + self.scaled_line_buffer_repeat;

            //Calculate y position of the next scan line
            if self.input_current_scan_line >= IN_HEIGHT as u16 - 1 {
                self.output_current_scan_line = 0;
                self.input_current_scan_line = 0;
            } else {
                self.input_current_scan_line += 1;
            }
        }
    }
}

fn generate_scaling_ratio(ratio: f32, size: usize, array: &mut [u16]) {
    let mut i = 0;
    while i < size {
        array[i] = num_traits::Float::ceil(ratio * (i + 1) as f32) as u16;
        i += 1;
    }
}
