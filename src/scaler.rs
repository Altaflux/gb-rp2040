use core::cell::{Ref, RefCell, RefMut};

//New Scaler
use alloc::{boxed::Box, rc::Rc, vec::Vec};

use crate::const_math::{ceilf, floorf};

pub struct ScreenScaler<
    const IN_HEIGHT: usize,
    const IN_WIDTH: usize,
    const OUT_HEIGHT: usize,
    const OUT_WIDTH: usize,
> {
    scaled_scan_line_buffer: Rc<Box<[u16; OUT_WIDTH]>>,
    width_ceil_calcs: Rc<Box<[u16; OUT_WIDTH]>>,
    height_ceil_calcs: Rc<Box<[u16; OUT_HEIGHT]>>,
    out_width_size: u16,
}

impl<
        const IN_HEIGHT: usize,
        const IN_WIDTH: usize,
        const OUT_HEIGHT: usize,
        const OUT_WIDTH: usize,
    > ScreenScaler<IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH>
{
    pub fn new(out_width: u16, out_height: u16) -> Self {
        let calc_out_width_frac = out_width as f32 / IN_WIDTH as f32;
        let calc_out_height_frac = out_height as f32 / IN_HEIGHT as f32;

        let mut width_ceil_calcs_1 = Box::new([0u16; OUT_WIDTH]);
        let mut height_ceil_calcs_1 = Box::new([0u16; OUT_HEIGHT]);
        gen_ceil_array_box_mut(
            calc_out_width_frac,
            out_width as usize,
            &mut *width_ceil_calcs_1,
        );
        gen_ceil_array_box_mut(
            calc_out_height_frac,
            out_height as usize,
            &mut *height_ceil_calcs_1,
        );

        Self {
            scaled_scan_line_buffer: Rc::new(Box::new([0u16; OUT_WIDTH])),
            width_ceil_calcs: Rc::new((width_ceil_calcs_1)),
            height_ceil_calcs: Rc::new((height_ceil_calcs_1)),
            out_width_size: out_width as u16,
        }
    }

    pub fn get_iter<I>(
        &mut self,
        iterator: I,
    ) -> ScalerIter<IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH, I>
    where
        I: Iterator<Item = u16>,
    {
        let ite: ScalerIter<IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH, I> = ScalerIter::new_iter(
            iterator,
            Rc::clone(&self.scaled_scan_line_buffer),
            self.out_width_size,
            Rc::clone(&self.width_ceil_calcs),
            Rc::clone(&self.height_ceil_calcs),
        );
        return ite;
    }
}

impl<
        const IN_HEIGHT: usize,
        const IN_WIDTH: usize,
        const OUT_HEIGHT: usize,
        const OUT_WIDTH: usize,
    > Clone for ScreenScaler<IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH>
{
    fn clone(&self) -> Self {
        Self {
            scaled_scan_line_buffer: self.scaled_scan_line_buffer.clone(),
            width_ceil_calcs: self.width_ceil_calcs.clone(),
            height_ceil_calcs: self.height_ceil_calcs.clone(),
            out_width_size: self.out_width_size.clone(),
        }
    }
}

pub struct ScalerIter<
    const IN_HEIGHT: usize,
    const IN_WIDTH: usize,
    const OUT_HEIGHT: usize,
    const OUT_WIDTH: usize,
    I: Iterator<Item = u16>,
> {
    iterator: I,
    input_current_scan_line: u16,
    output_current_scan_line: u16,
    scaled_scan_line_buffer: Rc<Box<[I::Item; OUT_WIDTH]>>,
    scaled_scan_line_buffer2: Box<[u16; OUT_WIDTH]>,
    scaled_scan_line_buffer3: Vec<u16>,
    width_ceil_calcs: Rc<Box<[I::Item; OUT_WIDTH]>>,
    height_ceil_calcs: Rc<Box<[I::Item; OUT_HEIGHT]>>,
    scaled_line_buffer_repeat: u16,
    current_scaled_line_index: u16,
    out_width_size: u16,
}

impl<
        const IN_HEIGHT: usize,
        const IN_WIDTH: usize,
        const OUT_HEIGHT: usize,
        const OUT_WIDTH: usize,
        I,
    > ScalerIter<IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH, I>
where
    I: Iterator<Item = u16>,
{
    const WIDTH_CEIL_CALCS: [I::Item; OUT_WIDTH] =
        gen_ceil_array(OUT_WIDTH as f32 / IN_WIDTH as f32);
    const HEIGHT_CEIL_CALCS: [I::Item; OUT_HEIGHT] =
        gen_ceil_array(OUT_HEIGHT as f32 / IN_HEIGHT as f32);

    pub fn new_iter(
        iterator: I,

        scaled_scan_line_buffer: Rc<Box<[I::Item; OUT_WIDTH]>>,
        out_width: u16,
        width_ceil_calcs: Rc<Box<[I::Item; OUT_WIDTH]>>,
        height_ceil_calcs: Rc<Box<[I::Item; OUT_HEIGHT]>>,
    ) -> Self {
        Self {
            iterator: iterator,
            input_current_scan_line: 0,
            output_current_scan_line: 0,
            scaled_scan_line_buffer: scaled_scan_line_buffer,
            scaled_scan_line_buffer2: Box::new([0u16; OUT_WIDTH]),
            scaled_scan_line_buffer3: alloc::vec![0; OUT_WIDTH],
            scaled_line_buffer_repeat: 0,
            current_scaled_line_index: 0,
            width_ceil_calcs,
            height_ceil_calcs,
            out_width_size: out_width as u16,
        }
    }
}

impl<
        I,
        const IN_HEIGHT: usize,
        const IN_WIDTH: usize,
        const OUT_HEIGHT: usize,
        const OUT_WIDTH: usize,
    > Iterator for ScalerIter<IN_HEIGHT, IN_WIDTH, OUT_HEIGHT, OUT_WIDTH, I>
where
    I: Iterator<Item = u16>,
{
    type Item = u16;
    //#[unroll::unroll_for_loops]
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.scaled_line_buffer_repeat > 0 {
                let pixel = self.scaled_scan_line_buffer2[self.current_scaled_line_index as usize];

                let next_current_scaled_line_index = self.current_scaled_line_index + 1;
                if next_current_scaled_line_index < OUT_WIDTH as u16 {
                    //if next_current_scaled_line_index < self.out_width_size {
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
                //let last_pixel = Self::WIDTH_CEIL_CALCS[count] as u16;
                self.scaled_scan_line_buffer2[(next_x_position as usize)..last_pixel as usize]
                    .fill(pixel.unwrap());

                next_x_position = last_pixel;
            }

            //Calculate y position of the next scan line
            let next_scan_line_start =
                self.height_ceil_calcs[(self.input_current_scan_line + 1) as usize] as u16;
            // let next_scan_line_start =
            //     Self::HEIGHT_CEIL_CALCS[(self.input_current_scan_line + 1) as usize] as u16;
            //How many scan lines are in bewteen the previous last scan line and the next, this is the amount of scan line repetitions needed for Y scaling

            self.scaled_line_buffer_repeat =
                (next_scan_line_start - self.output_current_scan_line) - 0;

            self.output_current_scan_line = next_scan_line_start;

            if self.input_current_scan_line >= IN_HEIGHT as u16 - 1 {
                self.output_current_scan_line = 0;
                self.input_current_scan_line = 0;
            } else {
                self.input_current_scan_line += 1;
            }
        }
    }
}

const fn gen_ceil_array<const N: usize>(ratio: f32) -> [u16; N] {
    let mut res = [0 as u16; N];

    let mut i = 0;

    while i < N as i32 {
        res[i as usize] = ceilf(ratio * i as f32) as u16;
        i += 1;
    }

    res
}

fn gen_ceil_array2(ratio: f32, size: usize) -> Vec<u16> {
    let mut res = Vec::with_capacity(size);

    let mut i = 0;

    while i < size as i32 {
        res.push(ceilf(ratio * i as f32) as u16);
        i += 1;
    }

    res
}

fn gen_ceil_array_box_mut(ratio: f32, size: usize, array: &mut [u16]) {
    let mut i = 0;
    while i < size as i32 {
        array[i as usize] = ceilf(ratio * i as f32) as u16;
        i += 1;
    }
}

fn gen_ceil_array3(ratio: u16, size: usize) -> Vec<u16> {
    let mut res = Vec::with_capacity(size);

    let mut i = 0;

    while i < size as u16 {
        res.push(((ratio * i) as u16));
        //res.push(0);
        i += 1;
    }

    res
}
