use core::cell::{Ref, RefCell, RefMut};

//New Scaler
use alloc::{rc::Rc, vec::Vec};

use crate::const_math::ceilf;

pub struct ScreenScaler<const IN_HEIGHT: usize, const IN_WIDTH: usize> {
    scaled_scan_line_buffer: Rc<RefCell<Vec<u16>>>,

    width_ceil_calcs: Rc<RefCell<Vec<u16>>>,
    height_ceil_calcs: Rc<RefCell<Vec<u16>>>,
    // width_ceil_calcs: RefCell<Vec<u16>>,
    // height_ceil_calcs: RefCell<Vec<u16>>,
    out_width_size: u16,
}

impl<const IN_HEIGHT: usize, const IN_WIDTH: usize> ScreenScaler<IN_HEIGHT, IN_WIDTH> {
    pub fn new(out_width: u16, out_height: u16) -> Self {
        let width_ceil_calcs: Vec<u16> =
            gen_ceil_array2(out_width as f32 / IN_WIDTH as f32, out_width as usize);
        let height_ceil_calcs: Vec<u16> =
            gen_ceil_array2(out_height as f32 / IN_HEIGHT as f32, out_height as usize);

        Self {
            scaled_scan_line_buffer: Rc::new(RefCell::new(alloc::vec![0; out_width as usize])),
            width_ceil_calcs: Rc::new(RefCell::new(width_ceil_calcs)),
            height_ceil_calcs: Rc::new(RefCell::new(height_ceil_calcs)),
            // width_ceil_calcs: Rc::new(RefCell::new(width_ceil_calcs)),
            // height_ceil_calcs: Rc::new(RefCell::new(height_ceil_calcs)),
            out_width_size: out_width as u16,
        }
    }
    // pub fn get_iter2(&mut self, iterator: I) -> () {
    //     let ite: ScalerIter<'_, IN_HEIGHT, IN_WIDTH, I> = ScalerIter::new_iter(
    //         iterator,
    //         self.out_width_size,
    //         &mut self.scaled_scan_line_buffer.borrow_mut(),
    //         &self.width_ceil_calcs.borrow(),
    //         &self.height_ceil_calcs.borrow(),
    //     );
    //     return ();
    // }
    pub fn get_iter<I>(&mut self, iterator: I) -> ScalerIter<'_, IN_HEIGHT, IN_WIDTH, I>
    where
        I: Iterator<Item = u16>,
    {
        let ite: ScalerIter<'_, IN_HEIGHT, IN_WIDTH, I> = ScalerIter::new_iter(
            iterator,
            self.out_width_size,
            self.scaled_scan_line_buffer.borrow_mut(),
            self.width_ceil_calcs.borrow(),
            self.height_ceil_calcs.borrow(),
        );
        return ite;
    }
}

impl<const IN_HEIGHT: usize, const IN_WIDTH: usize> Clone for ScreenScaler<IN_HEIGHT, IN_WIDTH> {
    fn clone(&self) -> Self {
        Self {
            scaled_scan_line_buffer: self.scaled_scan_line_buffer.clone(),
            width_ceil_calcs: self.width_ceil_calcs.clone(),
            height_ceil_calcs: self.height_ceil_calcs.clone(),
            out_width_size: self.out_width_size.clone(),
        }
    }
}

pub struct ScalerIter<'a, const IN_HEIGHT: usize, const IN_WIDTH: usize, I: Iterator<Item = u16>> {
    iterator: I,
    input_current_scan_line: u16,
    output_current_scan_line: u16,
    scaled_scan_line_buffer: RefMut<'a, Vec<I::Item>>,
    scaled_line_buffer_repeat: u16,
    current_scaled_line_index: u16,
    width_ceil_calcs: Ref<'a, Vec<I::Item>>,
    height_ceil_calcs: Ref<'a, Vec<I::Item>>,
    out_width_size: u16,
}

impl<'a, const IN_HEIGHT: usize, const IN_WIDTH: usize, I> ScalerIter<'a, IN_HEIGHT, IN_WIDTH, I>
where
    I: Iterator<Item = u16>,
{
    pub fn new_iter(
        iterator: I,
        out_width: u16,
        scaled_scan_line_buffer: RefMut<'a, Vec<I::Item>>,
        width_ceil_calcs: Ref<'a, Vec<I::Item>>,
        height_ceil_calcs: Ref<'a, Vec<I::Item>>,
    ) -> Self {
        Self {
            iterator: iterator,
            input_current_scan_line: 0,
            output_current_scan_line: 0,
            scaled_scan_line_buffer: scaled_scan_line_buffer,
            scaled_line_buffer_repeat: 0,
            current_scaled_line_index: 0,
            width_ceil_calcs,
            height_ceil_calcs,
            out_width_size: out_width as u16,
        }
    }
}

impl<'a, I, const IN_HEIGHT: usize, const IN_WIDTH: usize> Iterator
    for ScalerIter<'a, IN_HEIGHT, IN_WIDTH, I>
where
    I: Iterator<Item = u16>,
{
    type Item = u16;
    //#[unroll::unroll_for_loops]
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.scaled_line_buffer_repeat > 0 {
                let pixel = self.scaled_scan_line_buffer[self.current_scaled_line_index as usize];

                let next_current_scaled_line_index = self.current_scaled_line_index + 1;
                if next_current_scaled_line_index < self.out_width_size {
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
                self.height_ceil_calcs[(self.input_current_scan_line + 1) as usize] as u16;
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
        res.push(num_traits::Float::ceil(ratio * i as f32) as u16);
        i += 1;
    }

    res
}
