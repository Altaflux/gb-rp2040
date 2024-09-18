use core::cell::RefCell;

use alloc::boxed::Box;
use const_lru::ConstLru;
pub struct SdRomManager<
    'a,
    D: embedded_sdmmc::BlockDevice,
    T: embedded_sdmmc::TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> {
    file: RefCell<embedded_sdmmc::File<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>>,
    bank_0: Box<[u8; 0x4000]>,
    bank_lru: RefCell<ConstLru<usize, Box<[u8; 0x4000]>, 4, u8>>,
}
impl<
        'a,
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    > SdRomManager<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    pub fn new(mut file: embedded_sdmmc::File<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>) -> Self {
        let mut bank_0 = Box::new([0u8; 0x4000]);
        file.seek_from_start(0u32).unwrap();
        file.read(&mut *bank_0).unwrap();

        let result = Self {
            bank_0: bank_0,
            file: RefCell::new(file),
            bank_lru: RefCell::new(ConstLru::new()),
        };

        result
    }
    fn read_bank(&self, bank_offset: usize) -> Box<[u8; 0x4000]> {
        let mut buffer: Box<[u8; 0x4000]> = Box::new([0u8; 0x4000]);
        let mut file = self.file.borrow_mut();
        file.seek_from_start(bank_offset as u32).unwrap();
        file.read(&mut *buffer).unwrap();
        buffer
    }
}
impl<
        'a,
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    > gb_core::hardware::rom::RomManager
    for SdRomManager<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    fn read_from_offset(&self, seek_offset: usize, index: usize) -> u8 {
        if seek_offset == 0x0000 {
            return self.bank_0[index as usize];
        }
        let mut bank_lru = self.bank_lru.borrow_mut();
        let bank = bank_lru.get(&seek_offset);
        let value = match bank {
            Some(buffer) => buffer[index],
            None => {
                let buffer: Box<[u8; 0x4000]> = self.read_bank(seek_offset);
                let result = buffer[index];
                bank_lru.insert(seek_offset, buffer);
                result
            }
        };
        value
    }
}
impl<
        'a,
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    > core::ops::Index<usize> for SdRomManager<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    type Output = u8;

    fn index(&self, index: usize) -> &Self::Output {
        &self.bank_0[index as usize]
    }
}
impl<
        'a,
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    > core::ops::Index<core::ops::Range<usize>>
    for SdRomManager<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    type Output = [u8];

    fn index(&self, index: core::ops::Range<usize>) -> &Self::Output {
        return &self.bank_0[index];
    }
}
