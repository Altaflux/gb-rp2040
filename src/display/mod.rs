mod dma_streamer;
mod dma_transfer;
mod parallel_8bit_interface;
mod spi_pio_interface;

pub use dma_streamer::DmaStreamer;
use dma_transfer::DmaTransfer;

pub use parallel_8bit_interface::Parallel8BitDmaInterface;
pub use spi_pio_interface::SpiPioDmaInterface;

pub trait LineTransfer {
    type Item;
    fn send_scanline(
        &mut self,
        line: &'static mut [Self::Item],
        size: u32,
    ) -> &'static mut [Self::Item];
}
