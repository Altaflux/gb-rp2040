mod exclusive;
mod shared;
use core::fmt::{self, Debug, Display, Formatter};
use embedded_hal::spi::{Error, ErrorKind};
pub use exclusive::*;
#[derive(Copy, Clone, Eq, PartialEq, Debug)]

pub enum DeviceError<BUS, CS> {
    /// An inner SPI bus operation failed.
    Spi(BUS),
    /// Asserting or deasserting CS failed.
    Cs(CS),
}

impl<BUS, CS> Error for DeviceError<BUS, CS>
where
    BUS: Error + Debug,
    CS: Debug,
{
    #[inline]
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Spi(e) => e.kind(),
            Self::Cs(_) => ErrorKind::ChipSelectFault,
        }
    }
}

/// Dummy [`DelayNs`](embedded_hal::delay::DelayNs) implementation that panics on use.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct NoDelay;

#[cold]
fn no_delay_panic() {
    panic!("You've tried to execute a SPI transaction containing a `Operation::DelayNs` in a `SpiDevice` created with `new_no_delay()`. Create it with `new()` instead, passing a `DelayNs` implementation.");
}

impl embedded_hal::delay::DelayNs for NoDelay {
    #[inline]
    fn delay_ns(&mut self, _ns: u32) {
        no_delay_panic();
    }
}
