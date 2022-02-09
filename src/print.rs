#[allow(unused_macros)]
macro_rules! debug {
	($($arg:tt)*) => {{
		use core::mem::MaybeUninit;
		use crate::hal::{serial, stm32};
		use core::fmt::Write;

		#[allow(unused_unsafe)]
		let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
		write!(tx, $($arg)*).ok();
	}}
}
#[allow(unused_macros)]
macro_rules! debugln {
	($($arg:tt)*) => {{
		use core::mem::MaybeUninit;
		use crate::hal::{serial, stm32};
		use core::fmt::Write;

		#[allow(unused_unsafe)]
		let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
		writeln!(tx, $($arg)*).ok();
	}}
}

pub(crate) use debug;
pub(crate) use debugln;
