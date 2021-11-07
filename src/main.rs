#![no_main]
#![no_std]

mod null_waker;

use core::future::Future;
pub(crate) use core::pin::Pin;
use core::cell::RefCell;

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use crate::hal::serial;
use core::fmt::Write;

use crate::hal::{prelude::*, stm32};

use core::task::{Poll, Context};

#[allow(unused_macros)]
macro_rules! debug {
	($($arg:tt)*) => {{
		use core::mem::MaybeUninit;
		#[allow(unused_unsafe)]
		let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
		write!(tx, $($arg)*).ok();
	}}
}
#[allow(unused_macros)]
macro_rules! debugln {
	($($arg:tt)*) => {{
		use core::mem::MaybeUninit;
		#[allow(unused_unsafe)]
		let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
		writeln!(tx, $($arg)*).ok();
	}}
}


#[derive(Debug)]
enum UsbLoop {
	Connected,
	Disconnected,
	IgnoreThis
}

enum TransferState {
	WaitingForAvailableChannel,
	WaitingForTransferToFinish(u8)
}

struct UsbGlobals {
	grxsts: Option<stm32f4xx_hal::pac::otg_fs_global::grxstsp_host::R>,
	usb_host: &'static stm32f4xx_hal::pac::OTG_FS_HOST,
}

#[derive(Clone, Copy)]
enum DataPid {
	Data0 = 0,
	Data2 = 1,
	Data1 = 2,
	MdataSetup = 3
}

struct UsbOutTransfer<'a> {
	data: &'a [u8],
	state: TransferState,
	globals: &'a RefCell<UsbGlobals>,
	endpoint_type: EndpointType,
	endpoint_number: u8,
	device_address: u8,
	data_pid: DataPid,
	packet_size: u16,
	is_lowspeed: bool,
	last_error: Option<TransferError>
}

struct UsbInTransfer<'a> {
	data: &'a mut [u8],
	state: TransferState,
	globals: &'a RefCell<UsbGlobals>,
	rx_pointer: usize,
	endpoint_type: EndpointType,
	endpoint_number: u8,
	device_address: u8,
	data_pid: DataPid,
	packet_size: u16,
	is_lowspeed: bool,
	last_error: Option<TransferError>
}

#[derive(Copy, Clone, Debug)]
enum TransferError {
	DataToggleError,
	FrameOverrun,
	BabbleError,
	TransactionError,
	Nak,
	Stall,
	Unknown
}

impl Future for UsbInTransfer<'_> {
	type Output = Result<usize, TransferError>;

	fn poll(mut self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Result<usize, TransferError>> {
		let globals = self.globals.borrow_mut();
		let usb_host = globals.usb_host;
		//let tx = self.globals.tx;
		match self.state {
			TransferState::WaitingForAvailableChannel => {
				let available_channel = (0..8).find(|i| usb_host.hccharx(*i).read().chena().bit_is_clear());
				if let Some(channel) = available_channel {
					unsafe {
						usb_host.hctsizx(channel).write(|w| w
							.dpid().bits(self.data_pid as u8)
							.pktcnt().bits(div_ceil(self.data.len(), self.packet_size as usize) as u16)
							.xfrsiz().bits(self.data.len() as u32)
						);
						usb_host.hccharx(channel).write(|w| w
							.dad().bits(self.device_address)
							.mcnt().bits(1)
							.epdir().set_bit()
							.lsdev().bit(self.is_lowspeed)
							.epnum().bits(self.endpoint_number)
							.eptyp().bits(self.endpoint_type as u8)
							.mpsiz().bits(self.packet_size)
							.chena().set_bit()
						);
					}

					self.state = TransferState::WaitingForTransferToFinish(channel);
				}
			}
			TransferState::WaitingForTransferToFinish(channel) => {
				if let Some(ref grxsts) = globals.grxsts {
					if grxsts.chnum().bits() == channel {
						let packet_status = PacketStatus::from(grxsts.pktsts().bits());
						debugln!("#{}: read ch={} dpid={} bcnt={} pktsts={} {:?}", usb_host.hfnum.read().frnum().bits(), grxsts.chnum().bits(), grxsts.dpid().bits(), grxsts.bcnt().bits(), grxsts.pktsts().bits(), packet_status
						);

						/*
						if (error_condition) {
							// FIXME: cleanup
							return Poll::Ready(Err(TODO));
						}*/

						match packet_status {
							PacketStatus::InDataPacketReceived => {
								let len = grxsts.bcnt().bits() as usize;
								for i in (0..len).step_by(4) {
									let fifo_word = unsafe { core::ptr::read_volatile((0x50001000 + (channel as usize) * 0x1000) as *mut [u8; 4]) };
									let offset = self.rx_pointer + i;
									let remaining = usize::min(len - i, 4);
									self.data[offset..(offset + remaining)].copy_from_slice(&fifo_word[0..remaining]);
								}
								self.rx_pointer += len;
								if self.rx_pointer < self.data.len() {
									usb_host.hccharx(channel).modify(|_, w| w.chena().set_bit()); // FIXME really?
								}

								
							}
							_ => {}
						}
					}
				}
				let hcint = usb_host.hcintx(channel).read();
				usb_host.hcintx(channel).write(|w| unsafe { w.bits(!0) } );
				
				let error =
					if hcint.bberr().bit() { Some(TransferError::BabbleError) }
					else if hcint.dterr().bit() { Some(TransferError::DataToggleError) }
					else if hcint.frmor().bit() { Some(TransferError::FrameOverrun) }
					else if hcint.stall().bit() { Some(TransferError::Stall) }
					else if hcint.txerr().bit() { Some(TransferError::TransactionError) }
					else if hcint.nak().bit() { Some(TransferError::Nak) }
					else { None };

				if error.is_some() {
					debugln!("Error in IN transfer {:?}, disabling channel", error.unwrap());
					usb_host.hccharx(channel).modify(|_, w| w.chdis().set_bit());

					if self.last_error.is_some() {
						debugln!("WARNING: multiple errors in IN transfer");
					}

					self.last_error = error;
				}

				//FIXME is that really wrong? if usb_host.hccharx(channel).read().chena().bit_is_clear() {
				if hcint.xfrc().bit_is_set() && self.last_error.is_none() {
					return Poll::Ready(Ok(self.rx_pointer));
				}
				else if hcint.xfrc().bit_is_set() || hcint.chh().bit_is_set() {
					return Poll::Ready(Err(self.last_error.unwrap_or(TransferError::Unknown)));
				}
			}
		}
		return Poll::Pending;
	}
}

#[derive(Copy, Clone)]
enum EndpointType {
	Control = 0,
	Isochronous = 1,
	Bulk = 2,
	Interrupt = 3
}

fn div_ceil(a: usize, b: usize) -> usize {
	(a+b-1)/b
}

#[derive(Copy, Clone, Debug)]
enum PacketStatus {
	InDataPacketReceived,
	InTransferCompleted,
	DataToggleError,
	ChannelHalted,
	Unknown
}

impl From<u8> for PacketStatus {
	fn from(val: u8) -> PacketStatus {
		use PacketStatus::*;
		match val {
			2 => InDataPacketReceived,
			3 => InTransferCompleted,
			5 => DataToggleError,
			7 => ChannelHalted,
			_ => Unknown
		}
	}
}

impl Future for UsbOutTransfer<'_> {
	type Output = Result<(), TransferError>;

	fn poll(mut self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Self::Output> {
		let globals = self.globals.borrow_mut();
		let usb_host = globals.usb_host;
		match self.state {
			TransferState::WaitingForAvailableChannel => {
				let available_channel = (0..8).find(|i| usb_host.hccharx(*i).read().chena().bit_is_clear());
				if let Some(channel) = available_channel {
					unsafe {
						usb_host.hctsizx(channel).write(|w| w
							.dpid().bits(self.data_pid as u8)
							.pktcnt().bits( u16::max(1, div_ceil(self.data.len(), self.packet_size as usize) as u16))
							.xfrsiz().bits(self.data.len() as u32)
						);
						usb_host.hccharx(channel).write(|w| w
							.dad().bits(self.device_address)
							.mcnt().bits(1)
							.epdir().clear_bit()
							.lsdev().bit(self.is_lowspeed)
							.epnum().bits(self.endpoint_number) // 1 == in
							.eptyp().bits(self.endpoint_type as u8) // 0 == control
							.mpsiz().bits(self.packet_size)
							.chena().set_bit()
						);

						// NOTE: setting pktcnt correctly is *not* needed to send a packet.  Instead, always 
						// ceil(xfrsiz / mpsiz) packets are sent; the first n-1 packets are mpsiz long, the last has the
						// remaining length.  However, only after `pktcount` packets have been sent, a "transfer
						// complete" interrupt is generated. So even if *sending* works without setting pktcount,
						// getting notified after the send completes would not work.

						// NOTE: the CHENA is pure user convenience. It does *not* control actual activation of the
						// channel (which is triggered by writing enough words to the FIFO). But the application will
						// clear this bit after successfully transmitting `pktcnt` packets, i.e. at the same time when
						// the "transmission complete" interrupt is generated. If the user sets the bit, they can easily
						// poll which channels are free; however, the user *could* implement this bookkeeping using the
						// transmission complete interrupt with their own array, if they wanted to.

						// NOTE: it does not matter where to write in the area 0x50001000 to 0x50001FFF. You can write
						// your four-byte-chunks in ascending address order (as memcpy would do), but you can also write
						// them all to the same address (e.g. 0x50001000) or in descending addresses. I.e. reverse
						// memcpy would *not* cause the same result as memcpy.  we need to write word-wise into the
						// fifo. excess bytes seem to be ignored. i.e., to transmit 6 bytes, AA BB CC DD EE FF, we need
						// to write 0xAABBCCDD and 0xEEFF4242.
						
						// FIXME ensure the fifo has enough space!
						for chunk in self.data.chunks(4) {
							let mut tmp = [0; 4];
							tmp[0..chunk.len()].copy_from_slice(chunk);
							core::ptr::write_volatile((0x50001000 + (channel as usize) * 0x1000) as *mut [u8; 4], tmp);
						}
						//debugln!("gnptxsts = {:08x}, hptxfsiz = {:08x}", otg_fs_global.gnptxsts.read().bits(), otg_fs_global.hptxfsiz.read().bits());
					}

					self.state = TransferState::WaitingForTransferToFinish(channel);
				}
			}
			TransferState::WaitingForTransferToFinish(channel) => {
				let hcint = usb_host.hcintx(channel).read();
				usb_host.hcintx(channel).write(|w| unsafe { w.bits(!0) } );
				
				let error =
					if hcint.bberr().bit() { Some(TransferError::BabbleError) }
					else if hcint.dterr().bit() { Some(TransferError::DataToggleError) }
					else if hcint.frmor().bit() { Some(TransferError::FrameOverrun) }
					else if hcint.stall().bit() { Some(TransferError::Stall) }
					else if hcint.txerr().bit() { Some(TransferError::TransactionError) }
					else if hcint.nak().bit() { Some(TransferError::Nak) }
					else { None };

				if error.is_some() {
					debugln!("Error in OUT transfer {:?}, disabling channel", error.unwrap());
					usb_host.hccharx(channel).modify(|_, w| w.chdis().set_bit());
					
					if self.last_error.is_some() {
						debugln!("WARNING: multiple errors in OUT transfer");
					}

					self.last_error = error;
				}

				//debug!("{}", hcint.xfrc().bit());
				//FIXME is that really wrong? if usb_host.hccharx(channel).read().chena().bit_is_clear() {
				if hcint.xfrc().bit_is_set() && self.last_error.is_none() {
					return Poll::Ready(Ok(()));
				}
				else if hcint.xfrc().bit_is_set() || hcint.chh().bit_is_set() {
					return Poll::Ready(Err(self.last_error.unwrap_or(TransferError::Unknown)));
				}
			}
		}
		return Poll::Pending;
	}
}

struct SleepFuture {
	// TODO: check systick against end time.
}

trait Fnord {
	fn hcintx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINT0;
	fn hccharx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0;
	fn hcintmskx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0;
	fn hctsizx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0;
}

impl Fnord for stm32f4xx_hal::pac::OTG_FS_HOST {
	fn hcintx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINT0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCINT0 = &self.hcint0;
		unsafe { &*((ptr as usize).wrapping_add(i as usize * 0x20) as *const stm32f4xx_hal::stm32::otg_fs_host::HCINT0) }
	}
	fn hcintmskx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0 = &self.hcintmsk0;
		unsafe { &*((ptr as usize).wrapping_add(i as usize * 0x20) as *const stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0) }
	}
	fn hctsizx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0 = &self.hctsiz0;
		unsafe { &*((ptr as usize).wrapping_add(i as usize * 0x20) as *const stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0) }
	}
	fn hccharx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0 = &self.hcchar0;
		unsafe { &*((ptr as usize).wrapping_add(i as usize * 0x20) as *const stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0) }
	}
}


#[entry]
fn main() -> ! {
	if let (Some(dp), Some(cp)) = (
		stm32::Peripherals::take(),
		cortex_m::peripheral::Peripherals::take(),
	) {
		let rcc = dp.RCC.constrain();
		let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(84.mhz()).freeze();

		//let gpioc = dp.GPIOC.split();
		//let mut led = gpioc.pc13.into_push_pull_output();

		let gpioa = dp.GPIOA.split();
		let dm_pin = gpioa.pa11.into_alternate::<10>().internal_pull_down(true);
		let dp_pin = gpioa.pa12.into_alternate::<10>().internal_pull_down(true);

		// Create a delay abstraction based on SysTick
		let mut delay = hal::delay::Delay::new(cp.SYST, &clocks);


		let mut trigger_pin = gpioa.pa0.into_push_pull_output();
		trigger_pin.set_low();

		// Configure the USART
		let gpio_tx = gpioa.pa9.into_alternate();
		let gpio_rx = gpioa.pa10.into_alternate();
		let serial = serial::Serial::new(
			dp.USART1,
			(gpio_tx, gpio_rx),
			serial::config::Config::default().baudrate(115200.bps()),
			clocks
		).unwrap();
		let (mut tx, _rx) = serial.split();
		writeln!(tx, "hello world!").ok();


		let otg_fs_global = dp.OTG_FS_GLOBAL;
		let otg_fs_host = dp.OTG_FS_HOST;

		unsafe {
			// enable OTG clock
			let rcc = &(*crate::stm32::RCC::ptr());
			rcc.ahb2enr.modify(|_,w| { w.otgfsen().set_bit() } );

			/*
			// soft reset
			writeln!(tx, "resetting").ok();
			otg_fs_global.grstctl.modify(|_,w| w.csrst().set_bit());
			while otg_fs_global.grstctl.read().csrst().bit() { yield None; }
			writeln!(tx, "reset done").ok();
			delay.delay_ms(1_u32); // wait at least 3 PHY clocks

			// wait for AHB idle (docs for CSRST require this)
			writeln!(tx, "waiting for AHBIDL").ok();
			while !otg_fs_global.grstctl.read().ahbidl().bit() { yield None; }
			*/

			// "Core initialization" step 1
			otg_fs_global.gahbcfg.modify(|_,w| w.gint().set_bit());
			// FIXME rxflvl, txfifo empty level
			//otg_fs_global.gusbcfg.modify(|_,w| w.physel().set_bit()); // this bit is always 1

			// "Core initialization" step 2
			otg_fs_global.gusbcfg.modify(|_,w| w
				.hnpcap().clear_bit()
				.srpcap().clear_bit()
				//.trdt().bits(0xF) // FIXME can/should be smaller
				//.tocal().bits(8) // FIXME same
				.fdmod().clear_bit() // must wait for 25ms before the change takes effect
				.fhmod().set_bit() // must wait for 25ms before the change takes effect
			);
		
			
			// configure vbus sensing
			otg_fs_global.gccfg.modify(|_, w| w
			//	.vbusasen().set_bit()
			//	//.vbusbsen().set_bit()
				.pwrdwn().set_bit()
			);
			//otg_fs_global.gccfg.modify(|r, w| w.bits( r.bits() | (1<<21) )); // NOVBUSSENS
			

			// "Core initialization" step 3
			otg_fs_global.gintmsk.modify(|_,w| w // FIXME FIXME FIXME the C code has them all zero
				.otgint().set_bit()
				.mmism().set_bit()
				.sofm().set_bit()
//				.prtim().set_bit() // FIXME: "only accessible in host mode", why is this inaccessible then?!
			);
			// "Host initialization" step 1
			otg_fs_global.gintmsk.modify(|r, w| w.bits( r.bits() | (1<<24) )); // PRTIM


			// "Host initialization" step 2
			otg_fs_host.hcfg.modify(|_,w| w.fslspcs().bits(1)); // 48MHz
			otg_fs_host.hcfg.modify(|r, w| w.bits( r.bits() | (1<<2) )); // FSLSS
			
			
			loop {
				// "Host initialization" steps 3-4: enable PPWR and wait for PCDET
				writeln!(tx, "setting PPWR and waiting for PCDET").ok();
				otg_fs_host.hprt.modify(|_,w| w.ppwr().set_bit()); // FIXME unsure if that's needed
				while !otg_fs_host.hprt.read().pcdet().bit() { }
				writeln!(tx, "got PCDET!").ok();
				
				// "Host initialization" steps 5-9
				writeln!(tx, "resetting the port and waiting for PENCHNG").ok();
				otg_fs_host.hprt.modify(|_,w| w.prst().set_bit());
				delay.delay_ms(15_u32);
				otg_fs_host.hprt.modify(|_,w| w.prst().clear_bit());
				delay.delay_ms(15_u32);

				while !otg_fs_host.hprt.read().penchng().bit() { }
				writeln!(tx, "enumerated speed is {}", otg_fs_host.hprt.read().pspd().bits()).ok();
				
				// "Host initialization" step 10: program hfir
				// TODO: is that needed? seems to autoselect a good value

				// "Host initialization" step 11: set PHY clock
				// TODO: is that needed? we already set 1 = 48MHz here. reset port if changed.
				//otg_fs_host.hcfg.modify(|_,w| w.fslspcs().bits(1)); // 48MHz

				// "Host initialization" steps 12-14
				otg_fs_global.grxfsiz.modify(|_,w| w
					.rxfd().bits(64) // 64 32bit words RX fifo size
				);
				otg_fs_global.hnptxfsiz_mut().modify(|_,w| w
					.nptxfd().bits(64)
					.nptxfsa().bits(64)
				);
				otg_fs_global.hptxfsiz.modify(|_,w| w
					.ptxfsiz().bits(64)
					.ptxsa().bits(64+64)
				);

				// ---- done ----
				otg_fs_global.gintmsk.modify(|_,w| w.hcim().set_bit());

				otg_fs_global.gintmsk.modify(|_,w| w.nptxfem().set_bit());

				// "host programming model"/"channel initialization"



				delay.delay_ms(250_u32);

				otg_fs_global.gintmsk.modify(|_,w| w.hcim().set_bit());
				otg_fs_global.gintsts.modify(|_,w| w.bits(!0));


				writeln!(tx, "gintsts = {:08x}", otg_fs_global.gintsts.read().bits()).ok();
				writeln!(tx, "hprt = {:08x}", otg_fs_host.hprt.read().bits()).ok();
				delay.delay_ms(300_u32);
				writeln!(tx, "hprt = {:08x}", otg_fs_host.hprt.read().bits()).ok();
				let mut sofcount: u32 = 0;
				// TODO: notify client that we have a connection

				loop {
					if otg_fs_global.gintsts.read().sof().bit() {
						otg_fs_global.gintsts.write(|w| w.sof().set_bit());
						//write!(tx, "{:8} {:08x}\r", sofcount, otg_fs_host.hcchar0.read().bits());
						write!(tx, "{:8} {:08x} {:08x}\r", sofcount, otg_fs_host.hfnum.read().bits(), otg_fs_global.gnptxsts.read().bits()).ok();
						//writeln!(tx, "hprt = {:08x}", otg_fs_host.hprt.read().bits()).ok();
						//write!(tx, "{:08x}\r", otg_fs_global.gintsts.read().bits());
						sofcount += 1;
					}

					let frame_number = otg_fs_host.hfnum.read().frnum().bits();

					while otg_fs_global.gintsts.read().srqint().bit() {
						writeln!(tx, "srqint").ok();
						otg_fs_global.gintsts.write(|w| w.srqint().set_bit());
						// TODO do things
					}

					while otg_fs_global.gintsts.read().rxflvl().bit() {
						let rxstsp = otg_fs_global.grxstsp_host().read();
						writeln!(tx, "#{}: read ch={} dpid={} bcnt={} pktsts={} {}", frame_number, rxstsp.chnum().bits(), rxstsp.dpid().bits(), rxstsp.bcnt().bits(), rxstsp.pktsts().bits(),
							match rxstsp.pktsts().bits() {
								2 => "IN data packet received",
								3 => "IN transfer completed",
								5 => "Data toggle error",
								7 => "Channel halted",
								_ => "(unknown)"
							}
						).ok();
						// TODO do things https://github.com/libusbhost/libusbhost/blob/master/src/usbh_lld_stm32f4.c#L322
					}

					// OTGINT
					if otg_fs_global.gintsts.read().otgint().bit() {
						let val = otg_fs_global.gotgint.read().bits();
						writeln!(tx, "otg {:08x}", val).ok();
						otg_fs_global.gotgint.modify(|_,w| w.bits(val));
					}

					// HPRTINT
					if otg_fs_global.gintsts.read().hprtint().bit() {
						let val = otg_fs_host.hprt.read();
						writeln!(tx, "hprt {:08x}", val.bits()).ok();

						if val.penchng().bit() {
							writeln!(tx, "#{}, penchng, port enabled is {}", frame_number, val.pena().bit()).ok();

							for i in 0..8 {
								otg_fs_host.hcintx(i).write(|w| w.bits(!0));
								otg_fs_host.hcintmskx(i).write(|w| w
									.xfrcm().set_bit()
									.chhm().set_bit()
									.stallm().set_bit()
									.nakm().set_bit()
									.ackm().set_bit()
									.txerrm().set_bit()
									.bberrm().set_bit()
									.frmorm().set_bit()
									.dterrm().set_bit()
								);
							}
							
							otg_fs_host.haintmsk.write(|w| w.bits(0xFFFF));
			
							let waker = null_waker::create();
							let mut dummy_context = core::task::Context::from_waker(&waker);
					
							let globals = core::cell::RefCell::new(UsbGlobals {
								grxsts: None,
								usb_host: core::mem::transmute(&otg_fs_host) // FIXME FIXME FIXME FIXME FIXME!!!
							});

							let mut async_block = async {
								let setup_packet = [
									0x00u8, // device, standard, host to device
									0x05, // set address
									0x01, 0x00, // address 1
									0x00, 0x00, // index
									0x00, 0x00, // length
								];

								loop {
									let fnord = UsbOutTransfer {
										data: &setup_packet,
										globals: &globals,
										state: TransferState::WaitingForAvailableChannel,
										endpoint_type: EndpointType::Control,
										endpoint_number: 0,
										device_address: 0,
										data_pid: DataPid::MdataSetup,
										packet_size: 64,
										is_lowspeed: false,
										last_error: None
									};

									trigger_pin.set_high();
									let result = fnord.await;
									trigger_pin.set_low();

									if result.is_ok() {
										break;
									}
								}
								//debugln!("addr out {:?}", result);
								
								let mut zero_byte_buffer = [];

								loop {
									let fnord = UsbInTransfer {
										data: &mut zero_byte_buffer,
										globals: &globals,
										rx_pointer: 0,
										state: TransferState::WaitingForAvailableChannel,
										endpoint_type: EndpointType::Control,
										endpoint_number: 0,
										device_address: 0,
										data_pid: DataPid::Data1,
										packet_size: 64,
										is_lowspeed: false,
										last_error: None
									};

									let result = fnord.await;

									if result.is_ok() {
										break;
									}
								}
								//debugln!("addr in {:?}", result);
							

								let mut ep0_max_size = 64;
								let mut short_read = true;
								let mut descriptor_buffer = [0; 18];
								while short_read {
									debugln!("Retrieving configuration descriptor with maximum packet length of {}", ep0_max_size);
									loop {
										let get_descriptor_packet = [
											0x80u8, // device, standard, host to device
											0x06, // get descriptor
											0x00, 0x01, // descriptor 1
											0x00, 0x00, // index
											18, 0, // length
										];
										let fnord = UsbOutTransfer {
											data: &get_descriptor_packet,
											globals: &globals,
											state: TransferState::WaitingForAvailableChannel,
											endpoint_type: EndpointType::Control,
											endpoint_number: 0,
											device_address: 1,
											data_pid: DataPid::MdataSetup,
											packet_size: 64,
											is_lowspeed: false,
											last_error: None
										};
										trigger_pin.set_high();
										let result = fnord.await;
										//debugln!("descr out {:?}", result);

										if result.is_ok() {
											break;
										}
									}


									loop {
										let fnord = UsbInTransfer {
											data: &mut descriptor_buffer,
											globals: &globals,
											rx_pointer: 0,
											state: TransferState::WaitingForAvailableChannel,
											endpoint_type: EndpointType::Control,
											endpoint_number: 0,
											device_address: 1,
											data_pid: DataPid::Data1,
											packet_size: ep0_max_size,
											is_lowspeed: false,
											last_error: None
										};

										let result = fnord.await;
										trigger_pin.toggle();

										debugln!("descr in {:?}", result);
										if let Ok(size) = result {
											if size != descriptor_buffer.len() {
												if size >= 8 {
													ep0_max_size = descriptor_buffer[7] as u16;
													short_read = true;
												}
												else {
													panic!("Error reading descriptor");
												}
											}
											else {
												short_read = false;
											}

											break;
										}
									}

									// status stage (always DATA1)
									let fnord = UsbOutTransfer {
										data: &[],
										globals: &globals,
										state: TransferState::WaitingForAvailableChannel,
										endpoint_type: EndpointType::Control,
										endpoint_number: 0,
										device_address: 1,
										data_pid: DataPid::Data1,
										packet_size: 64,
										is_lowspeed: false,
										last_error: None
									};

									let result = fnord.await;

									debugln!("descr out {:?}", result);
								}
								
								debug!("Descriptor: ");
								for byte in descriptor_buffer {
									debug!("{:02X} ", byte);
								}
								debugln!("");
							};

							loop {
								{
									let mut globals = globals.borrow_mut();
									globals.grxsts = None;
									if otg_fs_global.gintsts.read().rxflvl().bit() {
										globals.grxsts = Some(otg_fs_global.grxstsp_host().read());
									}
								}

								match unsafe {core::pin::Pin::new_unchecked(&mut async_block)}.poll(&mut dummy_context) {
									core::task::Poll::Ready(_) => { break; }
									core::task::Poll::Pending => { continue; }
								}
							}

							core::mem::drop(async_block);


							// TODO do things https://github.com/libusbhost/libusbhost/blob/master/src/usbh_lld_stm32f4.c#L322

							//otg_fs_host.hcchar0.modify(|_, w| w.chdis().set_bit()); // FIXME seems unneeded

							writeln!(tx, "done").ok();
							
						}

						if val.pocchng().bit() {
							writeln!(tx, "overcurrent").ok();
						}

						if val.pcdet().bit() {
							writeln!(tx, "port connect detected (pcdet)").ok();
						}
				
						otg_fs_host.hprt.modify(|r,w| w.bits(r.bits() & !4)); // do not set PENA???
					}

					// DISCINT
					if otg_fs_global.gintsts.read().discint().bit() {
						otg_fs_global.gintsts.write(|w| w.discint().set_bit());
						writeln!(tx, "disconnect (discint)").ok();
						// TODO notify the application about the disconnect
						break;
					}

					// MMIS
					if otg_fs_global.gintsts.read().mmis().bit() {
						otg_fs_global.gintsts.write(|w| w.mmis().set_bit());
						writeln!(tx, "mode mismatch (mmis)").ok();
					}

					// IPXFR
					if otg_fs_global.gintsts.read().ipxfr_incompisoout().bit(){
						otg_fs_global.gintsts.write(|w| w.ipxfr_incompisoout().set_bit());
						writeln!(tx, "ipxfr").ok();
					}

					// HCINT
					if otg_fs_global.gintsts.read().hcint().bit() {
						trigger_pin.set_low();
						let haint = otg_fs_host.haint.read().bits();
						writeln!(tx, "#{} hcint (haint = {:08x})", frame_number, haint).ok();
						for i in 0..8 {
							if haint & (1 << i) != 0 {
								writeln!(tx, "hcint{} = {:08x}", i, otg_fs_host.hcintx(i).read().bits()).ok();
								otg_fs_host.hcintx(i).write(|w| w.bits(!0));
								writeln!(tx, "hcchar0 chena = {}", otg_fs_host.hcchar0.read().chena().bit()).ok();
							}
						}
					}

					//delay.delay_ms(1_u32);
				}
			}
		}
	}

	loop {}
}
