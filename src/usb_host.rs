use core::future::Future;
pub(crate) use core::pin::Pin;
use core::cell::RefCell;

use crate::hal::serial;
use core::fmt::Write;

use crate::hal::{prelude::*, stm32};

use core::task::{Poll, Context};

use crate::coroutine;

use crate::driver;
use crate::host_ext::OtgFsHostExt;


use crate::transaction::*;

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


use stm32f4xx_hal::gpio::{Output, PushPull};

pub(crate) struct UsbGlobals {
	pub(crate) grxsts: Option<stm32f4xx_hal::pac::otg_fs_global::grxstsp_host::R>,
	pub(crate) usb_host: stm32f4xx_hal::pac::OTG_FS_HOST,
	pub(crate) usb_global: stm32f4xx_hal::stm32::OTG_FS_GLOBAL,
}

pub struct SleepUntil<'a> {
	globals: &'a RefCell<UsbGlobals>,
	func: fn(&UsbGlobals) -> bool
}

impl Future for SleepUntil<'_> {
	type Output = ();

	fn poll(self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Self::Output> {
		let globals = self.globals.borrow();
		if (self.func)(&*globals) {
			return Poll::Ready(());
		}
		else {
			return Poll::Pending;
		}
	}
}

pub struct UsbHost {
	globals: RefCell<UsbGlobals>,
	dp_pin: gpio::Pin<Alternate<10_u8>, 'A', 12_u8>,
	dm_pin: gpio::Pin<Alternate<10_u8>, 'A', 11_u8>,
	delay: RefCell<stm32f4xx_hal::delay::Delay>,
	trigger_pin: RefCell<stm32f4xx_hal::gpio::Pin<Output<PushPull>, 'A', 0>>
}

impl UsbHost {
	fn sleep_until(&self, func: fn(&UsbGlobals) -> bool) -> impl Future<Output=()> + '_ {
		SleepUntil {
			globals: &self.globals,
			func
		}
	}
	pub async fn control_out_transfer(&self, request: &[u8], data: Option<&mut [u8]>, device_address: u8, packet_size: u16) {
		// setup stage
		loop {
			let fnord = UsbOutTransaction::new(
				request,
				EndpointType::Control,
				0, // endpoint number
				device_address,
				DataPid::MdataSetup,
				packet_size,
				false,
				&self.globals
			);

			let result = fnord.await;

			if result.is_ok() {
				break;
			}
		}
		
		// data stage
		if let Some(_data) = data {
			todo!();
		}

		// status stage
		loop {
			let fnord = UsbInTransaction::new(
				&mut [],
				EndpointType::Control,
				0,
				device_address,
				DataPid::Data1,
				packet_size,
				false,
				&self.globals
			);

			let result = fnord.await;

			if result.is_ok() {
				break;
			}
		}
	}

	pub async fn control_in_transfer(&self, request: &[u8], data: Option<&mut [u8]>, device_address: u8, packet_size: u16) -> usize {
		loop {
			let fnord = UsbOutTransaction::new(
				&request,
				EndpointType::Control,
				0,
				device_address,
				DataPid::MdataSetup,
				packet_size,
				false,
				&self.globals
			);

			let result = fnord.await;

			if result.is_ok() {
				break;
			}
		}

		let size_transferred =
			if let Some(data) = data {
				loop {
					let fnord = UsbInTransaction::new(
						data,
						EndpointType::Control,
						0,
						device_address,
						DataPid::Data1,
						packet_size,
						false,
						&self.globals
					);

					let result = fnord.await;

					if let Ok(size) = result {
						break size;
					}
				}
			}
			else {
				0
			};

		// status stage (always DATA1)
		let fnord = UsbOutTransaction::new(
			&[],
			EndpointType::Control,
			0,
			device_address,
			DataPid::Data1,
			packet_size,
			false,
			&self.globals
		);

		let result = fnord.await;

		return size_transferred;
	}

	pub async fn get_device_descriptor(&self, device_address: u8) -> Result<[u8; 18], ()> {
		let get_descriptor_packet = [
			0x80u8, // device, standard, host to device
			0x06, // get descriptor
			0x00, 0x01, // descriptor 1
			0x00, 0x00, // index
			18, 0, // length
		];
		let mut descriptor_buffer = [0; 18];

		let size_received = self.control_in_transfer(&get_descriptor_packet, Some(&mut descriptor_buffer), device_address, 64).await;

		if size_received < 8 {
			return Err(());
		}
		else if size_received < descriptor_buffer.len() {
			let ep0_max_size = descriptor_buffer[7].into();
			let size_received2 = self.control_in_transfer(&get_descriptor_packet, Some(&mut descriptor_buffer), device_address, ep0_max_size).await;

			if size_received2 != descriptor_buffer.len() {
				return Err(());
			}
		}
		
		return Ok(descriptor_buffer);
	}

	pub async fn get_configuration_descriptor(&self, device_address: u8, buffer: &mut [u8], max_packet_size: u16) -> Result<usize, GetDescriptorError> {
		assert! (buffer.len() >= 9);

		let mut packet = [
			0x80u8, // device, standard, host to device
			0x06, // get descriptor
			0x00, 0x02,
			0x00, 0x00, // index
			9, 0, // only the first descriptor with the total length field
		];

		let size_received = self.control_in_transfer(&packet, Some(&mut buffer[0..9]), device_address, max_packet_size).await;

		if size_received < 9 {
			return Err(GetDescriptorError::DeviceError);
		}

		let total_length = buffer[2] as u16 + ((buffer[3] as u16) << 8);

		if total_length as usize > buffer.len() {
			return Err(GetDescriptorError::BufferTooSmall(total_length.into()));
		}

		packet[6] = buffer[2];
		packet[7] = buffer[3];

		let size_received = self.control_in_transfer(&packet, Some(&mut buffer[0..total_length.into()]), device_address, max_packet_size).await;

		if size_received != total_length.into() {
			return Err(GetDescriptorError::DeviceError);
		}
		
		return Ok(total_length.into());
	}

	pub async fn set_configuration(&self, device_address: u8, configuration_index: u8) {
		let packet = [
			0x00u8, // device, standard, device to host
			0x09,
			configuration_index, 0x00,
			0x00, 0x00, // index
			0, 0,
		];

		self.control_out_transfer(&packet, None, device_address, 8).await;
	}
}


pub type UsbHostCoroutine<'a> = impl Future<Output=()>;


use stm32f4xx_hal::gpio;
use stm32f4xx_hal::gpio::Alternate;

type CoroutineType<'a> = impl Future<Output=()>;
fn handle_device(host: &UsbHost) -> CoroutineType {
	async fn foo(host: &UsbHost) {
		// ACTUAL DEVICE COMMUNICATION
		//debugln!("addr in {:?}", result);
		let setup_packet = [
			0x00u8, // device, standard, host to device
			0x05, // set address
			0x01, 0x00, // address 1
			0x00, 0x00, // index
			0x00, 0x00, // length
		];
		//host.trigger_pin.borrow().set_high();
		host.control_out_transfer(&setup_packet, None, 0, 64).await;
		//host.trigger_pin.borrow().set_low();

		//host.trigger_pin.borrow().set_high();
		let mut ep0_max_size = 64;
		if let Ok(descriptor) = host.get_device_descriptor(1).await {
			//host.trigger_pin.borrow().set_low();
			ep0_max_size = descriptor[7].into();
			debugln!("Descriptor: {:02X?}; max packet size on ep0 is {}", descriptor, ep0_max_size);
		}
		else {
			debug!("Descriptor: ERROR");
		}


		let mut blab = [0; 512];
		let result = host.get_configuration_descriptor(1, &mut blab, ep0_max_size).await;
		if let Ok(size) = result {
			debugln!("Config descriptor: {:02X?}", &blab[0..size]);

			let configuration_value = blab[5];

			host.trigger_pin.borrow_mut().set_high();
			host.set_configuration(1, configuration_value).await;

			parse_midi_config_descriptor(&blab[0..size]);
		}
		else {
			debugln!("Config descriptor: error {:?}", result);
		}

		let mut data_pid = DataPid::Data0;
		loop {
			let mut data = [0; 64];
			let fnord = UsbInTransaction::new(
				&mut data,
				EndpointType::Bulk,
				1,
				1,
				data_pid,
				64,
				false,
				&host.globals
			);

			let result = fnord.await;

			if let Ok(size) = result {
				debugln!("received: {:02X?}", &data[0..size]);
				data_pid = match data_pid {
					DataPid::Data0 => DataPid::Data1,
					DataPid::Data1 => DataPid::Data0,
					_ => unreachable!()
				};
			}
		}
	}
	foo(host)
}

impl UsbHost {
	fn globals(&self) -> core::cell::RefMut<UsbGlobals> {
		self.globals.borrow_mut()
	}

	pub fn make_coroutine<'a>(&'a self) -> UsbHostCoroutine<'a> {
		async fn foo(host: &UsbHost) {
			unsafe {
				let globals = host.globals.borrow();
				let otg_fs_global = &globals.usb_global;
				let otg_fs_host = &globals.usb_host;

				/*
				// soft reset
				debugln!("resetting");
				otg_fs_global.grstctl.modify(|_,w| w.csrst().set_bit());
				while otg_fs_global.grstctl.read().csrst().bit() { yield None; }
				debugln!("reset done");
				delay.delay_ms(1_u32); // wait at least 3 PHY clocks

				// wait for AHB idle (docs for CSRST require this)
				debugln!("waiting for AHBIDL");
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
			}

			let mut coroutine: Option<CoroutineType> = None;
			
			unsafe {
				loop {
					// "Host initialization" steps 3-4: enable PPWR and wait for PCDET
					debugln!("setting PPWR and waiting for PCDET");
					host.globals.borrow().usb_host.hprt.modify(|_,w| w.ppwr().set_bit()); // FIXME unsure if that's needed
					host.sleep_until(|g| g.usb_host.hprt.read().pcdet().bit()).await;
					debugln!("got PCDET!");
					
					// "Host initialization" steps 5-9
					debugln!("resetting the port and waiting for PENCHNG");
					host.globals.borrow().usb_host.hprt.modify(|_,w| w.prst().set_bit());
					host.delay.borrow_mut().delay_ms(15_u32);
					host.globals.borrow().usb_host.hprt.modify(|_,w| w.prst().clear_bit());
					host.delay.borrow_mut().delay_ms(15_u32);

					host.sleep_until(|g| g.usb_host.hprt.read().penchng().bit()).await;
					debugln!("enumerated speed is {}", host.globals.borrow().usb_host.hprt.read().pspd().bits());
					
					// "Host initialization" step 10: program hfir
					// TODO: is that needed? seems to autoselect a good value

					// "Host initialization" step 11: set PHY clock
					// TODO: is that needed? we already set 1 = 48MHz here. reset port if changed.
					//otg_fs_host.hcfg.modify(|_,w| w.fslspcs().bits(1)); // 48MHz

					// "Host initialization" steps 12-14
					host.globals.borrow().usb_global.grxfsiz.modify(|_,w| w
						.rxfd().bits(64) // 64 32bit words RX fifo size
					);
					host.globals.borrow().usb_global.hnptxfsiz_mut().modify(|_,w| w
						.nptxfd().bits(64)
						.nptxfsa().bits(64)
					);
					host.globals.borrow().usb_global.hptxfsiz.modify(|_,w| w
						.ptxfsiz().bits(64)
						.ptxsa().bits(64+64)
					);
					host.globals.borrow().usb_global.gintmsk.modify(|_,w| w
						.hcim().set_bit()
						.nptxfem().set_bit()
					);

					// "host programming model"/"channel initialization"



					host.delay.borrow_mut().delay_ms(250_u32);

					host.globals().usb_global.gintmsk.modify(|_,w| w.hcim().set_bit()); // FIXME delete
					host.globals().usb_global.gintsts.modify(|_,w| w.bits(!0));


					debugln!("gintsts = {:08x}", host.globals().usb_global.gintsts.read().bits());
					debugln!("hprt = {:08x}", host.globals().usb_host.hprt.read().bits());
					host.delay.borrow_mut().delay_ms(300_u32);
					debugln!("hprt = {:08x}", host.globals().usb_host.hprt.read().bits());
					let mut sofcount: u32 = 0;
					// TODO: notify client that we have a connection

					loop {
						coroutine::fyield().await;
						host.sleep_until(|g| g.usb_global.gintsts.read().bits() != 0).await;

						if let Some(ref mut future) = coroutine {
							coroutine::poll( unsafe { Pin::new_unchecked(future) } );
						}


						if host.globals().usb_global.gintsts.read().sof().bit() {
							host.globals().usb_global.gintsts.write(|w| w.sof().set_bit());
							//write!(tx, "{:8} {:08x}\r", sofcount, host.globals().usb_host.hcchar0.read().bits());
							let hfnum = host.globals().usb_host.hfnum.read().bits();
							let gnptxsts = host.globals().usb_global.gnptxsts.read().bits();
							//debug!("{:8} {:08x} {:08x}\r", sofcount, hfnum, gnptxsts);
							//debugln!("hprt = {:08x}", host.globals().usb_host.hprt.read().bits());
							//write!(tx, "{:08x}\r", host.globals().usb_global.gintsts.read().bits());
							sofcount += 1;
						}

						let frame_number = host.globals().usb_host.hfnum.read().frnum().bits();

						while host.globals().usb_global.gintsts.read().srqint().bit() {
							debugln!("srqint");
							host.globals().usb_global.gintsts.write(|w| w.srqint().set_bit());
							// TODO do things
						}

						// OTGINT
						if host.globals().usb_global.gintsts.read().otgint().bit() {
							let val = host.globals().usb_global.gotgint.read().bits();
							debugln!("otg {:08x}", val);
							host.globals().usb_global.gotgint.modify(|_,w| w.bits(val));
						}

						// HPRTINT
						if host.globals().usb_global.gintsts.read().hprtint().bit() {
							let val = host.globals().usb_host.hprt.read();
							debugln!("hprt {:08x}", val.bits());

							if val.penchng().bit() {
								debugln!("#{}, penchng, port enabled is {}", frame_number, val.pena().bit());

								if val.pena().bit() {
									coroutine = Some(handle_device(host));
								}
								else {
									coroutine = None; // TODO clean termination?
								}

								for i in 0..8 {
									host.globals().usb_host.hcintx(i).write(|w| w.bits(!0));
									host.globals().usb_host.hcintmskx(i).write(|w| w
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
								
								host.globals().usb_host.haintmsk.write(|w| w.bits(0xFFFF));
							}
								

							if val.pocchng().bit() {
								debugln!("overcurrent");
							}

							if val.pcdet().bit() {
								debugln!("port connect detected (pcdet)");
							}
					
							host.globals().usb_host.hprt.modify(|r,w| w.bits(r.bits() & !4)); // do not set PENA???
						}

						// DISCINT
						if host.globals().usb_global.gintsts.read().discint().bit() {
							host.globals().usb_global.gintsts.write(|w| w.discint().set_bit());
							debugln!("disconnect (discint)");
							// TODO notify the application about the disconnect
							break;
						}

						// MMIS
						if host.globals().usb_global.gintsts.read().mmis().bit() {
							host.globals().usb_global.gintsts.write(|w| w.mmis().set_bit());
							debugln!("mode mismatch (mmis)");
						}

						// IPXFR
						if host.globals().usb_global.gintsts.read().ipxfr_incompisoout().bit(){
							host.globals().usb_global.gintsts.write(|w| w.ipxfr_incompisoout().set_bit());
							debugln!("ipxfr");
						}

						// checking for HCINT and RXFLVL is handled by the coroutine(s)
					}
				}
			}
		}
		foo(self)
	}

	pub fn new(
		otg_fs_global: stm32f4xx_hal::stm32::OTG_FS_GLOBAL,
		otg_fs_host: stm32f4xx_hal::stm32::OTG_FS_HOST,
		dp_pin: gpio::Pin<Alternate<10_u8>, 'A', 12_u8>,
		dm_pin: gpio::Pin<Alternate<10_u8>, 'A', 11_u8>,
		delay: stm32f4xx_hal::delay::Delay,
		trigger_pin: stm32f4xx_hal::gpio::Pin<Output<PushPull>, 'A', 0>
	) -> UsbHost {
		let globals = core::cell::RefCell::new(UsbGlobals {
			grxsts: None,
			usb_host: otg_fs_host,
			usb_global: otg_fs_global
		});

		// enable OTG clock
		unsafe {
			let rcc = &(*crate::stm32::RCC::ptr());
			rcc.ahb2enr.modify(|_,w| { w.otgfsen().set_bit() } );
		}

		UsbHost { globals, dp_pin, dm_pin, delay: RefCell::new(delay), trigger_pin: RefCell::new(trigger_pin) }
	}

	pub fn release(self) -> (
		stm32f4xx_hal::stm32::OTG_FS_GLOBAL,
		stm32f4xx_hal::stm32::OTG_FS_HOST,
		gpio::Pin<Alternate<10_u8>, 'A', 12_u8>,
		gpio::Pin<Alternate<10_u8>, 'A', 11_u8>,
		stm32f4xx_hal::delay::Delay,
		stm32f4xx_hal::gpio::Pin<Output<PushPull>, 'A', 0>
	)
	{
		// disable OTG clock
		unsafe {
			let rcc = &(*crate::stm32::RCC::ptr());
			rcc.ahb2enr.modify(|_,w| { w.otgfsen().clear_bit() } );
		}
		let globals = self.globals.into_inner();
		(globals.usb_global, globals.usb_host, self.dp_pin, self.dm_pin, self.delay.into_inner(), self.trigger_pin.into_inner())
	}

	// FIXME actually figure out whether the drivers hold a reference to self
	pub fn poll(&self, host_coroutine: Pin<&mut UsbHostCoroutine>, drivers: &[Pin<&dyn driver::DriverInstance>]) {
		{
			let mut globals = self.globals.borrow_mut();
			globals.grxsts = None;
			if globals.usb_global.gintsts.read().rxflvl().bit() {
				globals.grxsts = Some(globals.usb_global.grxstsp_host().read());
			}
		}

		coroutine::poll(host_coroutine);
		for driver in drivers {
			driver.as_ref().poll();
		}
	}
}

#[derive(Debug)]
pub enum GetDescriptorError {
	DeviceError,
	BufferTooSmall(usize)
}


fn parse_midi_config_descriptor(data: &[u8]) {
	let mut i = 0;
	while i < data.len() && i + (data[i] as usize) <= data.len() {
		let descr = &data[i .. (i+data[i] as usize)];

		match descr[1] {
			0x02 => {
				debugln!("\n======== Configuration #{} with {} interfaces ========\n", descr[5], descr[4]);
			}
			0x04 => {
				debugln!("\n==== Interface #{} ====", descr[2]);
			}
			0x24 => {
				match descr[2] {
					0x01 => {
						debugln!("CS_INTERFACE");
					}
					0x02 => { // midi in jack
						debugln!("midi in jack #{} ({})", descr[4], descr[3]);
					}
					0x03 => { // midi out jack
						debugln!("midi out jack #{} ({})", descr[4], descr[3]);
					}
					0x04 => { debugln!("element"); }
					_ => { debugln!("??"); }
				}
			}
			0x05 => {
				debugln!("endpoint #{:02X}", descr[2]);
			}
			0x25 => {
				debugln!("  endpoint has {} embedded midi jacks", descr[3]);
				for i in 0..(descr[3] as usize) {
					debugln!("  {} -> jack {}", i, descr[4+i]);
				}
			}
			_ => { debugln!("?"); }

		}

		i += data[i] as usize;
	}
}
