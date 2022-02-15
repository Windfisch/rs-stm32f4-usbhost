use crate::print::debugln;
use core::future::Future;
use core::pin::Pin;
use core::cell::RefCell;

use core::task::{Poll, Context};

use crate::usb_host::UsbGlobals;

use crate::host_ext::OtgFsHostExt;



pub enum TransactionState {
	WaitingForAvailableChannel,
	WaitingForTransactionToFinish(u8)
}


#[derive(Clone, Copy)]
pub enum DataPid {
	Data0 = 0,
	Data2 = 1,
	Data1 = 2,
	MdataSetup = 3
}


pub struct UsbOutTransaction<'a> {
	data: &'a [u8],
	state: TransactionState,
	globals: &'a RefCell<UsbGlobals>,
	endpoint_type: EndpointType,
	endpoint_number: u8,
	device_address: u8,
	data_pid: DataPid,
	packet_size: u16,
	is_lowspeed: bool,
	last_error: Option<TransactionError>
}

pub struct UsbInTransaction<'a> {
	data: &'a mut [u8],
	state: TransactionState,
	globals: &'a RefCell<UsbGlobals>,
	rx_pointer: usize,
	endpoint_type: EndpointType,
	endpoint_number: u8,
	device_address: u8,
	data_pid: DataPid,
	packet_size: u16,
	is_lowspeed: bool,
	last_error: Option<TransactionError>
}

impl UsbOutTransaction<'_> {
	pub(crate) fn new<'a>(data: &'a [u8], endpoint_type: EndpointType, endpoint_number: u8, device_address: u8, data_pid: DataPid, packet_size: u16, is_lowspeed: bool, globals: &'a RefCell<UsbGlobals>) -> UsbOutTransaction<'a> {
		UsbOutTransaction {
			data,
			globals,
			endpoint_type,
			endpoint_number,
			device_address,
			data_pid,
			packet_size,
			is_lowspeed,
			state: TransactionState::WaitingForAvailableChannel,
			last_error: None
		}
	}
}

impl UsbInTransaction<'_> {
	pub(crate) fn new<'a>(data: &'a mut [u8], endpoint_type: EndpointType, endpoint_number: u8, device_address: u8, data_pid: DataPid, packet_size: u16, is_lowspeed: bool, globals: &'a RefCell<UsbGlobals>) -> UsbInTransaction<'a> {
		UsbInTransaction {
			data,
			globals,
			endpoint_type,
			endpoint_number,
			device_address,
			data_pid,
			packet_size,
			is_lowspeed,
			state: TransactionState::WaitingForAvailableChannel,
			last_error: None,
			rx_pointer: 0
		}
	}
}

#[derive(Copy, Clone, Debug)]
pub enum TransactionError {
	DataToggleError,
	FrameOverrun,
	BabbleError,
	TransactionError,
	Nak,
	Stall,
	Unknown
}

impl Future for UsbInTransaction<'_> {
	type Output = Result<usize, TransactionError>;

	fn poll(mut self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Result<usize, TransactionError>> {
		let globals = self.globals.borrow_mut();
		match self.state {
			TransactionState::WaitingForAvailableChannel => {
				let available_channel = (0..8).find(|i| globals.usb_host.hccharx(*i).read().chena().bit_is_clear());
				if let Some(channel) = available_channel {
					unsafe {
						globals.usb_host.hctsizx(channel).write(|w| w
							.dpid().bits(self.data_pid as u8)
							.pktcnt().bits(div_ceil(self.data.len(), self.packet_size as usize) as u16)
							.xfrsiz().bits(self.data.len() as u32)
						);
						globals.usb_host.hccharx(channel).write(|w| w
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

					self.state = TransactionState::WaitingForTransactionToFinish(channel);
				}
			}
			TransactionState::WaitingForTransactionToFinish(channel) => {
				// FIXME epnum should be chnum! see https://github.com/Windfisch/rs-stm32f4-usbhost/issues/13
				if globals.usb_global.gintsts.read().rxflvl().bit() &&  globals.usb_global.grxstsr_host().read().epnum().bits() == channel {
					let grxsts = globals.usb_global.grxstsp_host().read();
					assert!(grxsts.chnum() == channel);
					let packet_status = PacketStatus::from(grxsts.pktsts().bits());
					//debugln!("#{}: read ch={} dpid={} bcnt={} pktsts={} {:?}", globals.usb_host.hfnum.read().frnum().bits(), grxsts.chnum().bits(), grxsts.dpid().bits(), grxsts.bcnt().bits(), grxsts.pktsts().bits(), packet_status);

					match packet_status {
						PacketStatus::InDataPacketReceived => {
							let len = grxsts.bcnt().bits() as usize;
							debugln!("read {} bytes, pointer {}/{}", len, self.rx_pointer, self.data.len());
							for i in (0..len).step_by(4) {
								let fifo_word = unsafe { core::ptr::read_volatile((0x50001000 + (channel as usize) * 0x1000) as *mut [u8; 4]) };
								let offset = self.rx_pointer + i;
								let remaining = usize::min(len - i, 4);
								self.data[offset..(offset + remaining)].copy_from_slice(&fifo_word[0..remaining]);
							}
							self.rx_pointer += len;
							let packets_remaining = globals.usb_host.hctsizx(channel).read().pktcnt().bits();
							debugln!("{} / {} ({})", self.rx_pointer, self.data.len(), packets_remaining);
							if self.rx_pointer < self.data.len() && packets_remaining > 0 {
								globals.usb_host.hccharx(channel).modify(|_, w| w.chena().set_bit()); // FIXME really?
							}

							
						}
						_ => {}
					}
				}
				let hcint = globals.usb_host.hcintx(channel).read();
				globals.usb_host.hcintx(channel).write(|w| unsafe { w.bits(hcint.bits()) } );

				//debugln!("{:08X}", hcint.bits());
				
				let error =
					if hcint.bberr().bit() { Some(TransactionError::BabbleError) }
					else if hcint.dterr().bit() { Some(TransactionError::DataToggleError) }
					else if hcint.frmor().bit() { Some(TransactionError::FrameOverrun) }
					else if hcint.stall().bit() { Some(TransactionError::Stall) }
					else if hcint.txerr().bit() { Some(TransactionError::TransactionError) }
					else if hcint.nak().bit() { Some(TransactionError::Nak) }
					else { None };

				if error.is_some() {
					//debugln!("Error in IN transaction {:?}, disabling channel", error.unwrap());
					globals.usb_host.hccharx(channel).modify(|_, w| w.chdis().set_bit());

					if self.last_error.is_some() {
						debugln!("WARNING: multiple errors in IN transaction");
					}

					self.last_error = error;
				}

				//FIXME is that really wrong? if usb_host.hccharx(channel).read().chena().bit_is_clear() {
				if hcint.xfrc().bit_is_set() && self.last_error.is_none() {
					return Poll::Ready(Ok(self.rx_pointer));
				}
				else if hcint.xfrc().bit_is_set() || hcint.chh().bit_is_set() {
					return Poll::Ready(Err(self.last_error.unwrap_or(TransactionError::Unknown)));
				}
			}
		}
		return Poll::Pending;
	}
}

#[derive(Copy, Clone)]
pub enum EndpointType {
	Control = 0,
	Isochronous = 1,
	Bulk = 2,
	Interrupt = 3
}

fn div_ceil(a: usize, b: usize) -> usize {
	(a+b-1)/b
}

#[derive(Copy, Clone, Debug)]
pub enum PacketStatus {
	InDataPacketReceived,
	InTransactionCompleted,
	DataToggleError,
	ChannelHalted,
	Unknown
}

impl From<u8> for PacketStatus {
	fn from(val: u8) -> PacketStatus {
		use PacketStatus::*;
		match val {
			2 => InDataPacketReceived,
			3 => InTransactionCompleted,
			5 => DataToggleError,
			7 => ChannelHalted,
			_ => Unknown
		}
	}
}

impl Future for UsbOutTransaction<'_> {
	type Output = Result<(), TransactionError>;

	fn poll(mut self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Self::Output> {
		let globals = self.globals.borrow();
		let usb_host = &globals.usb_host;
		match self.state {
			TransactionState::WaitingForAvailableChannel => {
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
						
						for packet in self.data.chunks(self.packet_size as usize)
						{
							// FIXME ensure the fifo has enough space! move the below into a future!
							globals.usb_global.gintmsk.modify(|_, w| w.nptxfem().set_bit());
							loop {
								let txstatus = globals.usb_global.gnptxsts.read();
								debugln!("queue entries {}\t words {}, top {}", txstatus.nptqxsav().bits(), txstatus.nptxfsav().bits(), txstatus.nptxqtop().bits()); // TODO FIXME debug the tx fifo occupation here
								if txstatus.nptqxsav().bits() > 0 && txstatus.nptqxsav().bits() as usize* 4 >= packet.len() {
									break;
								}
							} // FIXME FIXME FIXME do not block in a future
							globals.usb_global.gintmsk.modify(|_, w| w.nptxfem().clear_bit());

							for chunk in packet.chunks(4) {
								let mut tmp = [0; 4];
								tmp[0..chunk.len()].copy_from_slice(chunk);
								core::ptr::write_volatile((0x50001000 + (channel as usize) * 0x1000) as *mut [u8; 4], tmp);
							}
						}
						//debugln!("gnptxsts = {:08x}, hptxfsiz = {:08x}", otg_fs_global.gnptxsts.read().bits(), otg_fs_global.hptxfsiz.read().bits());
					}

					self.state = TransactionState::WaitingForTransactionToFinish(channel);
				}
			}
			TransactionState::WaitingForTransactionToFinish(channel) => {
				let hcint = usb_host.hcintx(channel).read();
				usb_host.hcintx(channel).write(|w| unsafe { w.bits(hcint.bits()) } );
				
				let error =
					if hcint.bberr().bit() { Some(TransactionError::BabbleError) }
					else if hcint.dterr().bit() { Some(TransactionError::DataToggleError) }
					else if hcint.frmor().bit() { Some(TransactionError::FrameOverrun) }
					else if hcint.stall().bit() { Some(TransactionError::Stall) }
					else if hcint.txerr().bit() { Some(TransactionError::TransactionError) }
					else if hcint.nak().bit() { Some(TransactionError::Nak) }
					else { None };

				if error.is_some() {
					debugln!("Error in OUT transaction {:?}, disabling channel", error.unwrap());
					usb_host.hccharx(channel).modify(|_, w| w.chdis().set_bit());
					
					if self.last_error.is_some() {
						debugln!("WARNING: multiple errors in OUT transaction");
					}

					self.last_error = error;
				}

				//debug!("{}", hcint.xfrc().bit());
				//FIXME is that really wrong? if usb_host.hccharx(channel).read().chena().bit_is_clear() {
				if hcint.xfrc().bit_is_set() && self.last_error.is_none() {
					return Poll::Ready(Ok(()));
				}
				else if hcint.xfrc().bit_is_set() || hcint.chh().bit_is_set() {
					return Poll::Ready(Err(self.last_error.unwrap_or(TransactionError::Unknown)));
				}
			}
		}
		return Poll::Pending;
	}
}


