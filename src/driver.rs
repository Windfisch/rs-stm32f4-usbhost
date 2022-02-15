use crate::print::debugln;
use core::future::Future;
use core::pin::Pin;
use core::cell::RefCell;
use sharing_coroutines_nostd;


pub trait DriverContext {}

pub trait DriverDescriptor {
	type DriverFutureType : Future<Output = ()>;
	type DriverContextType : DriverContext;

	fn wants_device(&self, device_descriptor: &[u8], config_descriptor: &[u8]) -> bool;
	fn create_instance(&self) -> sharing_coroutines_nostd::FutureContainer<Self::DriverContextType, Self::DriverFutureType>;
}

pub trait DriverInstance {
	fn poll(self: Pin<&Self>);
}

impl <F: Future<Output = ()>, C: DriverContext> DriverInstance for sharing_coroutines_nostd::FutureContainer<C, F> {
	fn poll(self: Pin<&Self>) {
		self.poll()
	}
}

pub struct MidiDriverContext {
	send_queue: RefCell<heapless::Deque<[u8;4], 64>>,
	recv_queue: RefCell<heapless::Deque<[u8;4], 64>>,
}
impl DriverContext for MidiDriverContext {}

pub type MidiDriverFuture<'a> = impl Future<Output = ()> + 'a;



pub struct MidiDriverDescriptor {}

impl DriverDescriptor for MidiDriverDescriptor {
	type DriverFutureType = MidiDriverFuture<'static>;
	type DriverContextType = MidiDriverContext;

	fn wants_device(&self, device_descriptor: &[u8], config_descriptor: &[u8]) -> bool {
		let mut i = 0;
		let mut want = false;
		while i < config_descriptor.len() && i + (config_descriptor[i] as usize) <= config_descriptor.len() {
			let descr = &config_descriptor[i .. (i+config_descriptor[i] as usize)];

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
							want = true;
						}
						0x03 => { // midi out jack
							debugln!("midi out jack #{} ({})", descr[4], descr[3]);
							want = true;
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

			i += config_descriptor[i] as usize;
		}

		return want;
	}
	fn create_instance(&self) -> sharing_coroutines_nostd::FutureContainer<Self::DriverContextType, Self::DriverFutureType> {
		let context = MidiDriverContext {
			send_queue: RefCell::new(heapless::Deque::new()),
			recv_queue: RefCell::new(heapless::Deque::new())
		};

		unsafe { sharing_coroutines_nostd::FutureContainer::new(context, make_midi_driver_future) }
	}
}

impl MidiDriverContext {
	pub fn send(&self, message: [u8; 4]) -> Result<(), [u8; 4]> {
		self.send_queue.borrow_mut().push_back(message)
	}

	pub fn recv(&self) -> Option<[u8;4]> {
		self.recv_queue.borrow_mut().pop_front()
	}
}

fn make_midi_driver_future(data: &MidiDriverContext) -> MidiDriverFuture {
	async fn make(data: &MidiDriverContext) {
		todo!();
	}
	make(data)
}

