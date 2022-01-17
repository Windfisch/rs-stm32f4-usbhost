use core::future::Future;
use core::pin::Pin;
use crate::coroutine;
use core::cell::RefCell;
use crate::usb_host::UsbHost;
use sharing_coroutines_nostd;

pub trait DriverContext {}

pub trait DriverDescriptor {
	type DriverFutureType : Future<Output = ()>;
	type DriverContextType : DriverContext;

	fn wants_device(&self) -> bool; // TODO
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

	fn wants_device(&self) -> bool { todo!(); }
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

