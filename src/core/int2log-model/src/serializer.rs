use std::rc::Rc;
use derivative::Derivative;
use std::fmt;

use super::*;
use crate::log_message::*;
use crate::log_message_capnp;


pub trait Serialization<T>: fmt::Debug {
	fn serialize_msg(&self, t: &log_message::LogMessage) -> T;
	fn deserialize_msg(&self, t: &T) -> log_message::LogMessage;
}

#[derive(Derivative)]
#[derivative(Debug)]
/// Decorator For Serializer
struct Serializer {}

impl<T> Serialization<T> for Serializer {
    fn serialize_msg(&self, _msg: &LogMessage) -> T {
        unimplemented!("Base serialization not implemented")
    }

    fn deserialize_msg(&self, _data: &T) -> LogMessage {
        unimplemented!("Base deserialization not implemented")
    }
}
// Decorator Pattern
trait Decorator<T>: Serialization<T> {
    fn new(seiralizer: Rc<dyn Serialization<T>>) -> Self;
}

/* 
// - From<T> Trait 이용 → <T>를 Self로 변환가능.
//     - `From<T>`가 구현된 곳에는 자동으로 `Into<U>`도 사용가능
impl From<B> for A {
	fn from(b: B) -> Self {
		match b {
			B... => A...
		}
	}
}

//and you can use like this
let a: A = b.into(); // need ":A"!!
*/
impl From<super::log_message_capnp::log_message::LogLevel> for log_level::LogLevel {
	fn from(capnp_log_level: log_message_capnp::log_message::LogLevel) -> Self {
		match capnp_log_level {
			log_message_capnp::log_message::LogLevel::Trace => log_level::LogLevel::Trace,
			log_message_capnp::log_message::LogLevel::Debug => log_level::LogLevel::Debug,
			log_message_capnp::log_message::LogLevel::Info => log_level::LogLevel::Info,
			log_message_capnp::log_message::LogLevel::Warn => log_level::LogLevel::Warn,
			log_message_capnp::log_message::LogLevel::Error => log_level::LogLevel::Error,
		}
	}
}

impl From<super::log_level::LogLevel> for log_message_capnp::log_message::LogLevel {
	fn from(rust_log_level: log_level::LogLevel) -> Self {
		match rust_log_level {
			log_level::LogLevel::Trace => log_message_capnp::log_message::LogLevel::Trace,
			log_level::LogLevel::Debug => log_message_capnp::log_message::LogLevel::Debug,
			log_level::LogLevel::Info => log_message_capnp::log_message::LogLevel::Info,
			log_level::LogLevel::Warn => log_message_capnp::log_message::LogLevel::Warn,
			log_level::LogLevel::Error => log_message_capnp::log_message::LogLevel::Error,
		}
	}
}

#[derive(Debug)]
pub struct CapnpSerializer<T> {
    serializer: Rc<dyn Serialization<T>>,
}

impl Serialization<Vec<u8>> for CapnpSerializer<Vec<u8>>{
	fn serialize_msg(&self, log_message: &log_message::LogMessage) -> Vec<u8> {
		let mut message: capnp::message::Builder<capnp::message::HeapAllocator> = capnp::message::Builder::new_default();
		let mut log_msg: log_message_capnp::log_message::Builder = message.init_root::<log_message_capnp::log_message::Builder>();
		let rust_log_level: log_level::LogLevel = log_message.log_level.clone();
		let log_level: log_message_capnp::log_message::LogLevel = rust_log_level.into();
		log_msg.set_data(&log_message.data);
		log_msg.set_log_level(log_level);
		log_msg.set_timestamp(&log_message.timestamp);
		log_msg.set_logger(&log_message.logger);
	
		let data: Vec<u8> = capnp::serialize::write_message_to_words(&message);
		data
	}
	fn deserialize_msg(&self, data: &Vec<u8>) -> log_message::LogMessage {
		// Deserializing object
		let reader: capnp::message::Reader<capnp::serialize::OwnedSegments> = capnp::serialize::read_message(
			data.as_slice(),
			capnp::message::ReaderOptions::new()
		).unwrap();
	
		let log_msg: log_message_capnp::log_message::Reader = reader.get_root::<log_message_capnp::log_message::Reader>().unwrap();
		let data: String = log_msg.get_data().unwrap().to_string().unwrap();
		let capnp_log_level: log_message_capnp::log_message::LogLevel = log_msg.get_log_level().unwrap();
		let log_level: log_level::LogLevel = capnp_log_level.into();
		let timestamp: String = log_msg.get_timestamp().unwrap().to_string().unwrap();
		let logger: String = log_msg.get_logger().unwrap().to_string().unwrap();
		log_message::LogMessage {
			log_level,
			data,
			timestamp,
			logger,
		}
	}
}
// Decorator 패턴 사용
impl Decorator<Vec<u8>> for CapnpSerializer<Vec<u8>> {
    fn new(serializer: Rc<dyn Serialization<Vec<u8>>>) -> Self {
        CapnpSerializer { serializer }
    }
}

// Serializer Factory (여러 직렬화 기법을 사용자가 꺼내쓸 수 있도록 하기 위함)
/// Serializer Factory
pub struct SerializerFactory<T> {
	base: Rc<dyn Serialization<T>>,
	capnp_serializer: Rc<CapnpSerializer<T>>,
}

impl SerializerFactory<Vec<u8>> {
	/// Create SerializerFactory
	pub fn new() -> Self {
		let base: Rc<Serializer> = Rc::new(Serializer {});
		let capnp_serializer: Rc<CapnpSerializer<Vec<u8>>> = Rc::new(CapnpSerializer::new(base.clone()));
		SerializerFactory {
			base,
			capnp_serializer,
		}
	}

	/// Get Capn'Proto Serializer
	pub fn capnp_serializer(&self) -> Rc<CapnpSerializer<Vec<u8>>> {
		Rc::clone(&self.capnp_serializer)
	}
}

#[cfg(test)]
mod tests {
    use super::*;
    // #[tokio::test]
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn it_works() {
		// How to define 

		// Case 1
		let base_serializer: Rc<Serializer> = Rc::new(Serializer {});
		let capnp_serializer_dec: CapnpSerializer<Vec<u8>> = CapnpSerializer::new(base_serializer.clone());
		
		// Case 2
		let serializer_factory: SerializerFactory<Vec<u8>> = SerializerFactory::new();
		let capnp_serializer = serializer_factory.capnp_serializer();
		println!("{:?}", capnp_serializer);
	}
}
