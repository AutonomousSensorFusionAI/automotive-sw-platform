use int2log_common::*;
use capnp::message::Builder;
use capnp::serialize;
pub mod log_message_capnp;
use capnp::message::ReaderOptions;

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
impl From<log_message_capnp::log_message::LogLevel> for LogLevel {
    fn from(capnp_log_level: log_message_capnp::log_message::LogLevel) -> Self {
        match capnp_log_level {
            log_message_capnp::log_message::LogLevel::Trace => LogLevel::Trace,
            log_message_capnp::log_message::LogLevel::Debug => LogLevel::Debug,
            log_message_capnp::log_message::LogLevel::Info => LogLevel::Info,
            log_message_capnp::log_message::LogLevel::Warn => LogLevel::Warn,
            log_message_capnp::log_message::LogLevel::Error => LogLevel::Error,
        }
    }
}

impl From<LogLevel> for log_message_capnp::log_message::LogLevel {
    fn from(rust_log_level: LogLevel) -> Self {
        match rust_log_level {
            LogLevel::Trace => log_message_capnp::log_message::LogLevel::Trace,
            LogLevel::Debug => log_message_capnp::log_message::LogLevel::Debug,
            LogLevel::Info => log_message_capnp::log_message::LogLevel::Info,
            LogLevel::Warn => log_message_capnp::log_message::LogLevel::Warn,
            LogLevel::Error => log_message_capnp::log_message::LogLevel::Error,
        }
    }
}

#[derive(Debug)]
pub struct CapnpSerializer;

impl Default for CapnpSerializer {
    fn default() -> Self {
		CapnpSerializer // DefaultSerializer에 대한 기본 구현
    }
}

impl Serialization<Vec<u8>> for CapnpSerializer{
	fn serialize_msg(&self, log_message: &LogMessage) -> Vec<u8> {
		let mut message: Builder<capnp::message::HeapAllocator> = Builder::new_default();
		let mut log_msg: log_message_capnp::log_message::Builder = message.init_root::<log_message_capnp::log_message::Builder>();
		let rust_log_level: LogLevel = log_message.log_level.clone();
		let log_level: log_message_capnp::log_message::LogLevel = rust_log_level.into();
		log_msg.set_msg(&log_message.msg);
		log_msg.set_log_level(log_level);
		log_msg.set_timestamp(&log_message.timestamp);
	
		let data: Vec<u8> = serialize::write_message_to_words(&message);
		data
	}
	fn deserialize_msg(&self, data: &Vec<u8>) -> LogMessage {
		// Deserializing object
		let reader: capnp::message::Reader<serialize::OwnedSegments> = serialize::read_message(
			data.as_slice(),
			ReaderOptions::new()
		).unwrap();
	
		let log_msg: log_message_capnp::log_message::Reader = reader.get_root::<log_message_capnp::log_message::Reader>().unwrap();
		let msg: String = log_msg.get_msg().unwrap().to_string().unwrap();
		let capnp_log_level: log_message_capnp::log_message::LogLevel = log_msg.get_log_level().unwrap();
		let log_level: LogLevel = capnp_log_level.into();
		let timestamp: String = log_msg.get_timestamp().unwrap().to_string().unwrap();
		LogMessage {
			log_level,
			msg,
			timestamp,
		}
	}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn it_works() {
    }
}