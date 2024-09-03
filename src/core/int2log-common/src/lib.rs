use std::future::Future;

#[repr(i32)]
#[derive(Debug, Default, PartialEq, PartialOrd, Copy, Clone)]
pub enum LogLevel {
	Trace = 0, 
	#[default]
	Debug = 1, // This is Default Value of Log Level
	Info = 2, 
	Warn = 3,
	Error = 4,
}

impl LogLevel {
	pub fn from_str(level_str: &str) -> Option<Self>{
		match level_str {
			"Trace" => Some(LogLevel::Trace),
			"Debug" => Some(LogLevel::Debug),
			"Info" => Some(LogLevel::Info),
			"Warn" => Some(LogLevel::Warn),
			"Error" => Some(LogLevel::Error),
			_ => None,
		}
	}
}

#[repr(C)]
#[derive(Debug, Default)]
pub struct LogMessage {
	pub log_level: LogLevel,
	pub msg: String,
	pub timestamp: String,
}

pub trait Serialization<T> {
	fn serialize_msg(&self, t: &LogMessage) -> T;
	fn deserialize_msg(&self, t: &T) -> LogMessage;
}

pub trait Communication<T> {
	fn sender(&self, t: T) -> impl Future<Output = ()>; // async fn
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {

    }
}
