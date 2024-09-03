use chrono::offset::Utc;
use derivative::Derivative;
use std::marker::PhantomData;
use std::future::Future;
use tokio::runtime::Runtime;
use int2log_model::*;
use int2log_zenoh::*;
use int2log_common::*;


#[repr(C)]
#[derive(Derivative)]
#[derivative(Debug, Default)]
pub struct Log<T, Serializer, Middleware> 
where 
	Serializer: Serialization<T>,
	Middleware: Communication<T>,
{
	pub log_message: LogMessage,
	pub publish: bool,
	#[derivative(Default(value = "LogLevel::Error"))]
	pub publish_level: LogLevel,
	pub print: bool,
	pub print_level: LogLevel,
	pub serializer: Serializer,
	pub middleware: Middleware,
	_phantom: PhantomData<T>,
}

#[repr(C)]
#[derive(Debug)]
pub struct DefaultSerializer;
#[repr(C)]
#[derive(Debug)]
pub struct DefaultMiddleware;

impl Default for DefaultSerializer {
    fn default() -> Self {
		DefaultSerializer // DefaultSerializer에 대한 기본 구현
    }
}

impl Default for DefaultMiddleware {
    fn default() -> Self {
		DefaultMiddleware // DefaultMiddleware에 대한 기본 구현
    }
}

impl Serialization<String> for DefaultSerializer {
	fn serialize_msg(&self, log_message: &LogMessage) -> String {
		// Serialization
		let log_level: String = format!("{:?}", &log_message.log_level);
		let msg_data: String = log_message.msg.clone();
		let timestamp: String = log_message.timestamp.clone();
		
		let data: String = format!("{}-{}-{}", log_level, msg_data, timestamp);
		data
	}

	fn deserialize_msg(&self, data: &String) -> LogMessage {
		let first = data.find("-").unwrap();
		let second = data.rfind("-").unwrap();
		let log_level_str = &data[..first];
		let msg_data = &data[first..second];
		let timestamp = &data[second..];
		LogMessage {
			log_level: LogLevel::from_str(log_level_str).unwrap(),
			msg: msg_data.to_string(),
			timestamp: timestamp.to_string(),
		}
	}
}

impl DefaultMiddleware {
	async fn receiver(&self) {
		println!("You need to implement function the receiver function of Middleware.");
	}
}

impl Communication<String> for DefaultMiddleware {
	async fn sender(&self, data: String) {
		println!("You need to implement function the sender function of Middleware. Your Data is {}", &data);
	}
}

impl Communication<Vec<u8>> for DefaultMiddleware {
	async fn sender(&self, data: Vec<u8>) {
		println!("You need to implement function the sender function of Middleware. Your Data is {:?}", &data);
	}
}


impl<'a> Log <Vec<u8>, CapnpSerializer, ZenohMiddleware<'a>>
where 
	CapnpSerializer: Serialization<Vec<u8>> + Default,
	ZenohMiddleware<'a>: Communication<Vec<u8>>,
{
    #[no_mangle]
	pub async fn default() -> Self {
		let middleware = ZenohMiddlewareBuilder::default().config().await.unwrap().build().await.unwrap();
		Log::log().serializer(CapnpSerializer).middleware(middleware)
	}
}

impl<T, Serializer, Middleware> Log<T, Serializer, Middleware>
where
    Serializer: Serialization<T> + Default,
	Middleware: Communication<T> + Default,
{
	pub fn log() -> Self {
		Log { print: true, publish: true, .. Default::default() }
	}

	pub fn serializer(mut self, serializer: Serializer) -> Log<T, Serializer, Middleware> {
		Log {
			log_message: self.log_message,
            publish: self.publish,
            publish_level: self.publish_level,
            print: self.print,
            print_level: self.print_level,
			serializer,
			middleware: self.middleware,
			_phantom: Default::default(),
		}
		// self.serializer = serializer;
		// self
    }

    pub fn middleware(mut self, middleware: Middleware) -> Log<T, Serializer, Middleware> {
		Log {
			log_message: self.log_message,
            publish: self.publish,
            publish_level: self.publish_level,
            print: self.print,
            print_level: self.print_level,
			serializer: self.serializer,
			middleware,
			_phantom: Default::default(),
		}
    }

	pub fn print(&mut self) {
		if self.print == true {
			self.print = false;
		}
		else if self.print == false {
			self.print = true;
		}
	}

	pub fn publish(&mut self) {
		if self.publish == true {
			self.publish = false;
		}
		else if self.publish == false {
			self.publish = true;
		}
	}

	pub fn set_publish_level(&mut self, log_level: LogLevel) {
		self.publish_level = log_level;
	}

	pub fn set_print_level(&mut self, log_level: LogLevel) {
		self.print_level = log_level;
	}

    pub async fn process_log(&mut self) {
		if self.publish == true {
			if (self.publish_level) <= (self.log_message.log_level) {
				let data: T = self.serializer.serialize_msg(&self.log_message);
				self.middleware.sender(data).await;
			}
		}

		if self.print == true {
			if (self.print_level) <= (self.log_message.log_level) {
				match self.log_message.log_level { 
					LogLevel::Trace => println!("[Trace] {}", &self.log_message.msg),
					LogLevel::Debug => println!("[Debug] {}", &self.log_message.msg),
					LogLevel::Info => println!("[Info] {}", &self.log_message.msg),
					LogLevel::Warn => println!("[Warn] {}", &self.log_message.msg),
					LogLevel::Error => println!("[Error] {}", &self.log_message.msg),
				}
			}
		}
		self.log_message = LogMessage{..Default::default()}; // Message 초기화
	}

    fn get_timestamp(&self) -> String {
        let now: chrono::DateTime<Utc> = Utc::now();
        let now_format: String = now.format("%Y/%m/%d %T").to_string();
        now_format
    }

    pub async fn trace(&mut self, message: String) {
		let log_message: LogMessage = LogMessage {
			log_level: LogLevel::Trace,
			msg: message,
			timestamp: self.get_timestamp(),
		};
		self.log_message = log_message;
		self.process_log().await;
	}

    pub async fn debug(&mut self, message: String) {
		let log_message: LogMessage = LogMessage {
			log_level: LogLevel::Debug,
			msg: message,
			timestamp: self.get_timestamp(),
		};
		self.log_message = log_message;
		self.process_log().await;
	}


    pub async fn info(&mut self, message: String) {
		let log_message: LogMessage = LogMessage {
			log_level: LogLevel::Info,
			msg: message,
			timestamp: self.get_timestamp(),
		};
		self.log_message = log_message;
		self.process_log().await;
	}

    pub async fn warn(&mut self, message: String) {
		let log_message: LogMessage = LogMessage {
			log_level: LogLevel::Warn,
			msg: message,
			timestamp: self.get_timestamp(),
		};
		self.log_message = log_message;
		self.process_log().await;
	}

    pub async fn error(&mut self, message: String) {
		let log_message: LogMessage = LogMessage {
			log_level: LogLevel::Error,
			msg: message,
			timestamp: self.get_timestamp(),
		};
		self.log_message = log_message;
		self.process_log().await;
	}
}

#[cfg(test)]
mod tests {
    use super::*;

	#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn it_works() {
		let mut log: Log<Vec<u8>, CapnpSerializer, ZenohMiddleware<'_>> = Log::default().await;
		log.debug(String::from(format!("Default Setting: {:?}", log))).await;
		log.set_publish_level(LogLevel::Warn);
		log.debug(String::from(format!("Change Pub log level: {:?}", log))).await;
		log.set_print_level(LogLevel::Debug);
		log.debug(String::from(format!("Change Print log level: {:?}", log))).await;
		
		log.trace(String::from("This is Trace!")).await;
		log.debug(String::from("This is Debug!")).await;
		log.info(String::from("This is Info!")).await;
		log.warn(String::from("This is Warn!")).await;
		log.error(String::from("This is Error!")).await;
		log.debug(String::from(format!("log: {:?}", log))).await;
    }
}