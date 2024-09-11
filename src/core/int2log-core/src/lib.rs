use chrono::offset::Utc;
use derivative::Derivative;
use std::marker::PhantomData;
use std::future::Future;
use std::rc::Rc;
use tokio::runtime::Runtime;

use int2log_model::*;
use int2log_model::log_level::*;
use int2log_model::log_message::*;
use int2log_model::serializer::*;
use int2log_zenoh::*;

use std::fmt::Debug;
use std::ptr;

pub trait ILogging: Debug {
	fn set_log_level(&mut self, log_level: &str);
	fn set_true(&mut self);
	fn set_false(&mut self);
	fn process(&self, log_level: LogLevel, msg: &str);
}
pub trait ILogger<'a> {
	fn attach(&mut self, logger: &'a dyn ILogging);
	fn detach(&mut self, logger: &'a dyn ILogging);
	fn trace(&self, msg: &str);
	fn debug(&self, msg: &str);
	fn info(&self, msg: &str);
	fn warn(&self, msg: &str);
	fn error(&self, msg: &str);
}

#[derive(Debug)]
struct Logger<'a> {
	loggers: Vec<&'a dyn ILogging>,
}

impl<'a> Logger<'a> {
	fn new() -> Logger<'a> {
		Logger {
			loggers: Vec::new(),
		}
	}
}

impl<'a> ILogger<'a> for Logger<'a> {
	fn attach(&mut self, logger: &'a dyn ILogging) {
		self.loggers.push(logger);
	}
	// fn detach(&mut self, logger: *const dyn ILogging) {
	fn detach(&mut self, logger: &'a dyn ILogging) {
		let logger_ptr = logger as *const dyn ILogging;
		self.loggers.retain(|l| !std::ptr::eq(&**l as *const dyn ILogging, logger_ptr));
	}
	fn trace(&self, msg: &str) {
		for item in self.loggers.iter() {
			item.process(LogLevel::Trace, msg);
		}
	}
	fn debug(&self, msg: &str) {
		for item in self.loggers.iter() {
			item.process(LogLevel::Debug, msg);
		}
	}
	fn info(&self, msg: &str) {
		for item in self.loggers.iter() {
			item.process(LogLevel::Info, msg);
		}
	}
	fn warn(&self, msg: &str) {
		for item in self.loggers.iter() {
			item.process(LogLevel::Warn, msg);
		}
	}
	fn error(&self, msg: &str) {
		for item in self.loggers.iter() {
			item.process(LogLevel::Error, msg);
		}
	}
}

#[derive(Derivative)]
#[derivative(Default, Debug)]
pub struct ConsoleLogger {
	log_level: LogLevel,
	#[derivative(Default(value="true"))]
	activity: bool,
}

#[derive(Derivative)]
pub struct MiddlewareLogger<T, Middleware> 
where
	Middleware: Communication<T>,
{
	middleware: Middleware,
	log_level: LogLevel,
	log_message: LogMessage,
	#[derivative(Default(value="true"))]
	activity: bool,
	_phantom: PhantomData<T>,
}

#[derive(Derivative)]
#[derivative(Default)]
pub struct FileLogger {
	log_level: LogLevel,
	file_name: String,
	#[derivative(Default(value="true"))]
	activity: bool,
}

impl ILogging for ConsoleLogger {
	fn set_log_level(&mut self, log_level: &str) {
		match log_level {
			"trace" => self.log_level = LogLevel::Trace,
			"debug" => self.log_level = LogLevel::Debug,
			"info" => self.log_level = LogLevel::Info,
			"warn" => self.log_level = LogLevel::Warn,
			"error" => self.log_level = LogLevel::Error,
			_ => println!("To choose between 'trace', 'debug', 'info', 'warn', and 'error'"),
		}
	}
	fn set_true(&mut self) {
		self.activity = true;
	}
	fn set_false(&mut self) {
		self.activity = false;
	}
	fn process(&self, msg_level: LogLevel, msg: &str) {
		if self.activity == true {
			if (self.log_level) <= (msg_level) {
				match msg_level { 
					LogLevel::Trace => println!("[Trace] {}", &msg),
					LogLevel::Debug => println!("[Debug] {}", &msg),
					LogLevel::Info => println!("[Info] {}", &msg),
					LogLevel::Warn => println!("[Warn] {}", &msg),
					LogLevel::Error => println!("[Error] {}", &msg),
				}
			}
		}
	}
}


#[repr(C)]
#[derive(Derivative)]
#[derivative(Debug, Default)]
pub struct Log<T, Middleware> 
where 
	Middleware: Communication<T>,
{
	pub log_message: LogMessage,
	pub publish: bool,
	#[derivative(Default(value = "LogLevel::Error"))]
	pub publish_level: LogLevel,
	pub print: bool,
	pub print_level: LogLevel,
	#[derivative(Default(value = "Rc::new(serializer::Serializer {})"))]
	pub serializer: Rc<dyn Serialization<T>>,
	pub middleware: Middleware,
	_phantom: PhantomData<T>,
}


impl<'a> Log <Vec<u8>, ZenohMiddleware<'a>>
where 
	ZenohMiddleware<'a>: Communication<Vec<u8>>,
{
    #[no_mangle]
	pub async fn default() -> Self {
		let middleware = ZenohMiddlewareBuilder::default().config().await.unwrap().build().await.unwrap();
		let serializer = SerializerFactory::new().capnp_serializer();
		Log::log().serializer(serializer).middleware(middleware)
	}
}

impl<T, Middleware> Log<T, Middleware>
where
	T: Default,
	Middleware: Communication<T> + Default,
{
	pub fn log() -> Self {
		Log { print: true, publish: true, .. Default::default() }
	}

	pub fn serializer(mut self, serializer: Rc<dyn Serialization<T>>) -> Log<T, Middleware> {
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

    pub fn middleware(mut self, middleware: Middleware) -> Log<T, Middleware> {
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

    pub async fn trace(&mut self, message: String) {
		self.log_message.msg(LogLevel::Trace,message);
		self.process_log().await;
	}

    pub async fn debug(&mut self, message: String) {
		self.log_message.msg(LogLevel::Debug, message);
		self.process_log().await;
	}


    pub async fn info(&mut self, message: String) {
		self.log_message.msg(LogLevel::Info, message);
		self.process_log().await;
	}

    pub async fn warn(&mut self, message: String) {
		self.log_message.msg(LogLevel::Warn, message);
		self.process_log().await;
	}

    pub async fn error(&mut self, message: String) {
		self.log_message.msg(LogLevel::Error, message);
		self.process_log().await;
	}
}

#[cfg(test)]
mod tests {
    use super::*;
	use std::ptr;

	#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
	async fn logger_works() {
		let mut logger = Logger::new();
		let mut console_logger_a = ConsoleLogger::default();
		let mut console_logger_b = ConsoleLogger {
			activity: false,
			..Default::default()
		};

		logger.attach(&console_logger_a);
		logger.attach(&mut console_logger_b);
		println!("{:?}", logger);

		// Detach using the raw pointer to the logger
		logger.detach(&console_logger_a);
		println!("{:?}", logger);
		logger.attach(&console_logger_a);
		// console_logger_b.set_true();
		println!("{:?}", logger);
		logger.error("hi?");
	}
}