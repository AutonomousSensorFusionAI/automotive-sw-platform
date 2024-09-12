use chrono::offset::Utc;
use derivative::Derivative;
use std::{
	fmt::Debug,
	ptr,
	fs::{File, OpenOptions},
	io::Write,
	marker::PhantomData,
	future::Future,
	rc::Rc,
	cell::RefCell,
};
use tokio::{
	runtime::Runtime,
	task::block_in_place,
};
use futures::executor::block_on;

use int2log_model::*;
use int2log_model::log_level::*;
use int2log_model::log_message::*;
use int2log_model::serializer::*;
use int2log_zenoh::*;

pub trait ILogging: Debug {
	fn set_log_level(&mut self, log_level: &str);
	fn set_true(&mut self);
	fn set_false(&mut self);
	fn process(&self, log_message: &LogMessage);
}
pub trait ILogger {
	fn attach(&mut self, logger: Rc<RefCell<dyn ILogging>>);
	fn detach(&mut self, logger: Rc<RefCell<dyn ILogging>>);
	fn process(&self, log_message: &LogMessage);
}

#[derive(Debug)]
pub struct Logger {
	loggers: Vec<Rc<RefCell<dyn ILogging>>>,
}

impl Default for Logger {
	fn default() -> Self {
		let mut logger = Logger::new();
		let console_logger = Rc::new(RefCell::new(ConsoleLogger::default()));
		let file_logger = Rc::new(RefCell::new(FileLogger::default()));
		let middleware_logger = Rc::new(RefCell::new(MiddlewareLogger::default()));
		logger.attach(console_logger.clone());
		logger.attach(file_logger.clone());
		logger.attach(middleware_logger);
		logger
	}
}

impl Logger {
	pub fn new() -> Logger {
		Logger {
			loggers: Vec::new(),
		}
	}
}

impl ILogger for Logger {
	fn attach(&mut self, logger: Rc<RefCell<dyn ILogging>>) {
		self.loggers.push(logger);
	}
	fn detach(&mut self, logger: Rc<RefCell<dyn ILogging>>) {
		self.loggers.retain(|l| !Rc::ptr_eq(l, &logger));
	}
	fn process(&self, log_message: &LogMessage) {
		for item in self.loggers.iter() {
			let logger_ref = item.borrow();
			logger_ref.process(&log_message);
		}
	}
}

#[derive(Derivative)]
#[derivative(Default, Debug)]
pub struct ConsoleLogger {
	pub log_level: LogLevel,
	#[derivative(Default(value="true"))]
	pub activity: bool,
}

#[derive(Derivative)]
#[derivative(Debug)]
pub struct MiddlewareLogger<T> 
{
	pub middleware: Rc<dyn Communication<T>>,
	pub serializer: Rc<dyn Serialization<T>>,
	pub log_level: LogLevel,
	#[derivative(Default(value="true"))]
	pub activity: bool,
	pub _phantom: PhantomData<T>,
}

#[derive(Derivative)]
#[derivative(Default, Debug)]
pub struct FileLogger {
	pub log_level: LogLevel,
	#[derivative(Default(value="FileLogger::default_file_path()"))]
	pub file_path: String,
	#[derivative(Default(value="true"))]
	pub activity: bool,
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
	fn process(&self, log_message: &LogMessage) {
		if self.activity == true {
			if (self.log_level) <= (log_message.log_level) {
				match log_message.log_level { 
					LogLevel::Trace => println!("{} - Trace - {}", log_message.timestamp, &log_message.msg),
					LogLevel::Debug => println!("{} - Debug - {}", log_message.timestamp, &log_message.msg),
					LogLevel::Info => println!("{} - Info - {}", log_message.timestamp, &log_message.msg),
					LogLevel::Warn => println!("{} - Warn - {}", log_message.timestamp, &log_message.msg),
					LogLevel::Error => println!("{} - Error - {}", log_message.timestamp, &log_message.msg),
				}
			}
		}
	}
}

impl ILogging for FileLogger {
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
	fn process(&self, log_message: &LogMessage) {
		let mut file = self.get_log_file();
		if self.activity == true {
			if (self.log_level) <= (log_message.log_level) {
				let log_entry = match log_message.log_level {
					LogLevel::Trace => format!("{} - Trace - {} \n", log_message.timestamp, &log_message.msg),
					LogLevel::Debug => format!("{} - Debug - {} \n", log_message.timestamp, &log_message.msg),
					LogLevel::Info => format!("{} - Info - {} \n", log_message.timestamp, &log_message.msg),
					LogLevel::Warn => format!("{} - Warn - {} \n", log_message.timestamp, &log_message.msg),
					LogLevel::Error => format!("{} - Error - {} \n", log_message.timestamp, &log_message.msg),
				};
				file.write_all(log_entry.as_bytes()).expect("Failed to write log file");
			}
		}
	}
}

impl FileLogger {
	fn default_file_path() -> String {
		"log.txt".to_string()
	}

	pub fn set_file_path(&mut self, file_path: &str) {
		self.file_path = file_path.to_string();
	}

	fn get_log_file(&self) -> File{
		let mut file = OpenOptions::new()
			.write(true)
			.append(true)
			.create(true)
			// .truncate(true) // 파일 초기화
			.open(&self.file_path)
			.unwrap();
		file
	}
}

impl<T> ILogging for MiddlewareLogger<T> 
where
	T: Debug,
{
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
	fn process(&self, log_message: &LogMessage) {
		if self.activity == true {
			if (self.log_level) <= (log_message.log_level) {
				let data: T = self.serializer.serialize_msg(log_message);
				block_in_place(|| {
					block_on(async {
						self.middleware.sender(data).await
					})
				})
			}
		}
	}
}

impl MiddlewareLogger<Vec<u8>> {
	pub fn default() -> Self {
		let middleware = Rc::new(block_in_place(|| {
			block_on(async {
				ZenohMiddlewareBuilder::default().config().await.unwrap().build().await.unwrap()
			})
		}));
		let serializer = SerializerFactory::new().capnp_serializer();
		MiddlewareLogger::new(serializer, middleware)
	}
}

impl<T> MiddlewareLogger<T> {
	pub fn new(serializer: Rc<dyn Serialization<T>>, middleware: Rc<dyn Communication<T>>) -> Self {
		MiddlewareLogger {
			log_level: Default::default(),
			serializer,
			middleware,
			activity: true,
			_phantom: Default::default(),
		}
	}
	pub fn serializer(mut self, serializer: Rc<dyn Serialization<T>>) -> Self {
		MiddlewareLogger {
			log_level: self.log_level,
			serializer,
			middleware: self.middleware,
			activity: self.activity,
			_phantom: Default::default(),
		}
    }
	pub fn middleware(mut self, middleware: Rc<dyn Communication<T>>) -> Self {
		MiddlewareLogger {
			log_level: self.log_level,
			serializer: self.serializer,
			middleware,
			activity: self.activity,
			_phantom: Default::default(),
		}
    }
}

#[repr(C)]
#[derive(Derivative)]
#[derivative(Debug, Default)]
pub struct Log {
	pub log_level: LogLevel,
	pub log_message: LogMessage,
	pub logger: Logger,
}

impl Log {
	pub fn new(logger: Logger) -> Self {
		Log { logger, .. Default::default() }
	}

    pub fn process(&mut self, log_level: LogLevel, msg: String) {
		self.log_message.msg(log_level, msg);
		self.logger.process(&self.log_message);
		self.log_message = LogMessage{..Default::default()}; // Message 초기화
	}

    pub fn trace(&mut self, msg: String) {
		self.process(LogLevel::Trace, msg);
	}

    pub fn debug(&mut self, msg: String) {
		self.process(LogLevel::Debug, msg);
	}

    pub fn info(&mut self, msg: String) {
		self.process(LogLevel::Info, msg);
	}

    pub fn warn(&mut self, msg: String) {
		self.process(LogLevel::Warn, msg);
	}

    pub fn error(&mut self, msg: String) {
		self.process(LogLevel::Error, msg);
	}
}

#[cfg(test)]
mod tests {
    use super::*;

	#[test]
	fn logger_works() {
		let mut logger = Logger::new();
		let console_logger_a = Rc::new(RefCell::new(ConsoleLogger::default()));
		let console_logger_b = Rc::new(RefCell::new(ConsoleLogger {
			activity: false,
			..Default::default()
		}));
		let file_logger = Rc::new(RefCell::new(FileLogger {..Default::default()}));
		console_logger_b.borrow_mut().set_log_level("error");
		let middleware = Rc::new(RefCell::new(MiddlewareLogger::default()));
		logger.attach(console_logger_a.clone());
		logger.attach(console_logger_b.clone());
		logger.attach(file_logger.clone());
		logger.attach(middleware);
		println!("{:?}", logger);

		logger.detach(console_logger_a.clone());
		println!("{:?}", logger);
		logger.attach(console_logger_a.clone());
		console_logger_b.borrow_mut().set_true();
		println!("{:?}", logger);
		
		let log_message = LogMessage::make_msg(LogLevel::Error, "Hi im error".to_string());
		logger.process(&log_message);
	}

	#[test]
	fn log_works() {
		let mut log = Log::default();
		println!("{:?}", log);
		log.error("Hi, Error!".to_string());
	}
}