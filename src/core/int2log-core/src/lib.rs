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
	cell::RefCell
};
use tokio::runtime::Runtime;

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
	// fn trace(&self, msg: &str);
	// fn debug(&self, msg: &str);
	// fn info(&self, msg: &str);
	// fn warn(&self, msg: &str);
	// fn error(&self, msg: &str);
}

#[derive(Debug)]
struct Logger {
	loggers: Vec<Rc<RefCell<dyn ILogging>>>,
}

impl Logger {
	fn new() -> Logger {
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
		// let logger_ptr = Rc::as_ptr(&logger) as *const dyn ILogging;
		// self.loggers.retain(|l| Rc::as_ptr(l) as *const dyn ILogging != logger_ptr);
	}
	fn process(&self, log_message: &LogMessage) {
		for item in self.loggers.iter() {
			let logger_ref = item.borrow();
			logger_ref.process(&log_message);
		}
	}
	// fn trace(&self, msg: &str) {
	// 	let log_message = LogMessage::make_msg(LogLevel::Trace, msg.to_string());
	// 	for item in self.loggers.iter() {
	// 		let logger_ref = item.borrow();
	// 		logger_ref.process(&log_message);
	// 	}
	// }
	// fn debug(&self, msg: &str) {
	// 	let log_message = LogMessage::make_msg(LogLevel::Debug, msg.to_string());
	// 	for item in self.loggers.iter() {
	// 		let logger_ref = item.borrow();
	// 		logger_ref.process(&log_message);
	// 	}
	// }
	// fn info(&self, msg: &str) {
	// 	let log_message = LogMessage::make_msg(LogLevel::Info, msg.to_string());
	// 	for item in self.loggers.iter() {
	// 		let logger_ref = item.borrow();
	// 		logger_ref.process(&log_message);
	// 	}
	// }
	// fn warn(&self, msg: &str) {
	// 	let log_message = LogMessage::make_msg(LogLevel::Warn, msg.to_string());
	// 	for item in self.loggers.iter() {
	// 		let logger_ref = item.borrow();
	// 		logger_ref.process(&log_message);
	// 	}
	// }
	// fn error(&self, msg: &str) {
	// 	let log_message = LogMessage::make_msg(LogLevel::Error, msg.to_string());
	// 	for item in self.loggers.iter() {
	// 		let logger_ref = item.borrow();
	// 		logger_ref.process(&log_message);
	// 	}
	// }
}

#[derive(Derivative)]
#[derivative(Default, Debug)]
pub struct ConsoleLogger {
	log_level: LogLevel,
	#[derivative(Default(value="true"))]
	activity: bool,
}

#[derive(Derivative)]
#[derivative(Debug)]
pub struct MiddlewareLogger<T, Middleware> 
where
	Middleware: Communication<T> + Debug,
{
	middleware: Middleware,
	log_level: LogLevel,
	log_message: LogMessage,
	#[derivative(Default(value="true"))]
	activity: bool,
	_phantom: PhantomData<T>,
}

#[derive(Derivative)]
#[derivative(Default, Debug)]
pub struct FileLogger {
	log_level: LogLevel,
	#[derivative(Default(value="FileLogger::default_file_path()"))]
	file_path: String,
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

	fn set_file_path(&mut self, file_path: &str) {
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

	#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
	async fn logger_works() {
		let mut logger = Logger::new();
		let console_logger_a = Rc::new(RefCell::new(ConsoleLogger::default()));
		let console_logger_b = Rc::new(RefCell::new(ConsoleLogger {
			activity: false,
			..Default::default()
		}));
		let file_logger = Rc::new(RefCell::new(FileLogger {..Default::default()}));
		console_logger_b.borrow_mut().set_log_level("error");

		logger.attach(console_logger_a.clone());
		logger.attach(console_logger_b.clone());
		logger.attach(file_logger.clone());
		println!("{:?}", logger);

		logger.detach(console_logger_a.clone());
		println!("{:?}", logger);
		logger.attach(console_logger_a.clone());
		console_logger_b.borrow_mut().set_true();
		println!("{:?}", logger);
		// logger.error("hello! I'm error");
	}
}