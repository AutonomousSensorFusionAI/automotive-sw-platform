use int2log_core::*;
fn main() {
	let mut log = Log::default();
	println!("{:?}", log);
	log.error("Hi, Error!".to_string());
}


fn make_logger_ex() {
	use std::{
		rc::Rc,
		cell::RefCell,
	};
	use int2log_model::log_level::*;
	use int2log_model::log_message::*;


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