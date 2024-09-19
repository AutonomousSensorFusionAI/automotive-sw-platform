use int2log_core::*;
#[tokio::main]
async fn main() {
	let mut log = Log::default();
	println!("{:?}", log);
	
	loop{
		log.error("Hi, Error!".to_string());
		tokio::time::sleep(std::time::Duration::from_secs(1)).await;
	}
}

#[test]
fn log_works() {
	let mut log = Log::default();
	let my_console = log.logger.default_console();
	my_console.borrow_mut().set_log_level("error");
	println!("{:?}", log);
	log.error("Hi, Error!".to_string());
}

#[test]
fn make_logger_ex() {
	use std::{
		rc::Rc,
		cell::RefCell,
	};
	use int2log_model::log_level::*;
	use int2log_model::log_message::*;


	let mut logger = Logger::new();
		let console_logger_a = Rc::new(RefCell::new(ConsoleLogger::default()));
		let console_logger_b = Rc::new(RefCell::new(ConsoleLogger::default()));
		let file_logger = Rc::new(RefCell::new(FileLogger::default()));
		console_logger_b.borrow_mut().set_log_level("error");
		let middleware = Rc::new(RefCell::new(MiddlewareLogger::default()));
		logger.attach(console_logger_a.clone());
		logger.attach(console_logger_b.clone());
		logger.attach(file_logger.clone());
		logger.attach(middleware.clone());
		// let mut log = Log::default();
		let mut log = Log::new("debug", logger);
		println!("{:?}", log);
		log.error("Hi, Error!".to_string());
}