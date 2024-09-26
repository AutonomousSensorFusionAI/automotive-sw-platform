use std::{thread, time};
use int2log_core::*;


fn main() {
	let mut log = Log::default();
	let one_second = time::Duration::from_secs(1);
	println!("{:?}", log);
	
	loop{
		log.error("Hi, Error!".to_string());
		thread::sleep(one_second);
	}
}

#[test]
fn log_works() {
	let mut log = Log::default();
	let my_console = log.logger.borrow_mut().default_console();
	my_console.borrow_mut().set_log_level("error");
	println!("{:?}", log);
	log.error("Hi, Error!".to_string());
}