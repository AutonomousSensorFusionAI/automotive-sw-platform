use int2log_core::*;
use std::{cell::RefCell, rc::Rc, thread, time};

fn main() {
    let mut log = Log::default();
    /*
    세부 로거 수정은 다음과 같이.
    let default_console = log.logger.borrow_mut().default_console();
    default_console.borrow_mut().set_log_level("error");
    */
    let one_second = time::Duration::from_secs(1);
    println!("{:?}", log);

    loop {
        log.trace("Hi, Trace!".to_string());
        log.debug("Hi, Debug!".to_string());
        log.info("Hi, Info!".to_string());
        log.warn("Hi, Warn!".to_string());
        log.error("Hi, Error!".to_string());
        thread::sleep(one_second);
    }
}

#[test]
fn custom_logger() {
    let custom_logger = Rc::new(RefCell::new(Logger::new()));
    let console_logger = Rc::new(RefCell::new(ConsoleLogger::default()));
    /*
    console_logger.borrow_mut().set_active_false();
    */
    custom_logger.borrow_mut().attach(console_logger.clone());
    let mut log = Log::new("warn", custom_logger.clone());
    /*
    custom_logger.borrow_mut().detach(console_logger);
    */
    let one_second = time::Duration::from_secs(1);
    println!("{:?}", log);

    loop {
        log.trace("Hi, Trace!".to_string());
        log.debug("Hi, Debug!".to_string());
        log.info("Hi, Info!".to_string());
        log.warn("Hi, Warn!".to_string());
        log.error("Hi, Error!".to_string());
        thread::sleep(one_second);
    }
}
