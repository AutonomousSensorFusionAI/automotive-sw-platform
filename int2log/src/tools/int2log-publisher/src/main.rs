use int2log_core::*;
use int2log_model::serializer::SerializerFactory;
use int2log_zenoh::*;
use std::{
    cell::RefCell,
    rc::Rc,
    thread,
    time::{Duration, Instant},
};

#[tokio::main]
async fn main() {
    // Zenoh Setting
    let zenoh_config = ZenohConfiguration {
        config: Default::default(),
        pub_key: Some(String::from("log_pub")),
        sub_key: Some(String::from("log_sub")),
    };
    let middleware = Rc::new(
        ZenohMiddlewareBuilder::default()
            .config(zenoh_config)
            .build()
            .await
            .unwrap(),
    );

    let capnp = SerializerFactory::new().capnp_serializer();
    let mut mid_logger = Rc::new(RefCell::new(MiddlewareLogger::new(
        capnp,
        middleware.clone(),
    )));

    // Logger Setting
    let mut logger = Rc::new(RefCell::new(Logger::new()));
    logger.borrow_mut().attach(mid_logger);

    let file = Rc::new(RefCell::new(FileLogger::default()));
    logger.borrow_mut().attach(file.clone());
    file.borrow_mut().set_file_path("log_publisher.txt");
    file.borrow_mut().set_log_level("trace");
    file.borrow_mut().set_active_false();

    // Log Setting
    let mut log = Log::new("debug", logger);

    let hz_duration = get_duration(500);

    let mut start_flag = false;
    let health_check = Duration::from_secs(10);
    let mut sub_flag = false;

    // Subscriber 켜질 때까지 waiting (Pingpong으로 확인)
    while !start_flag {
        match tokio::time::timeout(health_check, middleware.receiver_capnp()).await {
            Ok(Some((_, value))) => match value.data.as_str() {
                "-1" if !sub_flag => {
                    log.debug("-2".to_string());
                    sub_flag = true;
                }
                "0" if sub_flag => {
                    log.debug("0".to_string());
                    start_flag = true;
                }
                _ => {}
            },
            Ok(None) => {}
            Err(_) => {
                sub_flag = false;
                println!(
                    "{} seconds passed without Subscriber starting. Retrying...",
                    health_check.as_secs()
                );
            }
        }
    }

    println!(
        "Starting... Setting hz: {:?}",
        1000 / hz_duration.as_millis()
    );
    thread::sleep(health_check);

    let publish_duration = Duration::from_secs(5 * 60);
    let mut count = 0;
    let start_time = Instant::now();

    while start_time.elapsed() < publish_duration {
        count += 1;
        log.debug(count.to_string());
        thread::sleep(hz_duration);
    }

    log.debug("-1".to_string());
    println!("Published {} messages", count);
}

fn get_duration(hz: u64) -> Duration {
    let millis: u64 = 1000 / hz;
    Duration::from_millis(millis)
}

#[test]
fn make_log() {
    let mut log = Log::default();
    /*
    세부 로거 수정은 다음과 같이.
    let default_console = log.logger.borrow_mut().default_console();
    default_console.borrow_mut().set_log_level("error");
    */
    let one_second = Duration::from_secs(1);
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
    let one_second = Duration::from_secs(1);
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
