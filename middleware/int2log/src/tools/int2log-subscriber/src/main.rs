use futures::FutureExt;
use int2log_core::*;
use int2log_model::serializer::SerializerFactory;
use int2log_zenoh::*;
use int2log_zenoh::*;
use std::{cell::RefCell, rc::Rc, time::Duration};

#[tokio::main]
async fn main() {
    // Zenoh Setting
    let zenoh_config = ZenohConfiguration {
        config: Default::default(),
        pub_key: Some(String::from("log_sub")),
        sub_key: Some(String::from("log_pub")),
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
    file.borrow_mut().set_file_path("log_subscriber.txt");
    file.borrow_mut().set_log_level("trace");
    file.borrow_mut().set_active_false();

    // Log Setting
    let mut log = Log::new("debug", logger);

    let mut start_flag = false;
    let health_check = Duration::from_secs(10);
    let mut pub_flag = false;
    log.debug("-1".to_string());

    // Publisher 켜질 때까지 waiting (Pingpong으로 확인)
    while !start_flag {
        match tokio::time::timeout(health_check, middleware.receiver_capnp()).await {
            Ok(Some((_, value))) => match value.data.as_str() {
                "-2" if !pub_flag => {
                    log.debug("0".to_string());
                    pub_flag = true;
                }
                "0" if pub_flag => {
                    start_flag = true;
                }
                _ => {}
            },
            Ok(None) => {}
            Err(_) => {
                pub_flag = false;
                println!(
                    "{} seconds passed without Subscriber starting. Retrying...",
                    health_check.as_secs()
                );
                log.debug("-1".to_string());
            }
        }
    }

    println!("Starting...");
    let mut missing: Vec<i32> = Vec::new();
    let mut count = 0;

    while let Some((_, value)) = middleware.receiver_capnp().await {
        count += 1;
        let pub_count: i32 = value.data.parse().unwrap();
        log.trace(format!("pub: {} sub: {}", pub_count, count));

        match pub_count {
            -1 => {
                count -= 1;
                println!("sub_count: {}, missing: {:?}", count, missing);
                break;
            }
            _ if pub_count != count => {
                for i in count..pub_count {
                    missing.push(i);
                }
                count = pub_count;
            }
            _ => {}
        }
    }
}

#[tokio::test]
async fn make_subscriber() {
    let zenoh_config = ZenohConfiguration {
        config: Default::default(),
        pub_key: None,
        sub_key: Some(String::from("log")),
    };
    let middleware = ZenohMiddlewareBuilder::default()
        .config(zenoh_config)
        .build()
        .await
        .unwrap();

    loop {
        let log_message = middleware.receiver_capnp().map(|value| {
            value.map(|(key, value)| {
                println!(">> [Subscriber] Received ('{}': '{:?}')", key, value);
            })
        });
        log_message.await;
    }
}
