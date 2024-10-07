use int2log_zenoh::*;
use futures::FutureExt; // map을 사용하기 위해 필요

#[tokio::main]
async fn main() {
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
