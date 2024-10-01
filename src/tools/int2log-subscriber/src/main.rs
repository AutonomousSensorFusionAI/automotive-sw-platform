use int2log_zenoh::*;

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
        middleware.receiver_capnp().await
    }
}
