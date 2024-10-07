//! `int2log-zenoh` is the Zenoh middleware library for Int2Log's logging systems.
//! It publishes log messages locally or to the cloud, depending on the user's configuration, using the Zenoh protocol.
//! Zenoh can be configured through the `zenoh.json5` file included in the crate.

//! # Examples
//! ### Publishing Data Using Default Value
//! The example below shows how to produce a value
//! ```
//! use int2log_zenoh::*;
//! use int2log_model::Communication; // this required to use the sender function from Communication trait
//!
//! #[tokio::main]
//! async fn main() {
//!     let middleware = ZenohMiddleware::default().await;
//!     std::thread::sleep(std::time::Duration::from_secs(10)); // Waiting for Opening Session
//!     middleware.sender(String::from("Hi! It's me!")).await;
//! }
//! ```
//!
//! ### Custom
//! The example below shows how to custom ZenohMiddleware.
//! ```
//! use int2log_zenoh::*;
//! use int2log_model::Communication; // this required to use the sender function from Communication trait
//!
//! #[tokio::main]
//! async fn main() {
//!     let zenoh_config = ZenohConfiguration::default().set_subscriber_key("log");
//!     let middleware = ZenohMiddlewareBuilder::default().config(zenoh_config).build().await.unwrap();
//!     std::thread::sleep(std::time::Duration::from_secs(10)); // Waiting for Opening Session
//!     middleware.sender(String::from("Hi! It's me!")).await;
//!     // receiver() is Subscriber's Callback function.
//!     // You will get `[Subscriber] Received PUT ('log': 'Hi! It's me!')`.
//!     middleware.receiver().await;
//! }
//! ```

use int2log_model::serializer::*;
use int2log_model::*;
use std::{future::Future, pin::Pin, sync::Arc, thread};
use tokio::{runtime::Runtime, sync::Mutex};

#[derive(Debug, Clone)]
pub struct ZenohConfiguration {
    /// Reading `zenoh.json5` Zenoh Configuration file in int2log-zenoh dir
    pub config: zenoh::Config,
    /// Default: Some(log)
    pub pub_key: Option<String>,
    /// Default: None
    pub sub_key: Option<String>,
}
use std::path::PathBuf;
impl ZenohConfiguration {
    /// Reading `zenoh.json5` Zenoh Configuration file in int2log-zenoh dir
    fn load_config_from_file(file_path: &str) -> zenoh::Config {
        let mut config_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        config_path.push(file_path);
        let config =
            zenoh::Config::from_file(config_path).expect("Failed to load configuration file");
        config
    }
    /// Setting Publisher key if you want to create publisher
    /// # Example
    /// ```
    /// use int2log_zenoh::*;
    ///
    /// fn main() {
    ///     let zenoh_config = ZenohConfiguration::default().set_publisher_key("custom_log");
    /// }
    /// ```
    pub fn set_publisher_key<T>(mut self, pub_key: T) -> Self
    where
        T: Into<String>,
    {
        self.pub_key = Some(pub_key.into());
        self
    }
    /// Setting Subscriber key if you want to create subscriber
    /// # Example
    /// ```
    /// use int2log_zenoh::*;
    ///
    /// fn main() {
    ///     let zenoh_config = ZenohConfiguration::default().set_subscriber_key("log");
    /// }
    /// ```
    pub fn set_subscriber_key<T>(mut self, sub_key: T) -> Self
    where
        T: Into<String>,
    {
        self.sub_key = Some(sub_key.into());
        self
    }
}

impl Default for ZenohConfiguration {
    fn default() -> Self {
        ZenohConfiguration {
            config: ZenohConfiguration::load_config_from_file("zenoh.json5"),
            pub_key: Some(String::from("log")), // default key log
            sub_key: None, // Log 시스템에서 구독자를 사용할 일이 없을 것 같아 따로 정의하지 X
        }
    }
}

// 빌더 패턴 사용
#[derive(Default, Debug)]
pub struct ZenohMiddlewareBuilder {
    pub config: ZenohConfiguration,
    // 외부 Arc<Mutex<...>>는 스레드간 이동 안전성 보장을 위해 사용됨
    // 내부 Arc는 퍼블리셔 혹은 세션을 Clone 하기 위해 사용(소유권 문제 해결)
    session: Arc<Mutex<Option<Arc<zenoh::Session>>>>,
    publisher: Arc<Mutex<Option<Arc<zenoh::pubsub::Publisher<'static>>>>>,
    subscriber:
        Arc<Mutex<Option<zenoh::pubsub::Subscriber<flume::Receiver<zenoh::sample::Sample>>>>>,
}

impl ZenohMiddlewareBuilder {
    /// You can use this when you want to set your pub/sub key
    /// # Example
    /// ```
    /// use int2log_zenoh::*;
    ///
    /// #[tokio::main]
    /// async fn main() {
    ///     let zenoh_config = ZenohConfiguration{
    ///         config: Default::default(),
    ///         pub_key: Some(String::from("my_key")),
    ///         sub_key: Some(String::from("my_key")),
    ///     };
    ///     let middleware_builder = ZenohMiddlewareBuilder::default().config(zenoh_config).build().await.unwrap();
    /// }
    /// ```
    pub fn config(mut self, config: ZenohConfiguration) -> Self {
        self.config = config;
        self
    }
    /// Build function
    /// # Example
    /// ```
    /// use int2log_zenoh::*;
    ///
    /// #[tokio::main]
    /// async fn main() {
    ///     let middleware = ZenohMiddlewareBuilder::default().build().await.unwrap();
    /// }
    /// ```
    pub async fn build(self) -> Result<ZenohMiddleware, &'static str> {
        let config = self.config.clone();
        let session = self.session.clone();
        let publisher = self.publisher.clone();
        let subscriber = self.subscriber.clone();

        // 백그라운드에서 제노 세션 생성 (생성되지 않아도 타 로거들이 작동하도록(콘솔, 파일 등))
        // tokio::spawn은 런타임을 생성하지 않고도 사용 가능하지만,
        // main 함수에 `.await`가 없어도, async main을 사용해야 함
        thread::spawn(move || {
            let rt = Runtime::new().unwrap();
            rt.block_on(async {
                // .await를 위한 런타임 생성
                loop {
                    // 세션 열기에 실패할 경우 loop를 통해 open 시도함.
                    match zenoh::open(config.config.clone()).await {
                        Ok(new_session) => {
                            let new_session = Arc::new(new_session);

                            let mut session_guard = session.lock().await;
                            *session_guard = Some(new_session.clone());
                            println!("Zenoh session established successfully.");

                            // ZenohConfiguration에 저장된 pub_key가 있을 경우에만 발행자 생성
                            if let Some(pub_key) = config.pub_key.clone() {
                                match session_guard
                                    .as_ref()
                                    .unwrap()
                                    .declare_publisher(pub_key)
                                    .await
                                {
                                    Ok(new_pub) => {
                                        let mut publisher_guard = publisher.lock().await;
                                        *publisher_guard = Some(Arc::new(new_pub));
                                        println!("Publisher created successfully.");
                                    }
                                    Err(e) => println!("Failed to create publisher: {:?}", e),
                                }
                            }

                            // ZenohConfiguration에 저장된 sub_key가 있을 경우에만 구독자 생성
                            if let Some(sub_key) = config.sub_key.clone() {
                                match new_session.declare_subscriber(sub_key).await {
                                    Ok(new_sub) => {
                                        let mut subscriber_guard = subscriber.lock().await;
                                        *subscriber_guard = Some(new_sub);
                                        println!("Subscriber created successfully.");
                                    }
                                    Err(e) => println!("Failed to create subscriber: {:?}", e),
                                }
                            }
                            break;
                        }
                        // Zenoh Session 생성 실패시 에러 발생 -> 5초 후 재시도
                        Err(e) => {
                            println!(
                                "Failed to establish Zenoh session: {:?}. Retrying in 5 seconds...",
                                e
                            );
                            tokio::time::sleep(std::time::Duration::from_secs(5)).await;
                        }
                    }
                }
            })
        });
        Ok(ZenohMiddleware::new(self.config, self.session, self.publisher, self.subscriber).await)
    }
}

#[derive(Debug)]
pub struct ZenohMiddleware {
    pub config: ZenohConfiguration,
    pub session: Arc<Mutex<Option<Arc<zenoh::Session>>>>,
    pub publisher: Arc<Mutex<Option<Arc<zenoh::pubsub::Publisher<'static>>>>>,
    pub subscriber:
        Arc<Mutex<Option<zenoh::pubsub::Subscriber<flume::Receiver<zenoh::sample::Sample>>>>>,
}

impl ZenohMiddleware {
    async fn new(
        config: ZenohConfiguration,
        session: Arc<Mutex<Option<Arc<zenoh::Session>>>>,
        publisher: Arc<Mutex<Option<Arc<zenoh::pubsub::Publisher<'static>>>>>,
        subscriber: Arc<
            Mutex<Option<zenoh::pubsub::Subscriber<flume::Receiver<zenoh::sample::Sample>>>>,
        >,
    ) -> Self {
        ZenohMiddleware {
            config,
            session,
            publisher,
            subscriber,
        }
    }
    /// Default function. This will return the Zenoh session and publisher based on your settings in `zenoh.json5`.
    /// # Example
    /// ```
    /// use int2log_zenoh::*;
    ///
    /// #[tokio::main]
    /// async fn main() {
    ///     let middleware = ZenohMiddleware::default().await;
    /// }
    /// ```
    pub async fn default() -> Self {
        ZenohMiddlewareBuilder::default().build().await.unwrap()
    }
}

impl Communication<String> for ZenohMiddleware {
    fn sender(&self, data: String) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
        Box::pin(async move {
            let publisher_opt = self.publisher.lock().await.as_ref().cloned();
            if let Some(publisher) = publisher_opt {
                publisher.put(data).await.unwrap();
            }
        })
    }
}

impl Communication<Vec<u8>> for ZenohMiddleware {
    // sender의 반환 타입 관련한 내용은 model의 lib.rs를 참고해주세요.
    fn sender(&self, data: Vec<u8>) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
        Box::pin(async move {
            let publisher_opt = self.publisher.lock().await.as_ref().cloned();
            if let Some(publisher) = publisher_opt {
                publisher.put(data).await.unwrap();
            }
        })
    }
}

impl ZenohMiddleware {
    /// Subscriber's callback function
    pub async fn receiver(&self) {
        let subscriber_opt = self.subscriber.lock().await;
        if let Some(subscriber) = &*subscriber_opt {
            if let Ok(sample) = subscriber.recv_async().await {
                let payload = sample
                    .payload()
                    .try_to_string()
                    .unwrap_or_else(|e| e.to_string().into());

                print!(
                    ">> [Subscriber] Received {} ('{}': '{}')",
                    sample.kind(),
                    sample.key_expr().as_str(),
                    payload
                );
                if let Some(att) = sample.attachment() {
                    let att = att.try_to_string().unwrap_or_else(|e| e.to_string().into());
                    print!(" ({})", att);
                }
                println!();
            }
        }
    }
    /// A callback function for the subscriber that includes deserializing the payload using Cap'n Proto.
    pub async fn receiver_capnp(&self) {
        let subscriber_opt = self.subscriber.lock().await;
        let serializer = SerializerFactory::new().capnp_serializer();
        if let Some(subscriber) = &*subscriber_opt {
            if let Ok(sample) = subscriber.recv_async().await {
                let payload = sample.payload().to_bytes().into_owned();
                let payload = serializer.deserialize_msg(&payload);
                print!(
                    ">> [Subscriber] Received {} ('{}': '{:?}')",
                    sample.kind(),
                    sample.key_expr().as_str(),
                    payload
                );
                if let Some(att) = sample.attachment() {
                    let att = att.try_to_string().unwrap_or_else(|e| e.to_string().into());
                    print!(" ({})", att);
                }
                println!();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures::executor::block_on;
    use std::rc::Rc;
    use tokio::task::block_in_place;

    // #[tokio::test]
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn it_works() {
        // let middleware_default = ZenohMiddlewareBuilder::default().config(ZenohConfiguration{..Default::default()}).await.unwrap().build().await.unwrap();
        // // println!("{:?}", middleware_default);

        let zenoh_config = ZenohConfiguration {
            config: Default::default(),
            pub_key: Some(String::from("log")),
            sub_key: Some(String::from("log")),
        };
        let zenoh_config = ZenohConfiguration::default().set_subscriber_key("log");
        let middleware = ZenohMiddlewareBuilder::default()
            .config(zenoh_config)
            .build()
            .await
            .unwrap();
        // let middleware = ZenohMiddlewareBuilder::default().config().await.unwrap().build().await.unwrap();
        // let middleware = ZenohMiddleware::default().await;
        thread::sleep(std::time::Duration::from_secs(10));

        let payload_1 = String::from("Hi! It's me!");
        let payload_2: String = String::from("Hi! It's me!!");
        middleware.sender(payload_1).await;
        middleware.receiver().await;
        middleware.sender(payload_2).await;
        middleware.receiver().await;
    }
}
