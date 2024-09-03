use int2log_common::*;
use zenoh::handlers::DefaultHandler;
use std::future::Future;

use zenoh::prelude::*;
use zenoh::time::Timestamp;
use zenoh::bytes::ZBytes;
use flume::Receiver;

use tokio::runtime::Runtime;

#[derive(Debug, Clone)]
pub struct ZenohConfiguration {
    pub config: zenoh::Config,
    pub pub_key: Option<String>,
    pub sub_key: Option<String>,
}
use std::path::PathBuf;
impl ZenohConfiguration {
    fn load_config_from_file(file_path: &str) -> zenoh::Config {
        let mut config_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        config_path.push(file_path);
        let config = zenoh::Config::from_file(config_path).expect("Failed to load configuration file");
        config
    }
}

impl Default for ZenohConfiguration {
    fn default() -> Self {
        ZenohConfiguration {
            config: ZenohConfiguration::load_config_from_file("zenoh.json5"),
            pub_key: Some(String::from("log")),
            sub_key: None,
            // sub_key: Some(String::from("log")),
        }
    }
}

#[derive(Default, Debug)]
pub struct ZenohMiddlewareBuilder<'a> {
    config: ZenohConfiguration,
    session: Option<&'a zenoh::Session>,
    publisher: Option<zenoh::pubsub::Publisher<'a>>,
    subscriber: Option<zenoh::pubsub::Subscriber<'a, flume::Receiver<zenoh::sample::Sample>>>,
}

impl<'a> ZenohMiddlewareBuilder<'a> {
    pub async fn config(mut self) -> Result<Self, zenoh::Error> {// -> Result<Self, zenoh::Error>{
        // self.config = Some(config.clone());
        
        let session = zenoh::open(self.config.config.clone()).await.unwrap();
        let session_ref = Box::leak(Box::new(session)); // 힙에 메모리를 할당해야 하지만, 프로그램이 끝날 때까지 해당 메모리를 해제할 필요가 없는 상황을 위한 것

        if let Some(pub_key) = self.config.pub_key.clone() {
            let publisher: zenoh::pubsub::Publisher = session_ref.declare_publisher(pub_key).await.unwrap();
            self = self.publisher(publisher);
        }

        if let Some(sub_key) = self.config.sub_key.clone() {
            let subscriber = session_ref.declare_subscriber(sub_key).await.unwrap();
            self = self.subscriber(subscriber);
        }

        self.session = Some(session_ref);
        Ok(self)
    }

    pub fn session(&mut self, session: Option<&'a zenoh::Session>) {
        self.session = session;
    }

    pub fn publisher(self, publisher: zenoh::pubsub::Publisher<'a>) -> ZenohMiddlewareBuilder<'a> {
        ZenohMiddlewareBuilder {
            config: self.config,
            session: self.session,
            publisher: Some(publisher),
            subscriber: self.subscriber,
        }
    }

    pub fn subscriber(self, subscriber: zenoh::pubsub::Subscriber<'a, flume::Receiver<zenoh::sample::Sample>>) -> ZenohMiddlewareBuilder<'a> {
        ZenohMiddlewareBuilder {
            config: self.config,
            session: self.session,
            publisher: self.publisher,
            subscriber: Some(subscriber),
        }
    }

    pub async fn build(self) -> Result<ZenohMiddleware<'a>, &'static str> {
        match self.session {
            Some(session) => Ok(ZenohMiddleware::new(
                self.config,
                self.session,
                self.publisher,
                self.subscriber,
            ).await),
            None => Err("Session is required to build ZenohMiddleware"),
        }
    }
}

#[derive(Debug, Default)]
pub struct ZenohMiddleware<'a> {
    pub config: ZenohConfiguration,
    pub session: Option<&'a zenoh::Session>,
    pub publisher: Option<zenoh::pubsub::Publisher<'a>>,
    pub subscriber: Option<zenoh::pubsub::Subscriber<'a, flume::Receiver<zenoh::sample::Sample>>>,
}

impl<'a> ZenohMiddleware<'a> {
    pub async fn new(
        config: ZenohConfiguration, 
        session: Option<&'a zenoh::Session>,
        publisher: Option<zenoh::pubsub::Publisher<'a>>, 
        subscriber: Option<zenoh::pubsub::Subscriber<'a, flume::Receiver<zenoh::sample::Sample>>>)
         -> Self {
        ZenohMiddleware {
            config,
            session,
            publisher,
            subscriber,
        }
    }

    pub async fn default() -> Self{
        ZenohMiddlewareBuilder::default().config().await.unwrap().build().await.unwrap()
    }
}

impl<'a> Communication<String> for ZenohMiddleware<'a>{
    async fn sender(&self, data: String) {
        if let Some(publisher) = &self.publisher {
            publisher.put(data).await.unwrap();
        }
    }
}

impl<'a> Communication<Vec<u8>> for ZenohMiddleware<'a>{
    async fn sender(&self, data: Vec<u8>) {
    // fn sender(&self, data: Vec<u8>) -> impl Future<Output = ()> {
        if let Some(publisher) = &self.publisher {
            publisher.put(data).await.unwrap();
        }
    }
}

impl<'a> ZenohMiddleware<'a> {
    pub async fn receiver(&self) {
        if let Some(subscriber) = &self.subscriber {
            if let Ok(sample) = subscriber.recv_async().await {
                // return Ok(Some(String::from_utf8_lossy(&sample.value.payload.contiguous()).into_owned()));
                let payload = sample
                    .payload()
                    .deserialize::<String>()
                    .unwrap_or_else(|e| format!("{}", e));

                print!(
                    ">> [Subscriber] Received {} ('{}': '{}')",
                    sample.kind(),
                    sample.key_expr().as_str(),
                    payload
                );
                if let Some(att) = sample.attachment() {
                    let att = att
                        .deserialize::<String>()
                        .unwrap_or_else(|e| format!("{}", e));
                    print!(" ({})", att);
                }
                println!();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use zenoh::config::default;

    use super::*;

    // #[tokio::test]
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn it_works() {
        // let middleware_default = ZenohMiddlewareBuilder::default().config(ZenohConfiguration{..Default::default()}).await.unwrap().build().await.unwrap();
        // // println!("{:?}", middleware_default);

        // let zenoh_config = ZenohConfiguration{
        //     config: default(),
        //     pub_key: Some(String::from("topic/test")),
        //     sub_key: Some(String::from("topic/test")),
        // };
        // let middleware = ZenohMiddlewareBuilder::default().config().await.unwrap().build().await.unwrap();
        let middleware = ZenohMiddleware::default().await;
        let payload_1 = String::from("Hi! It's me!");
        let payload_2: String = String::from("Hi! It's me!!");
        middleware.sender(payload_1).await;
        middleware.receiver().await;
        middleware.sender(payload_2).await;
        middleware.receiver().await;
        // let config = load_config_from_file("zenoh.json5");
        // println!("{:?}", config);
    }
}
