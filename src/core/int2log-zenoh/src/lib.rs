use int2log_common::*;
use zenoh::handlers::DefaultHandler;
use std::future::Future;

use zenoh::prelude::*;
use zenoh::time::Timestamp;
use zenoh::bytes::ZBytes;
use flume::Receiver;

use tokio::runtime::Runtime;

#[derive(Default, Debug, Clone)]
pub struct ZenohConfiguration {
    pub config: zenoh::Config,
    pub pub_key: Option<String>,
    pub sub_key: Option<String>,
}

#[derive(Default, Debug)]
pub struct ZenohMiddlewareBuilder<'a> {
    config: Option<ZenohConfiguration>,
    session: Option<&'a zenoh::Session>,
    publisher: Option<zenoh::pubsub::Publisher<'a>>,
    subscriber: Option<zenoh::pubsub::Subscriber<'a, flume::Receiver<zenoh::sample::Sample>>>,
}

impl<'a> ZenohMiddlewareBuilder<'a> {
    pub async fn config(mut self, config: ZenohConfiguration) -> Result<Self, zenoh::Error>{
        self.config = Some(config.clone());
        
        let session = zenoh::open(config.config.clone()).await.unwrap();
        let session_ref = Box::leak(Box::new(session));

        if let Some(pub_key) = config.pub_key.clone() {
            let publisher: zenoh::pubsub::Publisher = session_ref.declare_publisher(pub_key).await.unwrap();
            self = self.publisher(publisher);
        }

        if let Some(sub_key) = config.sub_key.clone() {
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
                session,
                self.publisher,
                self.subscriber,
            ).await),
            None => Err("Session is required to build ZenohMiddleware"),
        }
    }
}

#[derive(Debug)]
pub struct ZenohMiddleware<'a> {
    pub config: ZenohConfiguration,
    pub session: &'a zenoh::Session,
    pub publisher: Option<zenoh::pubsub::Publisher<'a>>,
    pub subscriber: Option<zenoh::pubsub::Subscriber<'a, flume::Receiver<zenoh::sample::Sample>>>,
}

impl<'a> ZenohMiddleware<'a> {
    pub async fn new(
        config: Option<ZenohConfiguration>, 
        session: &'a zenoh::Session, 
        publisher: Option<zenoh::pubsub::Publisher<'a>>, 
        subscriber: Option<zenoh::pubsub::Subscriber<'a, flume::Receiver<zenoh::sample::Sample>>>)
         -> Self {
        let config = match config {
            Some(config) => config,
            None => ZenohConfiguration{..Default::default()},
        };
        ZenohMiddleware {
            config,
            session,
            publisher,
            subscriber,
        }
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

use futures::executor::block_on;
impl<'a> Default for ZenohMiddleware<'a> {
    fn default() -> Self {
        let rt = Runtime::new().unwrap();
        // rt.block_on(async {
        // block_on(async{
        //     ZenohMiddlewareBuilder::default().config(ZenohConfiguration{..Default::default()}).await.unwrap().build().await.unwrap()
        // })
        // ZenohMiddlewareBuilder::default().config(ZenohConfiguration{..Default::default()}).await.unwrap().build().await.unwrap()
        tokio::task::block_in_place(|| {
            futures::executor::block_on(async {
                ZenohMiddlewareBuilder::default().config(ZenohConfiguration{..Default::default()}).await.unwrap().build().await.unwrap()
            })
        })
    }
}

#[cfg(test)]
mod tests {
    use zenoh::config::default;

    use super::*;

    // #[tokio::test]
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn it_works() {
        let middleware_default = ZenohMiddlewareBuilder::default().config(ZenohConfiguration{..Default::default()}).await.unwrap().build().await.unwrap();
        // println!("{:?}", middleware_default);

        let zenoh_config = ZenohConfiguration{
            config: default(),
            pub_key: Some(String::from("topic/test")),
            sub_key: Some(String::from("topic/test")),
        };
        let middleware = ZenohMiddlewareBuilder::default().config(zenoh_config).await.unwrap().build().await.unwrap();
        let payload_1 = String::from("Hi! It's me!");
        let payload_2: String = String::from("Hi! It's me!!");
        middleware.sender(payload_1).await;
        middleware.receiver().await;
        middleware.sender(payload_2).await;
        middleware.receiver().await;
    }
}
