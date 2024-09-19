use int2log_model::*;
use zenoh::pubsub::Subscriber;
use std::sync::Arc;
use tokio::sync::Mutex;
use std::future::Future;
use std::pin::Pin;
use zenoh::prelude::*;

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
pub struct ZenohMiddlewareBuilder {
    config: ZenohConfiguration,
    session: Arc<Mutex<Option<Arc<zenoh::Session>>>>,
    publisher: Arc<Mutex<Option<Arc<zenoh::pubsub::Publisher<'static>>>>>,
    // subscriber: Option<zenoh::pubsub::Subscriber<flume::Receiver<zenoh::sample::Sample>>>,
    subscriber: Arc<Mutex<Option<zenoh::pubsub::Subscriber<flume::Receiver<zenoh::sample::Sample>>>>>,
}

impl ZenohMiddlewareBuilder {
    pub async fn config(mut self) -> Self{ //-> Result<Self, zenoh::Error> {// -> Result<Self, zenoh::Error>{
        // self.session = Arc::new(Mutex::new(None));
        let config = self.config.clone();
        let session = self.session.clone();
        let publisher = self.publisher.clone();
        let subscriber = self.subscriber.clone();

        tokio::spawn(async move {
            loop {
                match zenoh::open(config.config.clone()).await {
                    Ok(new_session) => {
                        let new_session = Arc::new(new_session);

                        let mut session_guard = session.lock().await;
                        *session_guard = Some(new_session.clone());
                        println!("Zenoh session established successfully.");

                        if let Some(pub_key) = config.pub_key.clone() {   
                            match session_guard.as_ref().unwrap().declare_publisher(pub_key).await {
                                Ok(new_pub) => {
                                    // self = self.publisher(publisher);
                                    let mut publisher_guard = publisher.lock().await;
                                    *publisher_guard = Some(Arc::new(new_pub));
                                    println!("Publisher created successfully.");
                                }
                                Err(e) => println!("Failed to create publisher: {:?}", e),
                            }
                        }

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
                    Err(e) => {
                        println!("Failed to establish Zenoh session: {:?}. Retrying in 5 seconds...", e);
                        tokio::time::sleep(std::time::Duration::from_secs(5)).await;
                    }
                }
            }
        });
        self
    }

    pub async fn build(self) -> Result<ZenohMiddleware, &'static str> {
        Ok(ZenohMiddleware::new(
            self.config,
            self.session,
            self.publisher,
            self.subscriber,
        ).await)
    }
}

#[derive(Debug, Default)]
pub struct ZenohMiddleware {
    pub config: ZenohConfiguration,
    pub session: Arc<Mutex<Option<Arc<zenoh::Session>>>>,
    pub publisher: Arc<Mutex<Option<Arc<zenoh::pubsub::Publisher<'static>>>>>, 
    pub subscriber: Arc<Mutex<Option<zenoh::pubsub::Subscriber<flume::Receiver<zenoh::sample::Sample>>>>>,
}

impl ZenohMiddleware {
    pub async fn new(
        config: ZenohConfiguration, 
        session: Arc<Mutex<Option<Arc<zenoh::Session>>>>,
        publisher: Arc<Mutex<Option<Arc<zenoh::pubsub::Publisher<'static>>>>>, 
        subscriber: Arc<Mutex<Option<zenoh::pubsub::Subscriber<flume::Receiver<zenoh::sample::Sample>>>>>,)
         -> Self {
        ZenohMiddleware {
            config,
            session,
            publisher,
            subscriber,
        }
    }

    pub async fn default() -> Self{
        ZenohMiddlewareBuilder::default().config().await.build().await.unwrap()
    }
}

impl Communication<String> for ZenohMiddleware {
    fn sender(&self, data: String) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
        Box::pin(async move {
            let what = self.session.lock().await.as_ref().cloned();
            match what {
                Some(_) => println!("Some!"),
                None => println!("None"),
            }
            let publisher_opt = self.publisher.lock().await.as_ref().cloned();
            if let Some(publisher) = publisher_opt {
                publisher.put(data).await.unwrap();
            }
        })
    }

    // fn is_open(&self) -> impl Future<Output = bool> + Send {
    //     async move{
    //         let session_opt = self.session.lock().await.as_ref().cloned();
    //         match session_opt {
    //             Some(_) => true,
    //             None => false,
    //         }
    //     }
    // }
}

impl Communication<Vec<u8>> for ZenohMiddleware{
    fn sender(&self, data: Vec<u8>) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
        Box::pin(async move {
            let what = self.session.lock().await.as_ref().cloned();
            match what {
                Some(session) => println!("{:?}", session),
                None => println!("None"),
            }
            let publisher_opt = self.publisher.lock().await.as_ref().cloned();
            if let Some(publisher) = publisher_opt {
                publisher.put(data).await.unwrap();
            }
        })
    }

    // fn is_open(&self) -> impl Future<Output = bool> + Send {
    //     async move{
    //         let session_opt = self.session.lock().await.as_ref().cloned();
    //         match session_opt {
    //             Some(_) => true,
    //             None => false,
    //         }
    //     }
    // }
}

impl ZenohMiddleware {
    pub async fn receiver(&self) {
        let subscriber_opt = self.subscriber.lock().await;
        if let Some(subscriber) = &*subscriber_opt {
            if let Ok(sample) = subscriber.recv_async().await {
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
