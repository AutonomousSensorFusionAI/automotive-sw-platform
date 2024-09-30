use int2log_model::*;
use std::{sync::Arc, future::Future, pin::Pin, thread};
use tokio::{sync::Mutex, runtime::Runtime};


#[derive(Debug, Clone)]
pub struct ZenohConfiguration {
    pub config: zenoh::Config,
    pub pub_key: Option<String>,
    pub sub_key: Option<String>,
}
use std::path::PathBuf;
impl ZenohConfiguration {
    /// zenoh.json5 config를 읽어오기 위한 함수
    fn load_config_from_file(file_path: &str) -> zenoh::Config {
        let mut config_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        config_path.push(file_path);
        let config = zenoh::Config::from_file(config_path).expect("Failed to load configuration file");
        config
    }

    pub fn set_subscriber_key(mut self, sub_key: String) -> Self{
        self.sub_key = Some(sub_key);
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
    config: ZenohConfiguration,
    // 외부 Arc<Mutex<...>>는 스레드간 이동 안전성 보장을 위해 사용됨
    // 내부 Arc는 퍼블리셔 혹은 세션을 Clone 하기 위해 사용(소유권 문제 해결)
    session: Arc<Mutex<Option<Arc<zenoh::Session>>>>,
    publisher: Arc<Mutex<Option<Arc<zenoh::pubsub::Publisher<'static>>>>>,
    subscriber: Arc<Mutex<Option<zenoh::pubsub::Subscriber<flume::Receiver<zenoh::sample::Sample>>>>>,
}

impl ZenohMiddlewareBuilder {
    pub async fn config(mut self) -> Self{
        let config = self.config.clone();
        let session = self.session.clone();
        let publisher = self.publisher.clone();
        let subscriber = self.subscriber.clone();

        // 백그라운드에서 제노 세션 생성 (생성되지 않아도 타 로거들이 작동하도록(콘솔, 파일 등))
        // tokio::spawn은 런타임을 생성하지 않고도 사용 가능하지만,
        // main 함수에 `.await`가 없어도, async main을 사용해야 함
        thread::spawn(move || {
            let rt = Runtime::new().unwrap();
            rt.block_on(async { // .await를 위한 런타임 생성
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
                                match session_guard.as_ref().unwrap().declare_publisher(pub_key).await {
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
                            println!("Failed to establish Zenoh session: {:?}. Retrying in 5 seconds...", e);
                            tokio::time::sleep(std::time::Duration::from_secs(5)).await;
                        }
                    }
                }
            })
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

#[derive(Debug)]
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
        // ZenohMiddlewareBuilder::default().config().await.build().await.unwrap()
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

impl Communication<Vec<u8>> for ZenohMiddleware{
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
    use super::*;
    use tokio::{
        task::block_in_place,
    };
    use futures::executor::block_on;
    use std::rc::Rc;

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
