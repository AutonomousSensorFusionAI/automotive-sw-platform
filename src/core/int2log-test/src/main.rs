use int2log_core::*;
use int2log_common::*;
use int2log_model::*;
use int2log_zenoh::*;

// #[tokio::main]
#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() {
	/* Log version 1) 
	 Log + Capnp + Zenoh  */
	// let mut log = Log::default().await;
	let mut log = Log::default().await;
	/* Log version 2) 
	 Log + DefaultSerializer(Data to String or Vec<u8>, not Capnp) + DefaultMiddleware(Send, Receive - X) */
	// let mut log: Log<String, DefaultSerializer, DefaultMiddleware> = Log::log().middleware(DefaultMiddleware).serializer(DefaultSerializer);

	// Log version 3) Log + Capnp + DefaultMiddleware(Send, Receive - X)
	// let mut log: Log<Vec<u8>, CapnpSerializer, DefaultMiddleware> = Log::log().serializer(CapnpSerializer).middleware(DefaultMiddleware);
	
	log.debug(String::from(format!("Default Setting: {:?}", log))).await;
	log.set_publish_level(LogLevel::Warn);
	log.debug(String::from(format!("Change Pub log level: {:?}", log))).await;
	log.set_print_level(LogLevel::Debug);
	log.debug(String::from(format!("Change Print log level: {:?}", log))).await;
	
	log.trace(String::from("This is Trace!")).await;
	log.debug(String::from("This is Debug!")).await;
	log.info(String::from("This is Info!")).await;
	log.warn(String::from("This is Warn!")).await;
	log.error(String::from("This is Error!")).await;
	log.debug(String::from(format!("log: {:?}", log))).await;
}