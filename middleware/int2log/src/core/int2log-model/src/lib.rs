//! `int2log-model` is a model library for for Int2Log's logging systems.
//! It defines traits, serialization models, log levels, and log messages used in the logging system.
//! 
//! # Examples
//! ### Using Capn'Proto Serializer
//! The example below shows how to use serailzer model
//! ```
//! use int2log_model::{
//!		serializer::*,
//!		log_message::*,
//! 	log_level::*,
//!	};
//! 
//! fn main() {
//! 	let serializer_factory = SerializerFactory::new();
//!		let capnp_serializer = serializer_factory.capnp_serializer();
//!		
//! 	let mut log_message = LogMessage::default();
//! 	log_message.msg(LogLevel::Info, "This is Info");
//! 
//! 	let ser_msg = capnp_serializer.serialize_msg(&log_message);
//! 	println!("Capnp Serializer: {:?}, Serialized Data is {:?}", capnp_serializer, ser_msg);	
//! 
//! }
//! ```

use std::future::Future;
use std::pin::Pin;
use std::fmt;
pub mod log_message_capnp;
pub mod serializer;
pub mod log_level;
pub mod log_message;

pub trait Communication<T>: fmt::Debug {
	/*
		Box는 데이터를 Heap에 할당하여 컴파일 시점에 알려지지 않은 타입을 다룹니다.
		Pin은 데이터가 메모리 내에서 이동하지 않도록 보장합니다.
		Send는 타입이 스레드 간에 안전하게 전송될 수 있음을 나타냅니다. (소유권이 다른 스레드로 이동 가능, 자기참조에 유용)
		Pin<Box<dyn Future + Send>>는 Future가 다른 스레드로 전송될 수 있지만,
		그 과정에서 메모리 내 위치는 변경되지 않음을 의미하게 됩니다.
		'_: 생명주기 엘리전(lifetime elision). 추론된 생명주기를 의미하여, 컴파일러가 자동으로 추론하게끔 합니다.
		Pin<Box<dyn Future<Output = ()> + Send + '_>>에서의 역할:
			이 경우, '_는 반환된 Future의 생명주기가 self의 생명주기와 연결되어 있음을 나타냅니다.
			Future가 self에 대한 참조를 포함할 수 있음을 의미합니다.
		
		원래 impl Future<Output = ()> + Send 형태를 사용해도 무방했으나,
		MiddlewareLogger에서 middleware 필드에 Rc<dyn Communication<T>> 를 사용하면서,
		Pin<Box<dyn Future<Output = ()> + Send + '_>> 로 수정을 하게 되었습니다.
		
		반환 타입을 impl Future<Output = ()> + Send로 설정할 경우,
		impl Trait은 컴파일 시점에 구체적인 타입을 결정하지만, 
		dyn은 런타임에 동적 디스패치를 사용하여 이 두 개념이 충돌하게 되어 
		이 Communication 트레잇을 동적으로 사용할 수 없게 됩니다. 
		Box<dyn>을 사용하여 트레잇이 object safe하도록 수정하고,
		Pin과 Send로 안전성을 보장하였습니다.
	*/
	/// The publish function of the middleware publisher
	fn sender(&self, t: T) -> Pin<Box<dyn Future<Output = ()> + Send + '_>>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {

    }
}
