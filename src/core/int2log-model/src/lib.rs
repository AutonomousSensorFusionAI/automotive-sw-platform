use std::future::Future;
pub mod log_message_capnp;
pub mod serializer;
pub mod log_level;
pub mod log_message;

pub trait Communication<T> {
	fn sender(&self, t: T) -> impl Future<Output = ()>; // async fn
}

#[repr(C)]
#[derive(Debug)]
pub struct DefaultMiddleware;

impl Default for DefaultMiddleware {
    fn default() -> Self {
		DefaultMiddleware // DefaultMiddleware에 대한 기본 구현
    }
}

impl DefaultMiddleware {
	async fn receiver(&self) {
		unimplemented!("You need to implement function the receiver function of Middleware.");
	}
}

impl Communication<String> for DefaultMiddleware {
	async fn sender(&self, data: String) {
		unimplemented!("You need to implement function the sender function of Middleware. Your Data is {}", &data);
	}
}

impl Communication<Vec<u8>> for DefaultMiddleware {
	async fn sender(&self, data: Vec<u8>) {
		unimplemented!("You need to implement function the sender function of Middleware. Your Data is {:?}", &data);
	}
}



#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {

    }
}
