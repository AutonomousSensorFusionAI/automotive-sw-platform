use derivative::Derivative;
use std::{
	fmt::Debug,
	fs::{File, OpenOptions},
	io::Write,
	marker::PhantomData,
	rc::Rc,
	cell::RefCell,
};
use tokio::{
	task::block_in_place,
};
use futures::executor::block_on;

use int2log_model::*;
use int2log_model::log_level::*;
use int2log_model::log_message::*;
use int2log_model::serializer::*;
use int2log_zenoh::*;


/*
	- Rust의 supertrait에 관한 내용
	trait ILogging: Debug 에서 ':Debug'는 supertrait(고급 트레잇, 슈퍼 트레잇)이라고 부릅니다. 
	이는 ILogging 트레잇을 구현하는 타입은 Debug 타입을 구현해야 함을 의미합니다.

	여기서 ':Debug' 가 필요한 이유는 struct Logger가 Debug를 구현한다는 #[derive(Debug)] 특성을 지정했기 때문입니다.
	
	Struct Logger의 필드인 loggers는 ILogging 트레잇을 구현하는 어떤 타입도 가질 수 있게 되는데(dyn ILogging),
	컴파일러는 이 ILogging 트레잇을 구현하는 동적인 객체가 Dubug 타입을 구현한다는 것을 알 수 없으므로
	ILogging: Debug 를 정의하지 않으면, #[derive(Debug)]를 Logger 구조체에 사용할 수 없습니다.

	- ILogging 트레잇 설명
	ILogging 트레잇은 콘솔 로거, 파일 로거, 미들웨어 로거 등 로거에서 반드시 구현해야하는 트레잇입니다.
	여기서 로거는 log4j의 어펜더와 비슷한 것이라고 보시면 됩니다.
	해당 로거들에서 반드시 필요할 것이라고 생각한
	로그 레벨 세팅, active 여부 세팅(set true, false), 로그 메시지 처리(process),
	그리고 사용자가 로그 레벨을 세팅하지 않은 경우, 
	기본 로그 시스템의 로그 레벨을 가져올 수 있도록 하는 get_log_level 함수로 이루어져 있습니다.
*/
// This is for public function
pub trait LoggingSpec: Debug {
	fn set_log_level(&mut self, log_level: &str);
	fn set_active_true(&mut self);
	fn set_active_false(&mut self);
}
// This is for hidden function
trait LogCommon: Debug {
	fn get_log_level(&mut self, log_level: LogLevel);
	fn process(&self, log_message: &LogMessage);
}
#[allow(private_bounds)]// LogCommon->Hidden은 의도했던 것 => warning 무시
pub trait ILogging: LoggingSpec + LogCommon {}

/*
	- ILogger 트레잇 설명
	ILogger는 Logger(사용자가 세팅한 로거 집합)가 구현해야 하는 트레잇입니다.
	attach 함수로 원하는 로거를 붙이거나, detach로 제거 가능합니다.
	또한, get_log_level의 역할은 위 ILogging의 get_log_level 함수와 같습니다.
	loggers.get_log_level -> inner_logger.get_log_level 호출로 처리하게 됩니다.
	process 또한 위와 동일합니다.
*/
// This is for public function
pub trait LoggerSpec {
	fn attach(&mut self, logger: Rc<RefCell<dyn ILogging>>);
	fn detach(&mut self, logger: Rc<RefCell<dyn ILogging>>);
}
// This is for hidden function
// trait LogCommon {
// 	fn get_log_level(&self, log_level: LogLevel);
// 	fn process(&self, log_message: &LogMessage);
// }
#[allow(private_bounds)] // LogCommon->Hidden은 의도했던 것 => warning 무시
pub trait ILogger: LoggerSpec + LogCommon {}

#[cfg(test)]
mod logger_examples{
	use super::*;

	#[test]
	fn logger_example(){
		let default_logger = Logger::default();
		println!("{:?}", default_logger.default_console); // Some(RefCell { value: ConsoleLogger { log_level: Debug, active: true, set_flag: false } })

		let def_console_logger = default_logger.default_console.as_ref().unwrap();
		def_console_logger.borrow_mut().set_active_false();
		def_console_logger.borrow_mut().set_log_level("error");
		
		println!("{:?}", default_logger.default_console); // Some(RefCell { value: ConsoleLogger { log_level: Error, active: false, set_flag: true } })
	}

	#[test]
	fn none_ref_ex() {
		let default_logger = Logger::default();
		println!("{:?}", default_logger.default_console);

		let def_console_logger = default_logger.default_console.unwrap();
		def_console_logger.borrow_mut().set_active_false();
		def_console_logger.borrow_mut().set_log_level("error");
		
		// println!("{:?}", default_logger); // 에러
	}
}
/*
	- Option<...>
	Logger를 default로 정의한 경우(Logger::default()), 콘솔, 파일, 미들웨어 로거에 각각 접근하기가 어려워집니다.
	위의 예제 logger_example() 처럼 default인 경우에도 로그 레벨, active 세팅을 가능하게 하고자
	default_console, default_file, default_middeleware 필드를 따로 정의하였습니다.
	Rust에는 Null이 없으므로 default가 아닌 경우를 위해 Option을 사용하였습니다.
	default인 경우 Some(logger) 타입, 아닌 경우 None 타입을 필드에 저장합니다.

	- Vec<...>
	로거(트레잇 ILogging을 구현하는 동적 객체 타입)들을 저장하기 위해 사용하였습니다.
	
	- Rc<RefCell<..>>
	러스트 소유권 규칙 때문에 사용한 타입입니다.
	위의 예제 logger_example() 에서 def_console_logger 변수 정의시 .as_ref()를 사용하였는데, 
	none_ref_ex() 에서처럼 def_console_logger 정의할 때 .as_ref()를 따로 사용하지 않는 경우를 살펴보면,
	error[E0382]: borrow of partially moved value: `default_logger` 라는 에러를 내면서 동작하지 않습니다.
	default_logger.default_console의 소유권을 def_console_logger에 빼앗겼기 때문입니다. 이를 방지하고자 <Rc<RefCell<..>> 타입을 사용하였습니다.
	
	Rc<T>는 참조 카운터(다중 소유권 지원) 스마트 포인터의 일종이며, 
	RefCell<T>는 내부 가변성 패턴(단일 소유권 지원, 불변 참조를 사용하면서 값을 수정할 수 있도록 함)을 사용하는 스마트 포인터입니다.
	Rc<RefCell<T>> 로 사용한다면, 다중 소유권과 내부 가변성을 가진 자료형을 사용할 수 있게 됩니다.
	자세한 내용: https://doc.rust-kr.org/ch15-05-interior-mutability.html
*/
#[derive(Debug)]
/// Logger 집합 구조체
pub struct Logger {
	/// It is a collection of either default-defined or user-defined loggers that implement the ILogging interface.
	loggers: Vec<Rc<RefCell<dyn ILogging>>>,
	/// The field exists or is None only when defined as the default.
	default_console: Option<Rc<RefCell<ConsoleLogger>>>,
	/// The field exists or is None only when defined as the default.
	default_file: Option<Rc<RefCell<FileLogger>>>,
	/// The field exists or is None only when defined as the default.
	default_middeleware: Option<Rc<RefCell<MiddlewareLogger<Vec<u8>>>>>,
}

/*
	- Logger의 Default 트레잇
	Logger가 가진 필드의 타입들(Vec, Option)은 Default 트레잇 구현없이도 정의할 수 있습니다.
	#[derive(Default)] 사용시: Logger { loggers: [], default_console: None, default_file: None, default_middeleware: None }

	편의를 위해 다른 로거들(콘솔, 파일, 미들웨어)을 정의하지 않고도 기본으로 사용할 수 있도록 Default 트레잇을 따로 정의하였습니다.
 */
/// Logger에 ConsoleLogger, FileLogger, MiddlewareLogger(Zenoh) 집합 등록
impl Default for Logger {
	fn default() -> Self {
		let mut logger = Logger::new();
		let console_logger = Rc::new(RefCell::new(ConsoleLogger::default()));
		let file_logger = Rc::new(RefCell::new(FileLogger::default()));
		let middleware_logger = Rc::new(RefCell::new(MiddlewareLogger::default()));
		logger.default_console = Some(console_logger.clone());
		logger.default_file = Some(file_logger.clone());
		logger.default_middeleware = Some(middleware_logger.clone());
		// loggers 필드에도 default logger들 추가
		logger.attach(console_logger.clone());
		logger.attach(file_logger.clone());
		logger.attach(middleware_logger.clone());
		logger
	}
}

impl Logger {
	pub fn new() -> Self {
		Logger {
			loggers: Vec::new(),
			default_console: None,
			default_file: None,
			default_middeleware: None,
		}
	}

	/// Note that it is only available when defined as the default.
	/// Return type: (Rc<RefCell<ConsoleLogger>>, Rc<RefCell<FileLogger>>, Rc<RefCell<MiddlewareLogger<Vec<u8>>>>)
	pub fn get_default_logger(&self) -> (Rc<RefCell<ConsoleLogger>>, Rc<RefCell<FileLogger>>, Rc<RefCell<MiddlewareLogger<Vec<u8>>>>) {
		(self.default_console(), self.default_file(), self.default_middeleware())
	}
	/// Note that it is only available when defined as the default.
	/// Retrun type: Rc<RefCell<ConsoleLogger>>
	pub fn default_console(&self) -> Rc<RefCell<ConsoleLogger>> {
		self.default_console.clone().expect("You don't have Default Console Logger. Please check if you defined the Logger using the default function.")
	}
	/// Note that it is only available when defined as the default.
	/// Retrun type: Rc<RefCell<FileLogger>>
	pub fn default_file(&self) -> Rc<RefCell<FileLogger>> {
		self.default_file.clone().expect("You don't have Default File Logger. Please check if you defined the Logger using the default function.")
	}
	/// Note that it is only available when defined as the default.
	/// Retrun type: Rc<RefCell<MiddlewareLogger<Vec<u8>>>>
	pub fn default_middeleware(&self) -> Rc<RefCell<MiddlewareLogger<Vec<u8>>>> {
		self.default_middeleware.clone().expect("You don't have Default Middleware Logger. Please check if you defined the Logger using the default function.")
	}
}

impl LoggerSpec for Logger {
	/// You can use this function to attach your logger.
	fn attach(&mut self, logger: Rc<RefCell<dyn ILogging>>) {
		self.loggers.push(logger);
	}
	/// You can use this function to detach your logger.
	fn detach(&mut self, logger: Rc<RefCell<dyn ILogging>>) {
		self.loggers.retain(|l| !Rc::ptr_eq(l, &logger));
	}
}

impl LogCommon for Logger {
	/// If you do not specify the level for your logger, the Logger system will use this function to retrieve the log level defined in the Log system (Struct Log).
	fn get_log_level(&mut self, log_level: LogLevel) {
		for item in self.loggers.iter() {
			let mut logger_ref = item.borrow_mut();
			logger_ref.get_log_level(log_level);
		}
	}
	/// Processing Log Message.
	fn process(&self, log_message: &LogMessage) {
		for item in self.loggers.iter() {
			let logger_ref = item.borrow();
			logger_ref.process(&log_message);
		}
	}
}

impl ILogger for Logger {}

/*
	원래 bool의 Default 타입은 false입니다.
	#[derive(Derivative)] 사용 후
	#[derivative(Default(value="true"))]로 기본 타입을 따로 정의할 수 있습니다.
*/
#[derive(Derivative)]
#[derivative(Default, Debug)]
/// Console Logger
pub struct ConsoleLogger {
	log_level: LogLevel,
	#[derivative(Default(value="true"))]
	active: bool,
	set_flag: bool,
}

#[derive(Derivative)]
#[derivative(Debug)]
/// Middleware Logger
/// Default Middleware is Zenoh
pub struct MiddlewareLogger<T> 
{	
	// 여러 MiddlewareLogger 인스턴스가 동일한 middleware나 serializer를 참조할 수 있도록 Rc로 래핑.
	// 해당 트레잇을 구현하는 타입만 필드로 저장할 수 있도록 동적 디스패치 사용
	middleware: Rc<dyn Communication<T>>,
	serializer: Rc<dyn Serialization<T>>,
	log_level: LogLevel,
	#[derivative(Default(value="true"))]
	active: bool,
	set_flag: bool,
	_phantom: PhantomData<T>, // T를 위해 정의된 필드.
}

#[derive(Derivative)]
#[derivative(Default, Debug)]
pub struct FileLogger {
	log_level: LogLevel,
	/// If no file_path is provided, it will search for or create 'log.txt' in the current directory.
	/// You can set your file_path using set_file_path("your_path").
	#[derivative(Default(value="FileLogger::default_file_path()"))]
	file_path: String,
	#[derivative(Default(value="true"))]
	active: bool,
	set_flag: bool,
}

impl LogCommon for ConsoleLogger{
	/// If you do not specify the level for your logger, the ConsoleLogger will use this function to retrieve the log level defined in the Log system (Struct Log).
	fn get_log_level(&mut self, log_level: LogLevel) {
		if self.set_flag == false {
			self.log_level = log_level;
		}
	}
	/// Processing Log Message.
	fn process(&self, log_message: &LogMessage) {
		if self.active == true {
			if (self.log_level) <= (log_message.log_level) {
				match log_message.log_level { 
					LogLevel::Trace => println!("{} - Trace - {}", log_message.timestamp, &log_message.msg),
					LogLevel::Debug => println!("{} - Debug - {}", log_message.timestamp, &log_message.msg),
					LogLevel::Info => println!("{} - Info - {}", log_message.timestamp, &log_message.msg),
					LogLevel::Warn => println!("{} - Warn - {}", log_message.timestamp, &log_message.msg),
					LogLevel::Error => println!("{} - Error - {}", log_message.timestamp, &log_message.msg),
				}
			}
		}
	}
}
impl LoggingSpec for ConsoleLogger {
	/// You can set the log level like logger.set_log_level("trace").
	fn set_log_level(&mut self, log_level: &str) {
		self.set_flag = true;
		match log_level {
			"trace" => self.log_level = LogLevel::Trace,
			"debug" => self.log_level = LogLevel::Debug,
			"info" => self.log_level = LogLevel::Info,
			"warn" => self.log_level = LogLevel::Warn,
			"error" => self.log_level = LogLevel::Error,
			_ => println!("To choose between 'trace', 'debug', 'info', 'warn', and 'error'"),
		}
	}
	/// You can set the active true like logger.set_active_true().
	fn set_active_true(&mut self) {
		self.active = true;
	}
	/// You can set the active false like logger.set_active_false().
	fn set_active_false(&mut self) {
		self.active = false;
	}
}
impl ILogging for ConsoleLogger {}

impl LogCommon for FileLogger {
	/// If you do not specify the level for your logger, the FileLogger will use this function to retrieve the log level defined in the Log system (Struct Log).
	fn get_log_level(&mut self, log_level: LogLevel) {
		if self.set_flag == false {
			self.log_level = log_level;
		}
	}
	/// Processing Log Message.
	fn process(&self, log_message: &LogMessage) {
		let mut file = self.get_log_file();
		if self.active == true {
			if (self.log_level) <= (log_message.log_level) {
				let log_entry = match log_message.log_level {
					LogLevel::Trace => format!("{} - Trace - {} \n", log_message.timestamp, &log_message.msg),
					LogLevel::Debug => format!("{} - Debug - {} \n", log_message.timestamp, &log_message.msg),
					LogLevel::Info => format!("{} - Info - {} \n", log_message.timestamp, &log_message.msg),
					LogLevel::Warn => format!("{} - Warn - {} \n", log_message.timestamp, &log_message.msg),
					LogLevel::Error => format!("{} - Error - {} \n", log_message.timestamp, &log_message.msg),
				};
				file.write_all(log_entry.as_bytes()).expect("Failed to write log file");
			}
		}
	}
}
impl LoggingSpec for FileLogger {
	/// You can set the log level like logger.set_log_level("trace").
	fn set_log_level(&mut self, log_level: &str) {
		self.set_flag = true;
		match log_level {
			"trace" => self.log_level = LogLevel::Trace,
			"debug" => self.log_level = LogLevel::Debug,
			"info" => self.log_level = LogLevel::Info,
			"warn" => self.log_level = LogLevel::Warn,
			"error" => self.log_level = LogLevel::Error,
			_ => println!("To choose between 'trace', 'debug', 'info', 'warn', and 'error'"),
		}
	}
	/// You can set the active true like logger.set_active_true().
	fn set_active_true(&mut self) {
		self.active = true;
	}
	/// You can set the active false like logger.set_active_false().
	fn set_active_false(&mut self) {
		self.active = false;
	}
}
impl ILogging for FileLogger {}

impl FileLogger {
	// Default 정의를 위한 함수
	fn default_file_path() -> String {
		"log.txt".to_string()
	}

	pub fn set_file_path(&mut self, file_path: &str) {
		self.file_path = file_path.to_string();
	}

	// Message 처리시 사용자가 지정한 or default Path에 있는 Log File 찾는 함수
	fn get_log_file(&self) -> File{
		let file = OpenOptions::new()
			.write(true)
			.append(true) // 이어쓰기 허용
			.create(true)
			// .truncate(true) // 파일 초기화
			.open(&self.file_path)
			.unwrap();
		file
	}
}

impl<T> LogCommon for MiddlewareLogger<T> 
where
	T: Debug,
{
	/// If you do not specify the level for your logger, the MiddlewareLogger will use this function to retrieve the log level defined in the Log system (Struct Log).
	fn get_log_level(&mut self, log_level: LogLevel) {
		if self.set_flag == false {
			self.log_level = log_level;
		}
	}
	/// Processing Log Message.
	fn process(&self, log_message: &LogMessage) {
		if self.active == true {
			if (self.log_level) <= (log_message.log_level) {
				let data: T = self.serializer.serialize_msg(log_message);
				// process 함수를 동기함수로 사용하기 위해 tokio::task::block_in_place 사용
				block_in_place(|| {
					block_on(async {
						self.middleware.sender(data).await
					})
				})
			}
		}
	}
}
impl<T> LoggingSpec for MiddlewareLogger<T> 
where
	T: Debug,
{
	/// You can set the log level like logger.set_log_level("trace").
	fn set_log_level(&mut self, log_level: &str) {
		self.set_flag = true;
		match log_level {
			"trace" => self.log_level = LogLevel::Trace,
			"debug" => self.log_level = LogLevel::Debug,
			"info" => self.log_level = LogLevel::Info,
			"warn" => self.log_level = LogLevel::Warn,
			"error" => self.log_level = LogLevel::Error,
			_ => println!("To choose between 'trace', 'debug', 'info', 'warn', and 'error'"),
		}
	}
	/// You can set the active true like logger.set_active_true().
	fn set_active_true(&mut self) {
		self.active = true;
	}
	/// You can set the active false like logger.set_active_false().
	fn set_active_false(&mut self) {
		self.active = false;
	}
}
impl<T: Debug> ILogging for MiddlewareLogger<T> {}

impl MiddlewareLogger<Vec<u8>> {
	pub fn default() -> Self {
		// middleware default 생성을 동기함수로 사용하기 위해 tokio::task::block_in_place 사용
		let middleware = Rc::new(block_in_place(|| {
			block_on(async {
				ZenohMiddleware::default().await
			})
		}));
		let serializer = SerializerFactory::new().capnp_serializer();
		MiddlewareLogger::new(serializer, middleware)
	}
}

impl<T> MiddlewareLogger<T> {
	pub fn new(serializer: Rc<dyn Serialization<T>>, middleware: Rc<dyn Communication<T>>) -> Self {
		MiddlewareLogger {
			log_level: Default::default(),
			serializer,
			middleware,
			active: true,
			set_flag: false,
			_phantom: Default::default(),
		}
	}
	pub fn serializer(self, serializer: Rc<dyn Serialization<T>>) -> Self {
		MiddlewareLogger {
			log_level: self.log_level,
			serializer,
			middleware: self.middleware,
			active: self.active,
			set_flag: self.set_flag,
			_phantom: Default::default(),
		}
    }
	pub fn middleware(self, middleware: Rc<dyn Communication<T>>) -> Self {
		MiddlewareLogger {
			log_level: self.log_level,
			serializer: self.serializer,
			middleware,
			active: self.active,
			set_flag: self.set_flag,
			_phantom: Default::default(),
		}
    }
}

#[test]
fn create_middleware_logger() {
	let middlware_log = MiddlewareLogger::default();
	println!("{:?}", middlware_log);
	let serializer = SerializerFactory::new().capnp_serializer();
	// 변경 가능
	let middlware_log = MiddlewareLogger::default().serializer(serializer);
	println!("{:?}", middlware_log);
}

#[repr(C)]
#[derive(Derivative)]
#[derivative(Debug, Default)]
pub struct Log {
	pub log_level: LogLevel,
	pub log_message: LogMessage,
	// Custom한 Logger를 대입 후 소유권이 이전 -> 수정이 불가(detach 등의 작업 어려움), 
	// 수정 가능하도록 Rc<RefCell<..>>로 래핑
	pub logger: Rc<RefCell<Logger>>,
}

impl Log {
	pub fn new(log_level: &str, logger: Rc<RefCell<Logger>>) -> Self {
		// 사용자로부터 입력 받은 로그 레벨 -> enum 변환
		let log_level = match log_level {
			"trace" => Some(LogLevel::Trace),
			"debug" => Some(LogLevel::Debug),
			"info" => Some(LogLevel::Info),
			"warn" => Some(LogLevel::Warn),
			"error" => Some(LogLevel::Error),
			_ => None,
		};
		let log_level = log_level.expect("To choose between 'trace', 'debug', 'info', 'warn', and 'error'");

		// Logger에 해당 레벨 세팅 (Logger 생성시 세팅 되었을 경우 적용 X)
		logger.borrow_mut().get_log_level(log_level);

		Log { logger, .. Default::default() }
	}

    pub fn process(&mut self, log_level: LogLevel, msg: String) {
		// Log Message 생성 후 로거에 전달
		self.log_message.msg(log_level, msg);
		self.logger.borrow().process(&self.log_message);
		self.log_message = LogMessage{..Default::default()}; // Message 초기화
	}

    pub fn trace(&mut self, msg: String) {
		self.process(LogLevel::Trace, msg);
	}

    pub fn debug(&mut self, msg: String) {
		self.process(LogLevel::Debug, msg);
	}

    pub fn info(&mut self, msg: String) {
		self.process(LogLevel::Info, msg);
	}

    pub fn warn(&mut self, msg: String) {
		self.process(LogLevel::Warn, msg);
	}

    pub fn error(&mut self, msg: String) {
		self.process(LogLevel::Error, msg);
	}
}

#[cfg(test)]
mod tests {
    use super::*;

	#[test]
	fn logger_works() {
		let mut logger = Rc::new(RefCell::new(Logger::new()));
		let console_logger_a = Rc::new(RefCell::new(ConsoleLogger::default()));
		let console_logger_b = Rc::new(RefCell::new(ConsoleLogger {
			active: false,
			..Default::default()
		}));
		let file_logger = Rc::new(RefCell::new(FileLogger {..Default::default()}));
		console_logger_b.borrow_mut().set_log_level("error");
		let middleware = Rc::new(RefCell::new(MiddlewareLogger::default()));
		logger.borrow_mut().attach(console_logger_a.clone());
		logger.borrow_mut().attach(console_logger_b.clone());
		logger.borrow_mut().attach(file_logger.clone());
		logger.borrow_mut().attach(middleware);
		println!("{:?}", logger);

		logger.borrow_mut().detach(console_logger_a.clone());
		println!("{:?}", logger);
		logger.borrow_mut().attach(console_logger_a.clone());
		console_logger_b.borrow_mut().set_active_true();
		println!("{:?}", logger);
		
		let log_message = LogMessage::make_msg(LogLevel::Error, "Hi im error".to_string());
		logger.borrow_mut().process(&log_message);
	}

	#[test]
	fn log_works() {
		let mut log = Log::default();
		let my_console = log.logger.borrow().default_console();
		my_console.borrow_mut().set_log_level("error");
		println!("{:?}", log);
		log.error("Hi, Error!".to_string());
	}

	#[test]
	fn custom_log_works() {
		let logger = Rc::new(RefCell::new(Logger::new()));
		let console_logger_a = Rc::new(RefCell::new(ConsoleLogger::default()));
		let console_logger_b = Rc::new(RefCell::new(ConsoleLogger::default()));
		let file_logger = Rc::new(RefCell::new(FileLogger::default()));
		console_logger_b.borrow_mut().set_log_level("error");
		let middleware = Rc::new(RefCell::new(MiddlewareLogger::default()));
		logger.borrow_mut().attach(console_logger_a.clone());
		logger.borrow_mut().attach(console_logger_b.clone());
		logger.borrow_mut().attach(file_logger.clone());
		logger.borrow_mut().attach(middleware.clone());
		// let mut log = Log::default();
		let mut log = Log::new("debug", logger.clone());
		println!("{:?}", log);
		log.error("Hi, Error!".to_string());
		logger.borrow_mut().detach(console_logger_b);
		println!("{:?}", log);
		log.error("Hi, Error!".to_string());
	}
}