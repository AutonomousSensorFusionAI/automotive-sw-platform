use crate::log_level::*;
use chrono::offset::Utc;
use backtrace::Backtrace;
// use std::marker::Copy;

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct LogMessage {
    pub log_level: LogLevel,
    pub data: String,
    pub timestamp: String,
    pub logger: String,
}

impl LogMessage {
    pub fn new<T>(log_level: LogLevel, data: T) -> Self 
    where
        T: Into<String>,
    {
        LogMessage {
            log_level,
            data: data.into(),
            timestamp: Self::get_timestamp(),
            logger: match Self::caller_name() {
                Some(caller_info) => caller_info,
                None => "Unknown".to_string(),
            }
        }
    }

    pub fn msg<T>(&mut self, log_level: LogLevel, data: T)
    where
        T: Into<String>,
    {
        self.log_level = log_level;
        self.data = data.into();
        self.timestamp = Self::get_timestamp();
        self.logger = match Self::caller_name() {
            Some(caller_info) => caller_info,
            None => "Unknown".to_string(),
        };
    }

    fn get_timestamp() -> String {
        let now: chrono::DateTime<Utc> = Utc::now();
        let now_format: String = now.format("%Y/%m/%d %T").to_string();
        now_format
    }

    // 로그가 발생한 코드 정보 출력(depth 수정해야 할 수 있음)
    #[inline(never)]
    fn caller_name() -> Option<String> {
        let backtrace = Backtrace::new();
        let symbol = backtrace
            .frames()
            .iter()
            .flat_map(|frame| frame.symbols())
            .nth(4)?;
        let name = format!("{}::{}",
                                            symbol.name()?,
                                            symbol.lineno()?);
        Some(name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

	#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn it_works() {
        let mut message = LogMessage::default();
        message.msg(LogLevel::Info, "Hi");
        println!("{:?}",message.data);
    }
}