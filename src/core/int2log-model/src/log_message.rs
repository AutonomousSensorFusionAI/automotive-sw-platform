use super::log_level::*;
use chrono::offset::Utc;
use backtrace::Backtrace;
// use std::marker::Copy;

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct LogMessage {
    pub log_level: super::log_level::LogLevel,
    pub msg: String,
    pub timestamp: String,
    pub logger: String,
}

impl LogMessage {
    pub fn make_msg(log_level: super::log_level::LogLevel, msg: String) -> Self {
        LogMessage {
            log_level,
            msg,
            timestamp: Self::get_timestamp(),
            logger: match Self::caller_name() {
                Some(file) => file,
                None => "Unknown".to_string(),
            }
        }
    }

    pub fn msg(&mut self, log_level: super::log_level::LogLevel, msg: String) -> &mut Self{
        self.log_level = log_level;
        self.msg = msg;
        self.timestamp = Self::get_timestamp();
        self.logger = match Self::caller_name() {
            Some(file) => file,
            None => "Unknown".to_string(),
        };
        self
    }

    fn get_timestamp() -> String {
        let now: chrono::DateTime<Utc> = Utc::now();
        let now_format: String = now.format("%Y/%m/%d %T").to_string();
        now_format
    }

    #[inline(never)]
    fn caller_name() -> Option<String> {
        let backtrace = Backtrace::new();
        let symbol = backtrace
            .frames()
            .iter()
            .flat_map(|frame| frame.symbols())
            .nth(3)?;
        let name = format!("{}::{}",
                                            symbol.name()?,
                                            symbol.lineno()?);
        Some(name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // use super::log_level::*;
    use super::LogLevel;

	#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn it_works() {
        let mut message = LogMessage::default();
        println!("{:?}",message.msg(LogLevel::Info, "Hi".to_string()));
    }
}