#[repr(i32)]
#[derive(Debug, Default, PartialEq, PartialOrd, Copy, Clone)]
pub enum LogLevel {
    Trace = 0, 
    #[default]
    Debug = 1, // This is Default Value of Log Level
    Info = 2, 
    Warn = 3,
    Error = 4,
}

impl LogLevel {
    pub fn from_str(level_str: &str) -> Option<Self>{
        match level_str.to_lowercase().as_str() { // 대소문자 구분 없앰
            "trace" => Some(LogLevel::Trace),
            "debug" => Some(LogLevel::Debug),
            "info" => Some(LogLevel::Info),
            "warn" => Some(LogLevel::Warn),
            "error" => Some(LogLevel::Error),
            _ => None,
        }
    }
}