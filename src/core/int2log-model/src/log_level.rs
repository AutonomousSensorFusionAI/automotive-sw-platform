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
        match level_str {
            "Trace" => Some(LogLevel::Trace),
            "Debug" => Some(LogLevel::Debug),
            "Info" => Some(LogLevel::Info),
            "Warn" => Some(LogLevel::Warn),
            "Error" => Some(LogLevel::Error),
            _ => None,
        }
    }
}