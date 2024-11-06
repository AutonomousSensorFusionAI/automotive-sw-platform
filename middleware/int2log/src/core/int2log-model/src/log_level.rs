#[repr(i32)]
#[derive(Debug, Default, PartialEq, PartialOrd, Copy, Clone)]
pub enum LogLevel {
    Trace = 0, 
    #[default]
    /// This is Default Value of Log Level
    Debug = 1,
    Info = 2, 
    Warn = 3,
    Error = 4,
}

impl LogLevel {
    /// This function converts a type T, which can be converted to a String, into a Option<LogLevel>."
    /// # Example
    /// ```
    /// use int2log_model::log_level::*;
    /// 
    /// fn main() {
    ///     let info_str = "info";
    ///     let info_level = LogLevel::from_str(info_str).unwrap();
    /// }
    /// ```
    pub fn from_str<T>(level_str: T) -> Option<Self>
    where 
        T: Into<String>
    {
        match level_str.into().to_lowercase().as_str() { // 대소문자 구분 없앰
            "trace" => Some(LogLevel::Trace),
            "debug" => Some(LogLevel::Debug),
            "info" => Some(LogLevel::Info),
            "warn" => Some(LogLevel::Warn),
            "error" => Some(LogLevel::Error),
            _ => None,
        }
    }
}