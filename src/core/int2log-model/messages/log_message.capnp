@0xca236165c7442811; # $ capnp id

struct LogMessage {
  logLevel @0 :LogLevel; # _ x -> use camelCase
  msg @1 :Text;
  timestamp @2 :Text;
  
  enum LogLevel {
    trace @0;
    debug @1;
    info @2;
    warn @3;
    error @4;
  }
}