# Dependency Build & Installation

## Cargo and Rust

### Linux and MacOS

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Windows

download and run rustup-init.exe (https://www.rust-lang.org/tools/install)

## Capn'Proto

### Linux

```bash
apt-get install capnproto
```

### MacOS
```bash
brew install capnp
```

### Windows

download and run capnp.exe (https://capnproto.org/install.html#installation-windows)

## Test
1. `cd int2log-subscriber && cargo run`
2. `cd int2log-publisher && cargo run`
3. 결과
- Subscriber Output
```
>> [Subscriber] Received PUT ('log': 'LogMessage { log_level: Debug, data: "Hi, Debug!", timestamp: "2024/10/01 06:32:37", logger: "int2log_publisher::main::h532e1a2c93d87e38::16" }')
>> [Subscriber] Received PUT ('log': 'LogMessage { log_level: Info, data: "Hi, Info!", timestamp: "2024/10/01 06:32:37", logger: "int2log_publisher::main::h532e1a2c93d87e38::17" }')
>> [Subscriber] Received PUT ('log': 'LogMessage { log_level: Warn, data: "Hi, Warn!", timestamp: "2024/10/01 06:32:37", logger: "int2log_publisher::main::h532e1a2c93d87e38::18" }')
>> [Subscriber] Received PUT ('log': 'LogMessage { log_level: Error, data: "Hi, Error!", timestamp: "2024/10/01 06:32:37", logger: "int2log_publisher::main::h532e1a2c93d87e38::19" }')
```
- Publisher Output
```
2024/10/01 06:29:28 - Debug - Hi, Debug!
2024/10/01 06:29:28 - Info - Hi, Info!
2024/10/01 06:29:28 - Warn - Hi, Warn!
2024/10/01 06:29:28 - Error - Hi, Error!
```

## Docs
1. core 프로젝트 (int2log-core, int2log-model, int2log-zenoh) 중 원하는 디렉터리로 이동
2. `cargo doc --no-deps`