# int2DDS Performance Test

int2DDS의 성능 테스트를 위한 Publisher/Subscriber 예제입니다.

## 파일 구조

```
performance_test/
├── perftest_publisher.rs    # Publisher 구현 (Rust)
├── perftest_subscriber.rs   # Subscriber 구현 (Rust)
├── perf-build.sh           # 빌드 스크립트
├── perf-pub.sh             # Publisher 성능 테스트 자동화 스크립트
└── perf-sub.sh             # Subscriber 성능 테스트 자동화 스크립트
```

## 빌드

```bash
./perf-build.sh
```

또는 수동으로:

```bash
cargo build --release --example perftest_publisher
cargo build --release --example perftest_subscriber
```

## 테스트 모드

- **latency**: 지연시간(latency) 측정
- **throughput**: 처리량(throughput) 측정
- **local_latency**: 로컬 지연시간(local latency) 측정

## 테스트 설정

기본 설정으로 다음 시나리오를 테스트합니다:

| 테스트 모드 | 데이터 크기 | 주파수 |
|------------|------------|--------|
| latency/throughput | 1048576 bytes (1MB) | 60 Hz |
| latency/throughput | 65536 bytes (64KB) | 700 Hz |
| latency/throughput | 1024 bytes (1KB) | 1000 Hz |
| local_latency | 1048576 bytes (1MB) | 60 Hz |
| local_latency | 65536 bytes (64KB) | 700 Hz |
| local_latency | 1024 bytes (1KB) | 1000 Hz |

각 설정은 10회 반복 실행됩니다.

## 실행 방법

### Publisher 실행

자동화 스크립트 사용:
```bash
./perf-pub.sh
```

수동 실행:
```bash
./perftest_publisher --execution-time 30 --test-mode latency --warmup-time 2 --data-len 1024 --hz 1000
./perftest_publisher --execution-time 30 --test-mode local_latency --warmup-time 2 --data-len 1024 --hz 1000
```

### Subscriber 실행

자동화 스크립트 사용:
```bash
./perf-sub.sh
```

수동 실행:
```bash
./perftest_subscriber --execution-time 30 --test-mode latency --data-len 1024 --hz 1000
./perftest_subscriber --execution-time 30 --test-mode local_latency --data-len 1024 --hz 1000
```

## 실행 옵션

- `--execution-time`: 테스트 실행 시간 (초)
- `--test-mode`: 테스트 모드 (latency/throughput/local_latency)
- `--warmup-time`: 워밍업 시간 (초)
- `--data-len`: 전송 데이터 크기 (bytes)
- `--hz`: 메시지 전송 주파수 (Hz)

## 로그

- Publisher 로그: `publisher.log`
- Subscriber 로그: `subscriber.log`

## 사용 예시

1. Publisher와 Subscriber를 각각 다른 터미널에서 실행
2. 자동화 스크립트는 모든 테스트 설정을 순차적으로 실행
3. 각 테스트는 30초 실행 후 40초 주기로 반복
4. 결과는 로그 파일에 기록됨
