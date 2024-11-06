use capnp_message::*;
use flume::Receiver;
use std::{
    thread,
    time::{Duration, Instant},
};
use zenoh::{pubsub::Subscriber, sample::Sample, Config};

#[tokio::main]
async fn main() {
    let config_path = format!("{}/zenoh.json5", env!("CARGO_MANIFEST_DIR"));
    let config = Config::from_file(config_path).expect("Failed to load configuration file");
    let pub_key = "log/pub";
    let sub_key = "log/sub/**";
    const SUBS_NUM: usize = 2;

    println!("Opening session...");
    let session = loop {
        match zenoh::open(config.clone()).await {
            Ok(session) => break session,
            Err(e) => {
                println!("Failed to open session: {:?}. Retrying...", e);
                tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            }
        }
    }; // client일 때만 Result 작동

    println!("Declaring Publisher on '{pub_key}'...");
    println!("Declaring Subscriber on '{sub_key}'...");
    let publisher = session.declare_publisher(pub_key).await.unwrap();
    let subscriber = session.declare_subscriber(sub_key).await.unwrap();

    let hz_duration = get_duration(500);

    let mut start_flag = false;
    let health_check = Duration::from_secs(10);
    let mut sub_flag = [false; SUBS_NUM];

    // // Subscriber 켜질 때까지 waiting (Pingpong으로 확인)
    while !start_flag {
        match tokio::time::timeout(health_check, receiver(&subscriber)).await {
            Ok(Some((sub_key, count))) => match count {
                -1 if !sub_flag[sub_key] => {
                    publisher.put(serialize_msg(-2)).await.unwrap();
                    sub_flag[sub_key] = true;
                }
                0 if sub_flag[sub_key] => {
                    println!(
                        "{}th subscriber connected, waiting for other subscribers",
                        sub_key
                    );
                    if sub_flag.iter().all(|&x| x) {
                        publisher.put(serialize_msg(0)).await.unwrap();
                        start_flag = true;
                    }
                }
                _ => {}
            },
            Ok(None) => {}
            Err(_) => {
                sub_flag = [false; SUBS_NUM];
                println!(
                    "{} seconds passed without Subscriber starting. Retrying...",
                    health_check.as_secs()
                );
            }
        }
    }

    println!(
        "Starting... Setting hz: {:?}",
        1000 / hz_duration.as_millis()
    );
    subscriber.undeclare().await.unwrap();
    thread::sleep(health_check);

    let publish_duration = Duration::from_secs(5 * 60);
    let mut count = 0;
    let start_time = Instant::now();

    while start_time.elapsed() < publish_duration {
        count += 1;
        publisher.put(serialize_msg(count)).await.unwrap();
        thread::sleep(hz_duration);
    }

    publisher.put(serialize_msg(-1)).await.unwrap();
    println!("Published {} messages", count);
    publisher.undeclare().await.unwrap();
}

async fn receiver(subscriber: &Subscriber<Receiver<Sample>>) -> Option<(usize, i32)> {
    if let Ok(sample) = subscriber.recv_async().await {
        let mut sub_key = sample.key_expr().as_str().to_string();
        let sub_num = sub_key
            .pop()
            .unwrap()
            .to_digit(10)
            .map(|d| d as usize)
            .unwrap();
        let payload = sample.payload().to_bytes().into_owned();
        let (count, _) = capnp_message::deserialize_msg(&payload);
        Some((sub_num, count))
    } else {
        None
    }
}

fn get_duration(hz: u64) -> Duration {
    let millis: u64 = 1000 / hz;
    Duration::from_millis(millis)
}
