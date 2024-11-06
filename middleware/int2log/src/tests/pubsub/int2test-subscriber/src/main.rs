use capnp_message::*;
use flume::Receiver;
use std::time::Duration;
use zenoh::{pubsub::Subscriber, sample::Sample, Config};

#[tokio::main]
async fn main() {
    let config = Config::default();
    let pub_key = "log/sub/0";
    let sub_key = "log/pub";

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

    let mut start_flag = false;
    let health_check = Duration::from_secs(10);
    let mut pub_flag = false;

    publisher.put(serialize_msg(-1)).await.unwrap();

    // Publisher 켜질 때까지 waiting (Pingpong으로 확인)
    while !start_flag {
        match tokio::time::timeout(health_check, receiver(&subscriber)).await {
            Ok(Some(count)) => match count {
                -2 if !pub_flag => {
                    publisher.put(serialize_msg(0)).await.unwrap();
                    pub_flag = true;
                }
                0 if pub_flag => {
                    start_flag = true;
                }
                _ => {}
            },
            Ok(None) => {}
            Err(_) => {
                pub_flag = false;
                println!(
                    "{} seconds passed without Subscriber starting. Retrying...",
                    health_check.as_secs()
                );
                publisher.put(serialize_msg(-1)).await.unwrap();
            }
        }
    }

    println!("Starting...");
    let mut missing: Vec<i32> = Vec::new();
    let mut count = 0;

    while let Some(pub_count) = receiver(&subscriber).await {
        count += 1;
        // println!("pub: {} sub: {}", pub_count, count);

        match pub_count {
            -1 => {
                count -= 1;
                // println!("sub_count: {}, missing: {:?}", count, missing);
                break;
            }
            _ if pub_count != count => {
                for i in count..pub_count {
                    missing.push(i);
                }
                count = pub_count;
            }
            _ => {}
        }
    }
}

async fn receiver(subscriber: &Subscriber<Receiver<Sample>>) -> Option<i32> {
    if let Ok(sample) = subscriber.recv_async().await {
        let payload = sample.payload().to_bytes().into_owned();
        let (count, _) = capnp_message::deserialize_msg(&payload);
        Some(count)
    } else {
        None
    }
}
