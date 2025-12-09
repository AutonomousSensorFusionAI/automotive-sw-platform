use std::sync::{
    mpsc::{sync_channel, SyncSender},
    Arc,
};

use int2dds::{
    common::{
        env::{set_console_log_level, set_log_type},
        log::{LogLevel, LogType},
    },
    core::time::Duration,
    domain::{domain_participant_factory::DomainParticipantFactory, qos::DomainParticipantQos},
    infrastructure::{
        qos_policy::{ReliabilityQosPolicy, ReliabilityQosPolicyKind},
        status::StatusMask,
    },
    subscription::{
        data_reader_listener::DataReaderListener,
        qos::{DataReaderQos, SubscriberQos},
        sample_info::{InstanceStateKind, SampleStateKind, ViewStateKind},
    },
    topic::{qos::TopicQos, type_support::DdsType},
};
use speedy::{Readable, Writable};

struct SubListener {
    sender: SyncSender<bool>,
}

impl DataReaderListener for SubListener {
    type Foo = HelloWorldType;
    fn on_subscription_matched(
        &self,
        _reader: &int2dds::subscription::data_reader::DataReader<Self::Foo>,
        status: &int2dds::infrastructure::status::SubscriptionMatchedStatus,
    ) {
        if status.current_count() > 0 {
            // info!("Publisher matched! Sending start signal...");
            println!("Publisher matched! Sending start signal...");
            // Send signal from callback and return immediately
            let _ = self.sender.try_send(true);
        } else {
            // info!("No Publishers. Sending stop signal...");
            println!("No Publishers. Sending stop signal...");
            let _ = self.sender.try_send(false);
        }
    }

    fn on_data_available(
        &self,
        reader: &int2dds::subscription::data_reader::DataReader<Self::Foo>,
    ) {
        if let Ok(samples) = reader.take(
            1,
            &[SampleStateKind::ANY_SAMPLE_STATE],
            &[ViewStateKind::ANY_VIEW_STATE],
            &[InstanceStateKind::ANY_INSTANCE_STATE],
        ) {
            for sample in samples.iter() {
                let sample = sample.data().unwrap();
                // info!("Read sample: {:?}", sample);
                println!("Read sample: {:?}", sample);
            }
        }
    }
}

#[derive(DdsType, Readable, Writable)]
#[dds_type(crate_path = "int2dds")]
struct HelloWorldType {
    index: u32,
    message: String,
}

//tmux attach -t int2multirp
//cargo run --package int2dds --example 5x_hello_world_2_participant_publisher
fn main() {
    set_log_type(LogType::Console);
    set_console_log_level(LogLevel::Info);

    let domain_id_50 = 50;
    let domain_id_51 = 51;
    let participant_qos = DomainParticipantQos::default();
    let factory = DomainParticipantFactory::get_instance();
    let participant_50 = factory
        .create_participant(domain_id_50, participant_qos.clone(), None, StatusMask::default())
        .unwrap();
    let participant_51 = factory
        .create_participant(domain_id_51, participant_qos.clone(), None, StatusMask::default())
        .unwrap();

    let topic_50 = participant_50
        .create_topic::<HelloWorldType>(
            "hello_world_topic",
            "HelloWorld",
            TopicQos::default(),
            None,
            StatusMask::default(),
        )
        .unwrap();
    let topic_51 = participant_51
        .create_topic::<HelloWorldType>(
            "hello_world_topic",
            "HelloWorld",
            TopicQos::default(),
            None,
            StatusMask::default(),
        )
        .unwrap();

    let subscriber_50 = participant_50
        .create_subscriber(SubscriberQos::default(), None, StatusMask::default())
        .unwrap();
    let subscriber_51 = participant_51
        .create_subscriber(SubscriberQos::default(), None, StatusMask::default())
        .unwrap();

    let reader_qos = DataReaderQos {
        reliability: ReliabilityQosPolicy {
            kind: ReliabilityQosPolicyKind::Reliable,
            max_blocking_time: Duration { sec: 0, nanosec: 100_000_000 },
        },
        ..Default::default()
    };

    let (sender, _receiver) = sync_channel(0);
    let read_listener_50 = SubListener { sender: sender.clone() };
    let read_listener_51 = SubListener { sender: sender.clone() };
    let _reader_50 = subscriber_50
        .create_datareader::<HelloWorldType>(
            &topic_50,
            reader_qos.clone(),
            Some(Arc::new(read_listener_50)),
            StatusMask::default(),
        )
        .unwrap();
    let _reader_51 = subscriber_51
        .create_datareader::<HelloWorldType>(
            &topic_51,
            reader_qos.clone(),
            Some(Arc::new(read_listener_51)),
            StatusMask::default(),
        )
        .unwrap();

    println!(
        "[subscriber INFO] domain_id: {:?}, hostname: {:?}",
        domain_id_50,
        hostname::get().unwrap()
    );
    println!(
        "[subscriber INFO] domain_id: {:?}, hostname: {:?}",
        domain_id_51,
        hostname::get().unwrap()
    );

    loop {
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
