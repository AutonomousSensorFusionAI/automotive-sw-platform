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
    dcps::infrastructure::qos_policy::{HistoryQosPolicy, HistoryQosPolicyKind},
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

#[derive(DdsType, Readable, Writable)]
#[dds_type(crate_path = "int2dds")]
struct HelloWorldType {
    index: u32,
    message: String,
}

struct ReaderListener;

impl DataReaderListener for ReaderListener {
    type Foo = HelloWorldType;
    fn on_subscription_matched(
        &self,
        _reader: &int2dds::subscription::data_reader::DataReader<Self::Foo>,
        status: &int2dds::infrastructure::status::SubscriptionMatchedStatus,
    ) {
        let change = status.current_count_change();
        let count = status.current_count();

        if change > 0 {
            println!("Publisher matched! Current: {}", count);
        } else if change < 0 {
            println!("Publisher unmatched! Current: {}", count);
        }

        if count == 0 {
            println!("No Publishers remaining.");
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
                println!("{:?}", sample);
            }
        }
    }
}

fn main() {
    let domain_id = 40;
    let participant_qos = DomainParticipantQos::default();
    let factory = DomainParticipantFactory::get_instance();
    let participant = factory
        .create_participant(domain_id, participant_qos, None, StatusMask::default())
        .unwrap();

    let topic = participant
        .create_topic::<HelloWorldType>(
            "hello_world_topic",
            "HelloWorld",
            TopicQos::default(),
            None,
            StatusMask::default(),
        )
        .unwrap();

    let subscriber = participant
        .create_subscriber(SubscriberQos::default(), None, StatusMask::default())
        .unwrap();

    let reader_qos = DataReaderQos {
        history: HistoryQosPolicy { kind: HistoryQosPolicyKind::KeepLast(10) },
        reliability: ReliabilityQosPolicy {
            kind: ReliabilityQosPolicyKind::Reliable,
            max_blocking_time: Duration {
                sec: 0,
                nanosec: 100_000_000,
            },
        },
        ..Default::default()
    };
    let read_listener = ReaderListener;
    let _reader = subscriber
        .create_datareader::<HelloWorldType>(
            &topic,
            reader_qos,
            Some(Arc::new(read_listener)),
            StatusMask::default(),
        )
        .unwrap();

    println!(
        "[subscriber INFO] domain_id: {:?}, hostname: {:?}",
        domain_id,
        hostname::get().unwrap()
    );

    loop {
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
