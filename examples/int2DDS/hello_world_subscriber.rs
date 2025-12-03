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

#[derive(DdsType, Readable, Writable)]
#[dds_type(crate_path = "int2dds")]
struct HelloWorldType {
    index: u32,
    message: String,
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

    let _reader = subscriber
        .create_datareader::<HelloWorldType>(
            &topic,
            DataReaderQos::default(),
            None,
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
