use std::sync::Arc;

use int2dds::{
    common::{
        env::{set_console_log_level, set_log_type},
        instance_handle::InstanceHandle,
        log::{LogLevel, LogType},
    },
    core::time::Duration,
    domain::{domain_participant_factory::DomainParticipantFactory, qos::DomainParticipantQos},
    infrastructure::{
        qos_policy::{ReliabilityQosPolicy, ReliabilityQosPolicyKind},
        status::StatusMask,
        wait_set::WaitSet,
    },
    publication::{
        data_writer_listener::DataWriterListener,
        qos::{DataWriterQos, PublisherQos},
    },
    topic::{qos::TopicQos, type_support::DdsType},
};
use log::info;
use speedy::{Readable, Writable};

#[derive(DdsType, Readable, Writable)]
#[dds_type(crate_path = "int2dds")]
struct HelloWorldType {
    index: u32,
    message: String,
}

struct PubListener;

impl DataWriterListener for PubListener {
    type Foo = HelloWorldType;
    fn on_publication_matched(
        &self,
        _writer: &int2dds::publication::data_writer::DataWriter<Self::Foo>,
        status: &int2dds::infrastructure::status::PublicationMatchedStatus,
    ) {
        if status.current_count() > 0 {
            info!("Subscriber matched! Sending start signal...");
        } else {
            info!("No subscribers. Sending stop signal...");
        }
    }
}

//tmux attach -t int2multirr
//cargo run --package int2dds --example 50_hello_world_2_participant_subscriber
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

    let publisher_50 = participant_50
        .create_publisher(PublisherQos::default(), None, StatusMask::default())
        .unwrap();
    let publisher_51 = participant_51
        .create_publisher(PublisherQos::default(), None, StatusMask::default())
        .unwrap();

    // let writer_qos = DataWriterQos::default();
    let listener_50: PubListener = PubListener;
    let listener_51 = PubListener;
    let writer_qos = DataWriterQos {
        reliability: ReliabilityQosPolicy {
            kind: ReliabilityQosPolicyKind::Reliable,
            max_blocking_time: Duration { sec: 0, nanosec: 100_000_000 },
        },
        ..Default::default()
    };
    let writer_50 = publisher_50
        .create_datawriter::<HelloWorldType>(
            &topic_50,
            writer_qos.clone(),
            Some(Arc::new(listener_50)),
            StatusMask::default(),
        )
        .unwrap();
    let writer_51 = publisher_51
        .create_datawriter::<HelloWorldType>(
            &topic_51,
            writer_qos.clone(),
            Some(Arc::new(listener_51)),
            StatusMask::default(),
        )
        .unwrap();
    info!(
        "[publisher INFO] domain_id: {:?}, hostname: {:?}",
        domain_id_50,
        hostname::get().unwrap()
    );
    info!(
        "[publisher INFO] domain_id: {:?}, hostname: {:?}",
        domain_id_51,
        hostname::get().unwrap()
    );
    let mut condition_50 = writer_50.get_statuscondition().unwrap().clone();
    condition_50.set_enabled_statuses(StatusMask::PUBLICATION_MATCHED).unwrap();
    let mut condition_51 = writer_51.get_statuscondition().unwrap().clone();
    condition_51.set_enabled_statuses(StatusMask::PUBLICATION_MATCHED).unwrap();
    let wait_set = WaitSet::new();
    wait_set.attach_condition(condition_50).unwrap();
    wait_set.attach_condition(condition_51).unwrap();
    wait_set.wait(Duration::infinite()).unwrap();
    writer_50.get_publication_matched_status().unwrap();
    writer_51.get_publication_matched_status().unwrap();

    let mut i = 0;
    loop {
        let data_50 = HelloWorldType {
            index: i,
            message: format!("[{:?}]HelloWorld_reliable_d50", hostname::get().unwrap()),
        };
        writer_50.write(&data_50, InstanceHandle::NIL).unwrap();
        info!("Published {:?}", data_50);

        let data_51 = HelloWorldType {
            index: i,
            message: format!("[{:?}]HelloWorld_reliable_d51", hostname::get().unwrap()),
        };
        writer_51.write(&data_51, InstanceHandle::NIL).unwrap();
        info!("Published {:?}", data_51);
        std::thread::sleep(std::time::Duration::from_millis(1000));
        i += 1;
    }
}
