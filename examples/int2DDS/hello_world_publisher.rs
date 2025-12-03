use std::sync::Arc;

use int2dds::{
    common::instance_handle::InstanceHandle,
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
use speedy::{Readable, Writable};

#[derive(DdsType, Readable, Writable)]
#[dds_type(crate_path = "int2dds")]
struct HelloWorldType {
    index: u32,
    message: String,
}

struct WriterListener;

impl DataWriterListener for WriterListener {
    type Foo = HelloWorldType;
    fn on_publication_matched(
        &self,
        _writer: &int2dds::publication::data_writer::DataWriter<Self::Foo>,
        status: &int2dds::infrastructure::status::PublicationMatchedStatus,
    ) {
        let change = status.current_count_change();
        let count = status.current_count();

        if change > 0 {
            println!("Subscriber matched! Current: {}", count);
        } else if change < 0 {
            println!("Subscriber unmatched! Current: {}", count);
        }

        if count == 0 {
            println!("No subscribers remaining.");
        }
    }
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();
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

    let publisher = participant
        .create_publisher(PublisherQos::default(), None, StatusMask::default())
        .unwrap();

    let writer_qos = DataWriterQos {
        reliability: ReliabilityQosPolicy {
            kind: ReliabilityQosPolicyKind::Reliable,
            max_blocking_time: Duration {
                sec: 0,
                nanosec: 100_000_000,
            },
        },
        ..Default::default()
    };
    let listener = WriterListener;
    let writer = publisher
        .create_datawriter::<HelloWorldType>(
            &topic,
            writer_qos,
            Some(Arc::new(listener)),
            StatusMask::default(),
        )
        .unwrap();

    println!(
        "[publisher INFO] domain_id: {:?}, hostname: {:?}",
        domain_id,
        hostname::get().unwrap()
    );

    let mut condition = writer.get_statuscondition().unwrap().clone();
    condition
        .set_enabled_statuses(StatusMask::PUBLICATION_MATCHED)
        .unwrap();
    let wait_set = WaitSet::new();
    wait_set.attach_condition(condition).unwrap();
    wait_set.wait(Duration::infinite()).unwrap();

    let mut i = 0;
    loop {
        let data = HelloWorldType {
            index: i,
            message: "HelloWorld".to_string(),
        };
        writer.write(&data, InstanceHandle::NIL).unwrap();
        println!("Published {:?}", data);
        std::thread::sleep(std::time::Duration::from_millis(1000));
        i += 1;
    }
}
