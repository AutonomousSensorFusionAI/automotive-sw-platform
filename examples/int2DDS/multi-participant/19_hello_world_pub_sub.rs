use std::sync::{
    mpsc::{sync_channel, SyncSender},
    Arc,
};

use int2dds::{
    common::instance_handle::InstanceHandle,
    core::time::Duration,
    domain::{domain_participant_factory::DomainParticipantFactory, qos::DomainParticipantQos},
    infrastructure::{status::StatusMask, wait_set::WaitSet},
    publication::{
        data_writer_listener::DataWriterListener,
        qos::{DataWriterQos, PublisherQos},
    },
    subscription::{
        data_reader_listener::DataReaderListener,
        qos::{DataReaderQos, SubscriberQos},
        sample_info::{InstanceStateKind, SampleStateKind, ViewStateKind},
    },
    topic::{qos::TopicQos, type_support::DdsType},
};

#[derive(DdsType, speedy::Writable, speedy::Readable)]
#[dds_type(crate_path = "int2dds")]
pub struct HelloWorld {
    pub index: u32,
    pub message: String,
}

struct SubListener {
    sender: SyncSender<bool>,
}

impl DataReaderListener for SubListener {
    type Foo = HelloWorld;
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
        // Send counter signal whenever data is available
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

struct PubListener;

impl DataWriterListener for PubListener {
    type Foo = HelloWorld;
    fn on_publication_matched(
        &self,
        _writer: &int2dds::publication::data_writer::DataWriter<Self::Foo>,
        status: &int2dds::infrastructure::status::PublicationMatchedStatus,
    ) {
        if status.current_count() > 0 {
            println!("Subscriber matched! Sending start signal...");
        } else {
            println!("No subscribers. Sending stop signal...");
        }
    }
}

fn main() {
    let _ = env_logger::builder().filter_level(log::LevelFilter::Info).try_init();

    let domain_id = 19;
    let factory = DomainParticipantFactory::get_instance();
    let pub_participant = factory
        .create_participant(domain_id, DomainParticipantQos::default(), None, StatusMask::default())
        .unwrap();

    let topic = pub_participant
        .create_topic::<HelloWorld>(
            "hello_world",
            "HelloWorldType",
            TopicQos::default(),
            None,
            StatusMask::default(),
        )
        .unwrap();

    let publisher = pub_participant
        .create_publisher(PublisherQos::default(), None, StatusMask::default())
        .unwrap();
    let listener = PubListener;
    let writer = publisher
        .create_datawriter::<HelloWorld>(
            &topic,
            DataWriterQos::default(),
            Some(Arc::new(listener)),
            StatusMask::default(),
        )
        .unwrap();

    let sub_participant = factory
        .create_participant(domain_id, DomainParticipantQos::default(), None, StatusMask::default())
        .unwrap();

    let topic = sub_participant
        .create_topic::<HelloWorld>(
            "hello_world",
            "HelloWorldType",
            TopicQos::default(),
            None,
            StatusMask::default(),
        )
        .unwrap();

    let subscriber = sub_participant
        .create_subscriber(SubscriberQos::default(), None, StatusMask::default())
        .unwrap();
    let reader_qos = DataReaderQos::default();
    let (sub_sender, _sub_receiver) = sync_channel(0);
    let read_listener = SubListener { sender: sub_sender };
    let _data_reader = subscriber
        .create_datareader::<HelloWorld>(
            &topic,
            reader_qos,
            Some(Arc::new(read_listener)),
            StatusMask::default(),
        )
        .unwrap();

    let mut condition = writer.get_statuscondition().unwrap().clone();
    condition.set_enabled_statuses(StatusMask::PUBLICATION_MATCHED).unwrap();
    let wait_set = WaitSet::new();
    wait_set.attach_condition(condition).unwrap();
    wait_set.wait(Duration::infinite()).unwrap();
    writer.get_publication_matched_status().unwrap();

    let mut i = 0;
    loop {
        let data1 = HelloWorld { index: i, message: "HelloWorld".to_string() };
        writer.write(&data1, InstanceHandle::NIL).unwrap();
        std::thread::sleep(std::time::Duration::from_secs(1));
        i += 1;
    }
}
