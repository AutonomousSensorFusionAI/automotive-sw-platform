use capnp::message::{Builder, ReaderOptions};
use capnp::serialize;
use chrono::offset::Utc;

pub mod test_message_capnp;

fn get_timestamp() -> String {
    let now: chrono::DateTime<Utc> = Utc::now();
    let now_format: String = now.format("%Y/%m/%d %T").to_string();
    now_format
}

pub fn serialize_msg(count: i32) -> Vec<u8> {
    let mut message = Builder::new_default();

    let mut test = message.init_root::<test_message_capnp::test_message::Builder>();
    test.set_count(count);
    test.set_timestamp(get_timestamp());

    let data = serialize::write_message_to_words(&message);
    data
}

pub fn deserialize_msg(data: &Vec<u8>) -> (i32, String) {
    let reader = serialize::read_message(data.as_slice(), ReaderOptions::new()).unwrap();

    let test = reader
        .get_root::<test_message_capnp::test_message::Reader>()
        .unwrap();

    let count = test.get_count();
    let timestamp = test.get_timestamp().unwrap().to_string().unwrap();
    (count, timestamp)
}
