extern crate capnpc;

fn main () {
  capnpc::CompilerCommand::new()
    .output_path("src/")
    .src_prefix("messages/")
    .file("messages/log_message.capnp")
    .run().unwrap();
}