#include <iostream>
#include "include/zenoh_data_publisher.hpp"


IntZenohPub::IntZenohPub()
{

}

void IntZenohPub::init_zenoh_session(const string *configfile, const string &key)
{
  try
  {
    Config config;
    if (configfile)
    {
      config = expect(config_from_file(configfile->c_str()));
    }
    session = make_shared<Session>(expect<Session>(open(move(config))));
    zenoh_pub = expect<Publisher>(session->declare_publisher(key.c_str()));
  }
  catch (ErrorMessage e)
  {
    cout << "Error: " << e.as_string_view() << endl;
  }
}

void IntZenohPub::set_session(shared_ptr<Session> opened_session) {
  session = opened_session;
}

void IntZenohPub::set_key(const string &key) {
  // cout << "Declaring Publisher on '" << key << "'..." << endl;
  zenoh_pub = expect<Publisher>(session->declare_publisher(key.c_str()));
}

void IntZenohPub::set_key(const string &key, PublisherOptions &qos) {
  // cout << "Declaring Publisher on '" << key << "'..." << endl;
  zenoh_pub = expect<Publisher>(session->declare_publisher(key.c_str(), qos));
}

void IntZenohPub::zenoh_publisher(BytesView &payload)
{
  zenoh_pub->put(payload);
}

void IntZenohPub::zenoh_publisher(json &payload)
{
  string payload_str = payload.dump();
  const char *data = payload_str.c_str();
  zenoh_pub->put(data);
}

void IntZenohPub::zenoh_publisher(int payload)
{
  // zenoh_pub->put(payload);
  string data = to_string(payload);
  zenoh_pub->put(data);
}

void IntZenohPub::zenoh_publisher(string &payload)
{
  zenoh_pub->put(payload);
}

void IntZenohPub::zenoh_publisher(double &payload)
{
  string data = to_string(payload);
  zenoh_pub->put(data);
}

void IntZenohPub::destroy_zenoh() {
  zenoh_pub.reset();  // Publisher 객체를 소멸시키고 optional을 비웁니다.
}