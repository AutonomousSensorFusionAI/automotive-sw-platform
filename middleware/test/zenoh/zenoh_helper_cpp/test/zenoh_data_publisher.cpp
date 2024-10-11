#include "zenoh.hxx"
#include <iostream>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;
using namespace zenoh;

class IntZenohPub
{
public:
  IntZenohPub()
  {

  }
  
  void init_zenoh_session(const string *configfile, const string &key)
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

  void set_session(shared_ptr<Session> opened_session) {
    session = opened_session;
  }

  void set_key(const string &key) {
    cout << "Declaring Publisher on '" << key << "'..." << endl;
    zenoh_pub = expect<Publisher>(session->declare_publisher(key.c_str()));
  }

  void zenoh_publisher(json &payload)
  {
    string payload_str = payload.dump();
    const char *data = payload_str.c_str();
    zenoh_pub->put(data);
  }

  void zenoh_publisher(int payload)
  {
    // zenoh_pub->put(payload);
    string data = to_string(payload);
    zenoh_pub->put(data);
  }

  void zenoh_publisher(string &payload)
  {
    zenoh_pub->put(payload);
  }

  void zenoh_publisher(double &payload)
  {
    string data = to_string(payload);
    zenoh_pub->put(data);
  }

private:
  shared_ptr<zenohc::Session> session;
  optional<zenohc::Publisher> zenoh_pub;
};

int main(int argc, char **argv)
{
  const string *configfile = nullptr;
  const string key = "demo";

  IntZenohPub publisher;
  publisher.init_zenoh_session(configfile, key);
  int payload = 1;
  publisher.zenoh_publisher(payload);
}