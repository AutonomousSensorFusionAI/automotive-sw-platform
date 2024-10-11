#include "zenoh.hxx"
#include <iostream>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;
using namespace zenoh;

class IntZenohSub
{
public:
  IntZenohSub()
  {
    callback_handler = nullptr;
  }
  
  void init_zenoh_session(const string *configfile)
  {
    try
    {
      Config config;
      if (configfile)
      {
        config = expect(config_from_file(configfile->c_str()));
      }
      session = make_shared<Session>(expect<Session>(open(move(config))));
    }
    catch (ErrorMessage e)
    {
      cout << "Error: " << e.as_string_view() << endl;
    }
  }

  void set_session(shared_ptr<Session> opened_session)
  {
    session = opened_session;
  }

  void set_handler(const function<void(const string_view&, const string_view&)>& handler) {
    callback_handler = handler;
  }

  void zenoh_callback(const Sample &sample)
  {
    string_view key_expr = sample.get_keyexpr().as_string_view();
    string_view payload = sample.get_payload().as_string_view();
    // cout << "Received: " << key_expr << ": " << payload << endl;

    if (callback_handler)
    {
      callback_handler(key_expr, payload);
    }
  }

  void zenoh_subscriber(const string &key)
  {
    zenoh_sub = expect<Subscriber>(session->declare_subscriber(key.c_str(), bind(&IntZenohSub::zenoh_callback, this, placeholders::_1)));
  }

private:
  shared_ptr<zenohc::Session> session;
  optional<zenohc::Subscriber> zenoh_sub;
  function<void(const string_view&, const string_view&)> callback_handler;
};

void my_callback(const string_view &key_expr, const string_view &payload)
{
  json data = json::parse(payload);
  auto it = data.begin();
  cout << "Hi I also got " << key_expr << ": " << it.key() << endl;
}

int main(int argc, char **argv)
{
  const string *configfile = nullptr;
  const string key = "topic/**";
  // const string key = "demo/**";

  IntZenohSub subscriber;
  subscriber.init_zenoh_session(configfile);
  subscriber.set_handler(my_callback);
  subscriber.zenoh_subscriber(key);
  
  char c = getchar();
}