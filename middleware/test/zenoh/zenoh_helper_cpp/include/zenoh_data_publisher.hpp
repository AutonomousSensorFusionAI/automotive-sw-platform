#ifndef ZENOH_DATA_PUBLISHER_HPP
#define ZENOH_DATA_PUBLISHER_HPP

#include <string>
#include <memory>
#include "zenoh.hxx"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;
using namespace zenoh;

class IntZenohPub
{
public:
  IntZenohPub();
  
  void init_zenoh_session(const string *configfile, const string &key);
  void set_session(shared_ptr<Session> opened_session);
  void set_key(const string &key);
  void set_key(const string &key, PublisherOptions &qos);

  void zenoh_publisher(BytesView &payload);
  void zenoh_publisher(json &payload);
  void zenoh_publisher(int payload);
  void zenoh_publisher(string &payload);
  void zenoh_publisher(double &payload);
  void destroy_zenoh();

private:
  shared_ptr<zenohc::Session> session;
  optional<zenohc::Publisher> zenoh_pub;
};

#endif // ZENOH_DATA_PUBLISHER_HPP