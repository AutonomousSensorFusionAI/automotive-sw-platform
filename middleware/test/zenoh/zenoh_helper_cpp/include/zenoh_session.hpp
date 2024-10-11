#ifndef ZENOH_SESSION_HPP
#define ZENOH_SESSION_HPP

#include <memory>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>
#include "zenoh.hxx"

using namespace std;
using namespace zenoh;


class ZenohSession {
public:
  static ZenohSession& get_instance();

  shared_ptr<Session> get_global_session();
  shared_ptr<Session> get_local_session();

  // 복사 및 이동 생성자/할당 연산자 삭제
  ZenohSession(const ZenohSession&) = delete;
  ZenohSession& operator=(const ZenohSession&) = delete;
  ZenohSession(ZenohSession&&) = delete;
  ZenohSession& operator=(ZenohSession&&) = delete;

private:
  ZenohSession();
  ~ZenohSession();

  Config get_shm_param();
  Config init_config(const string& config_path);
  void init_global_session();
  void init_local_session();

  Config global_config;
  Config local_config;
  shared_ptr<Session> global_session;
  shared_ptr<Session> local_session;

  static ZenohSession instance;
};

#endif // ZENOH_SESSION_HPP