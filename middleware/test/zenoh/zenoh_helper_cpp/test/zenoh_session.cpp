#include <memory>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>
#include "zenoh.hxx"

using namespace std;
using namespace zenoh;

const string LOCAL_CONFIG_PATH = "";
const string GLOBAL_CONFIG_PATH= "/root/automotive-sw-platform/middleware/test/zenoh/zenoh_helper_cpp/zenoh.json5";

class ZenohSession {
public:
  static ZenohSession& get_instance() {
    return instance;
  }

  Session& get_global_session() {
    if (!global_session) {
      // init_global_session();
      throw runtime_error("Global Zenoh session not initialized");
    }
    return *global_session;
  }

  Session& get_local_session() {
    if (!local_session) {
      // init_local_session();
      throw runtime_error("Local Zenoh session not initialized");
    }
    return *local_session;
  }

  // 복사 및 이동 생성자/할당 연산자 삭제
  ZenohSession(const ZenohSession&) = delete;
  ZenohSession& operator=(const ZenohSession&) = delete;
  ZenohSession(ZenohSession&&) = delete;
  ZenohSession& operator=(ZenohSession&&) = delete;

private:
  ZenohSession():
    local_config(init_config(LOCAL_CONFIG_PATH)),
    global_config(init_config(GLOBAL_CONFIG_PATH))
  {
    init_global_session();
    init_local_session();
  }

  ~ZenohSession() {
    global_session.reset();
    local_session.reset();
  }

  Config init_config(const string& config_path)
  {
    if (config_path.empty())
    {
      return Config();  // 빈 설정 반환
    }
    else
    {
      try
      {
        return expect(config_from_file(config_path.c_str()));
      }
      catch (const exception& e)
      {
        cerr << "Error loading config from " << config_path << ": " << e.what() << endl;
        cerr << "Using default configuration." << endl;
        return Config();  // 에러 발생 시 빈 설정 반환
      }
    }
  }

  void init_global_session()
  {
    auto session_result = open(move(global_config));
    if (holds_alternative<Session>(session_result)) // sesion_result가 Session Type을 보유하고 있는지 확인(세션 생성 결과 확인)
    {
      global_session = make_unique<Session>(get<Session>(move(session_result)));
    }
    else
    {
      const auto& error = get<Value>(session_result);
      throw runtime_error("Failed to open Global Zenoh session: " + string(error.as_string_view()));
    }
  }

  void init_local_session()
  {
    auto session_result = open(move(local_config));
    if (holds_alternative<Session>(session_result)) // sesion_result가 Session Type을 보유하고 있는지 확인(세션 생성 결과 확인)
    {
      local_session = make_unique<Session>(get<Session>(move(session_result)));
    }
    else
    {
      const auto& error = get<Value>(session_result);
      throw runtime_error("Failed to open Local Zenoh session: " + string(error.as_string_view()));
    }
  }

  Config global_config;
  Config local_config;
  shared_ptr<Session> global_session;
  shared_ptr<Session> local_session;

  static ZenohSession instance;
};

// 정적 인스턴스 초기화
ZenohSession ZenohSession::instance;

void testPublishSubscribe() {
  auto& zenoh = ZenohSession::get_instance();

  try {
    // Global 세션을 사용한 발행자
    auto& global_session = zenoh.get_global_session();
    auto publisher_result = global_session.declare_publisher("test/global");
    
    if (holds_alternative<Publisher>(publisher_result)) {
      auto global_publisher = get<Publisher>(move(publisher_result));

      // Local 세션을 사용한 구독자
      // auto& local_session = zenoh.get_local_session();
      auto subscriber_result = global_session.declare_subscriber("test/global",
        [](const Sample& sample) {
          cout << "Received (Local): " << sample.get_payload().as_string_view() << endl;
      });

      if (holds_alternative<Subscriber>(subscriber_result)) {
        auto local_subscriber = get<Subscriber>(move(subscriber_result));

        // 메시지 발행
        string msg = "Hello, Zenoh!";
        global_publisher.put(msg);

        // 메시지 수신을 위한 대기
        this_thread::sleep_for(chrono::seconds(1));

      } else {
        throw runtime_error("Failed to declare subscriber");
      }
    } else {
      throw runtime_error("Failed to declare publisher");
    }

  } catch (const exception& e) {
    cerr << "Error in testPublishSubscribe: " << e.what() << endl;
  }
}

int main() {
  try {
    cout << "Starting Zenoh session test..." << endl;
    testPublishSubscribe();
    cout << "Zenoh session test completed." << endl;

    // Zenoh Session 명시적 정리
    auto& zenohInstance = ZenohSession::get_instance();
    zenohInstance.get_global_session().~Session();
    zenohInstance.get_local_session().~Session();
  } catch (const Value& e) {
    cerr << "Zenoh error: " << e.as_string_view() << endl;
    return 1;
  } catch (const exception& e) {
    cerr << "Error in main: " << e.what() << endl;
    return 1;
  } 

  return 0;
}