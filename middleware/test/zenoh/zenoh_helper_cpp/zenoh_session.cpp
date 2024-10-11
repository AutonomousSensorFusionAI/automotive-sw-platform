#include "zenoh_session.hpp"

const string LOCAL_CONFIG_PATH = "";
const string GLOBAL_CONFIG_PATH= "/root/automotive-sw-platform/middleware/test/zenoh/zenoh_helper_cpp/zenoh.json5";
// This for Zero Copy
const bool USING_SHM = true;
const string Z_LOCAL_CONFIG_PATH = "/root/automotive-sw-platform/middleware/test/zenoh/zenoh_helper_cpp/zero_copy.json5";

// 정적 인스턴스 초기화
ZenohSession ZenohSession::instance;

ZenohSession& ZenohSession::get_instance() {
  return instance;
}

shared_ptr<Session> ZenohSession::get_global_session() {
  if (!global_session) {
    throw runtime_error("Global Zenoh session not initialized");
  }
  return global_session;
}

shared_ptr<Session> ZenohSession::get_local_session() {
  if (!local_session) {
    throw runtime_error("Local Zenoh session not initialized");
  }
  return local_session;
}

ZenohSession::ZenohSession():
  local_config(get_shm_param()),
  global_config(init_config(GLOBAL_CONFIG_PATH))
{
  init_global_session();
  init_local_session();
}

ZenohSession::~ZenohSession() {
  global_session.reset();
  local_session.reset();
}

Config ZenohSession::get_shm_param()
{
  if (!USING_SHM)
  {
    return init_config(LOCAL_CONFIG_PATH);
  }
  else
  {
    cout << "using zero-copy (shared-memory)" << endl;
    return init_config(Z_LOCAL_CONFIG_PATH);
  }
}

Config ZenohSession::init_config(const string& config_path)
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

void ZenohSession::init_global_session()
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

void ZenohSession::init_local_session()
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