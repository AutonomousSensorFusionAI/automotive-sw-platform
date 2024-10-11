cmake_minimum_required(VERSION 3.8)
project(test_pubsub_zenoh_cpp)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

FetchContent_Declare(c_backend GIT_REPOSITORY "https://github.com/eclipse-zenoh/zenoh-c" GIT_TAG main)
FetchContent_MakeAvailable(c_backend)
FetchContent_Declare(cpp_wrapper GIT_REPOSITORY "https://github.com/eclipse-zenoh/zenoh-cpp" GIT_TAG main)
FetchContent_MakeAvailable(cpp_wrapper)
FetchContent_Declare(json GIT_REPOSITORY "https://github.com/nlohmann/json.git" GIT_TAG develop)
FetchContent_MakeAvailable(json)


# find dependencies
find_package(zenohc)
find_package(zenohcxx)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/../zenoh_helper_cpp
  ${CMAKE_BINARY_DIR}/zenoh_helper_cpp/build   # 빌드 결과물을 배치할 binary dir을 지정  
)

add_executable(publisher_string publisher_string.cpp)
target_link_libraries(publisher_string PUBLIC 
  zenohcxx::zenohc::lib
  nlohmann_json::nlohmann_json
  zenoh_data_publisher
  zenoh_data_subscriber
  zenoh_session
)
target_include_directories(publisher_string PRIVATE
  "../zenoh_helper_cpp/include"
)
set_property(TARGET publisher_string PROPERTY LANGUAGE CXX)
set_property(TARGET publisher_string PROPERTY CXX_STANDARD 17)


add_executable(subscriber_string subscriber_string.cpp)
target_link_libraries(subscriber_string PUBLIC 
  zenohcxx::zenohc::lib
  nlohmann_json::nlohmann_json
  zenoh_data_publisher
  zenoh_data_subscriber
  zenoh_session
)
target_include_directories(subscriber_string PRIVATE
  "../zenoh_helper_cpp/include"
)
set_property(TARGET subscriber_string PROPERTY LANGUAGE CXX)
set_property(TARGET subscriber_string PROPERTY CXX_STANDARD 17)

set(PROTO_SRCS ${CMAKE_SOURCE_DIR}/image_transmission/image_transmission.pb.cc)
set(PROTO_HDRS ${CMAKE_SOURCE_DIR}/image_transmission/image_transmission.pb.h)


add_executable(publisher_image publisher_image.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(publisher_image PUBLIC
  zenohcxx::zenohc::lib
  nlohmann_json::nlohmann_json # for zenoh_data_publisher, subscriber
  zenoh_data_publisher
  zenoh_data_subscriber
  zenoh_session
  ${OpenCV_LIBS}
  ${Protobuf_LIBRARIES}
)
target_include_directories(publisher_image PRIVATE
  "../zenoh_helper_cpp/include"
  ${OpenCV_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/image_transmission
)
set_property(TARGET publisher_image PROPERTY LANGUAGE CXX)
set_property(TARGET publisher_image PROPERTY CXX_STANDARD 17)