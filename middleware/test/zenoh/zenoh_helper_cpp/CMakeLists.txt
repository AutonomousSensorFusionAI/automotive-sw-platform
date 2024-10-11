cmake_minimum_required(VERSION 3.8)
project(test_pubsub_zenoh_cpp)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add include directory
include_directories(include)

include(FetchContent)

FetchContent_Declare(c_backend GIT_REPOSITORY "https://github.com/eclipse-zenoh/zenoh-c" GIT_TAG main)
FetchContent_MakeAvailable(c_backend)
FetchContent_Declare(cpp_wrapper GIT_REPOSITORY "https://github.com/eclipse-zenoh/zenoh-cpp" GIT_TAG main)
FetchContent_MakeAvailable(cpp_wrapper)
FetchContent_Declare(json GIT_REPOSITORY "https://github.com/nlohmann/json.git" GIT_TAG develop)
FetchContent_MakeAvailable(json)

# find dependencies
# find_package(zenohc)
# find_package(zenohcxx)

add_library(zenoh_session STATIC zenoh_session.cpp)
target_link_libraries(zenoh_session PUBLIC zenohcxx::zenohc::lib)
set_property(TARGET zenoh_session PROPERTY LANGUAGE CXX)
set_property(TARGET zenoh_session PROPERTY CXX_STANDARD 17)

add_library(zenoh_data_publisher STATIC zenoh_data_publisher.cpp)
target_link_libraries(zenoh_data_publisher PUBLIC zenohcxx::zenohc::lib PRIVATE nlohmann_json::nlohmann_json)
set_property(TARGET zenoh_data_publisher PROPERTY LANGUAGE CXX)
set_property(TARGET zenoh_data_publisher PROPERTY CXX_STANDARD 17)

add_library(zenoh_data_subscriber STATIC zenoh_data_subscriber.cpp)
target_link_libraries(zenoh_data_subscriber PUBLIC zenohcxx::zenohc::lib PRIVATE nlohmann_json::nlohmann_json)
set_property(TARGET zenoh_data_subscriber PROPERTY LANGUAGE CXX)
set_property(TARGET zenoh_data_subscriber PROPERTY CXX_STANDARD 17)