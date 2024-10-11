cmake_minimum_required(VERSION 3.8)
project(test_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(image_transport REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Test.msg"
  DEPENDENCIES std_msgs
#   ADD_LINTER_TESTS
)

add_executable(pub src/publisher_member_function.cpp)
ament_target_dependencies(pub
  rclcpp)
add_executable(pub_image src/publisher_member_function_image.cpp)
ament_target_dependencies(pub_image
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV)


add_executable(sub src/subscriber_member_function.cpp)
ament_target_dependencies(sub
  rclcpp std_msgs)

add_executable(sub_image src/subscriber_member_function_image.cpp)
ament_target_dependencies(sub_image
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV)

add_executable(image_ping src/image_ping.cpp)
ament_target_dependencies(image_ping
  rclcpp
  sensor_msgs
)
add_executable(image_pong src/image_pong.cpp)
ament_target_dependencies(image_pong
  rclcpp
  sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(pub "${cpp_typesupport_target}")

target_link_libraries(pub_image
  "${cpp_typesupport_target}"
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_imgcodecs)


target_link_libraries(sub "${cpp_typesupport_target}")

target_link_libraries(sub_image
  "${cpp_typesupport_target}"
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_imgcodecs)


target_link_libraries(image_ping "${cpp_typesupport_target}")
target_link_libraries(image_pong "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  pub
  sub
  pub_image
  sub_image
  image_ping
  image_pong
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
