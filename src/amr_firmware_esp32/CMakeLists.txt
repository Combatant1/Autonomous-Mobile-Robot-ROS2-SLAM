cmake_minimum_required(VERSION 3.8)
project(amr_firmware_esp32)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


pkg_check_modules(SERIAL libserial)

add_library(${PROJECT_NAME} SHARED
  src/amr_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${SERIAL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${SERIAL_LDFLAGS})
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rclcpp
)

add_executable(simple_serial_transmitter src/simple_serial_transmitter.cpp)
ament_target_dependencies(simple_serial_transmitter rclcpp std_msgs)
target_link_libraries(simple_serial_transmitter "${SERIAL_LDFLAGS}")
target_include_directories(simple_serial_transmitter PUBLIC "${SERIAL_INCLUDE_DIRS}")

add_executable(simple_serial_receiver src/simple_serial_receiver.cpp)
ament_target_dependencies(simple_serial_receiver rclcpp std_msgs)
target_link_libraries(simple_serial_receiver "${SERIAL_LDFLAGS}")
target_include_directories(simple_serial_receiver PUBLIC "${SERIAL_INCLUDE_DIRS}")

pluginlib_export_plugin_description_file(hardware_interface amr_interface.xml)

install(TARGETS
  simple_serial_transmitter
  simple_serial_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
  )

  install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib
  )

  install(
    DIRECTORY include
    DESTINATION include
  )

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rclcpp
)
ament_package()