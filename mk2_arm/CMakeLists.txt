cmake_minimum_required(VERSION 3.22)
project(mk2_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)


find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# install(
#  DIRECTORY scripts/
#  DESTINATION lib/${PROJECT_NAME}
#  PATTERN "*.py"
#)

install (PROGRAMS
  scripts/joint_publisher_serial.py
  DESTINATION lib/${PROJECT_NAME}
)

install (PROGRAMS
  scripts/joint_publisher_can.py
  DESTINATION lib/${PROJECT_NAME}
)

install (PROGRAMS
  scripts/can_sender.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(servo_keyboard_input
  src/servo_keyboard_input.cpp  # Percorso del file sorgente
)

target_link_libraries(servo_keyboard_input
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(servo_keyboard_input
  rclcpp
  control_msgs
  geometry_msgs
  moveit_msgs
)

install(TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
