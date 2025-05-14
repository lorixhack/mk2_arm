cmake_minimum_required(VERSION 3.22)
project(mk2_arm)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Aggiungi queste dipendenze per l'eseguibile C++
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

# Aggiungi l'eseguibile C++
add_executable(servo_keyboard_input
  src/servo_keyboard_input.cpp  # Percorso del file sorgente
)

# Linka le dipendenze
target_link_libraries(servo_keyboard_input
  ${rclcpp_LIBRARIES}
)

# Specifica le dipendenze AMENT
ament_target_dependencies(servo_keyboard_input
  rclcpp
  control_msgs
  geometry_msgs
  moveit_msgs
)

# Installa l'eseguibile
install(TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install setup assistant files
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
