cmake_minimum_required(VERSION 3.8)
project(gentact_ros_tools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Install Python package so modules can be imported
ament_python_install_package(${PROJECT_NAME})

# Preserve "ros2 run" names for Python nodes by generating wrapper scripts
set(PY_CONSOLE_ENTRIES
  calibration_direct_pub gentact_ros_tools.calibration_direct_pub:main
  sensor_publisher gentact_ros_tools.sensor_publisher:main
  sensor_subscriber gentact_ros_tools.sensor_subscriber:main
  capacitive_pcl gentact_ros_tools.capacitive_pcl:main
  tuner gentact_ros_tools.tuner:main
  sensor_tracking_pub gentact_ros_tools.sensor_tracking_pub:main
  udp_sensor_publisher gentact_ros_tools.udp_sensor_publisher:main
  processor gentact_ros_tools.processor:main
  ik_solver gentact_ros_tools.ik_solver:main
  training_data_processor gentact_ros_tools.training_data_processor:main
  ee_prediction_model_mlp gentact_ros_tools.ee_prediction_model_mlp:main
  ee_prediction_model_mamba gentact_ros_tools.ee_prediction_model_mamba:main
  ee_prediction_verifier gentact_ros_tools.ee_prediction_verifier:main
  panda2fr3 gentact_ros_tools.panda2fr3:main
  fake_obj_pub gentact_ros_tools.fake_obj_pub:main
  joint_state_test_pub gentact_ros_tools.joint_state_test_pub:main
  franky_relay gentact_ros_tools.franky_relay:main
  franky_xbox gentact_ros_tools.franky_xbox:main
  closest_obstacle gentact_ros_tools.closest_obstacle:main
  tof_pub_pc gentact_ros_tools.tof_pub_pc:main
  franka_ros2_controller gentact_ros_tools.franka_ros2_controller:main
  joint_states_monitor gentact_ros_tools.joint_states_monitor:main
  test_talker gentact_ros_tools.test_talker:main
)

set(GENERATED_PY_SCRIPTS)
math(EXPR ENTRY_COUNT "${PY_CONSOLE_ENTRIES_LENGTH} - 1" OUTPUT_FORMAT DECIMAL) # fallback if property not set
# CMake doesn't auto-define *_LENGTH for custom lists; compute manually
list(LENGTH PY_CONSOLE_ENTRIES _PY_LEN)
if(NOT (_PY_LEN EQUAL 0))
  math(EXPR _LAST_INDEX "${_PY_LEN} - 1")
  foreach(_i RANGE 0 ${_LAST_INDEX} 2)
    list(GET PY_CONSOLE_ENTRIES ${_i} _name)
    math(EXPR _j "${_i} + 1")
    list(GET PY_CONSOLE_ENTRIES ${_j} _entry)
    # split "pkg.mod:func"
    string(REPLACE ":" ";" _parts ${_entry})
    list(GET _parts 0 _module)
    list(GET _parts 1 _func)
    set(_wrapper "${CMAKE_CURRENT_BINARY_DIR}/${_name}")
    file(WRITE ${_wrapper}
"#!/usr/bin/env python3\nfrom ${_module} import ${_func} as _main\nif __name__ == '__main__':\n    _main()\n")
    # ensure executable
    file(CHMOD ${_wrapper} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
    list(APPEND GENERATED_PY_SCRIPTS ${_wrapper})
  endforeach()
endif()

if(GENERATED_PY_SCRIPTS)
  install(PROGRAMS ${GENERATED_PY_SCRIPTS} DESTINATION lib/${PROJECT_NAME})
endif()

# C++ nodes
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

add_executable(udp_grid_listener ${SRC_DIR}/udp_grid_listener.cpp)
ament_target_dependencies(udp_grid_listener rclcpp sensor_msgs Boost std_msgs)
target_link_libraries(udp_grid_listener Boost::system)

add_executable(udp_grid_listener_array ${SRC_DIR}/udp_grid_listener_array.cpp)
ament_target_dependencies(udp_grid_listener_array rclcpp sensor_msgs Boost std_msgs)
target_link_libraries(udp_grid_listener_array Boost::system)

add_executable(udp_grid_pcl ${SRC_DIR}/udp_grid_pcl.cpp)
ament_target_dependencies(udp_grid_pcl rclcpp sensor_msgs Boost std_msgs)
target_link_libraries(udp_grid_pcl Boost::system)

install(TARGETS
  udp_grid_listener
  udp_grid_listener_array
  udp_grid_pcl
  DESTINATION lib/${PROJECT_NAME}
)

# Ament resource index
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/resource/${PROJECT_NAME})
  install(FILES resource/${PROJECT_NAME}
    DESTINATION share/ament_index/resource_index/packages)
endif()

# Package manifest
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Install assets
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
  install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
endif()

ament_package()
