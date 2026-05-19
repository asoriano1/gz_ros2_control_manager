cmake_minimum_required(VERSION 3.16)
project(gz_ros2_control_manager CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---- ROS 2 ----
find_package(rclcpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ---- Gazebo Harmonic ----
find_package(gz-gui8 REQUIRED)

# ---- Qt 5 ----
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Concurrent Gui Quick QuickControls2)

# ===================================================================
# Plugin target
# Class:   Ros2ControlManagerGui  →  libRos2ControlManagerGui.so
# QML res: :/Ros2ControlManagerGui/Ros2ControlManagerGui.qml
# ===================================================================
set(PLUGIN_NAME Ros2ControlManagerGui)

set(PLUGIN_SOURCES
  src/Ros2ControlManagerGui.cc
  src/Ros2ControlDiscovery.cc
  src/ControllerManagerClient.cc
)

set(PLUGIN_MOC_HEADERS
  include/gz_ros2_control_manager/Ros2ControlManagerGui.hh
)

qt5_add_resources(PLUGIN_RCC Ros2ControlManagerGui.qrc)

add_library(${PLUGIN_NAME} SHARED
  ${PLUGIN_SOURCES}
  ${PLUGIN_MOC_HEADERS}
  ${PLUGIN_RCC}
)

target_include_directories(${PLUGIN_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PLUGIN_NAME}
  PRIVATE
    gz-gui8::gz-gui8
    Qt5::Core
    Qt5::Concurrent
    Qt5::Gui
    Qt5::Quick
    Qt5::QuickControls2
)

ament_target_dependencies(${PLUGIN_NAME}
  PUBLIC
    rclcpp
    controller_manager_msgs
    rcl_interfaces
)

# ---- Install ----
install(TARGETS ${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY gui/
  DESTINATION share/${PROJECT_NAME}/gui
  FILES_MATCHING PATTERN "*.qml"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf.xacro" PATTERN "*.xacro"
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.sdf"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.py"
)

# Resolve gz_ros2_control's lib dir so the env-hook can set
# GZ_SIM_SYSTEM_PLUGIN_PATH automatically on workspace source.
find_package(gz_ros2_control QUIET)
if(gz_ros2_control_FOUND AND DEFINED gz_ros2_control_DIR)
  # gz_ros2_control_DIR is <prefix>/share/gz_ros2_control/cmake.
  # Walk up three levels: cmake → gz_ros2_control → share → <prefix>.
  get_filename_component(_gz_ctrl_d1 "${gz_ros2_control_DIR}" DIRECTORY)
  get_filename_component(_gz_ctrl_d2 "${_gz_ctrl_d1}"         DIRECTORY)
  get_filename_component(_gz_ctrl_prefix "${_gz_ctrl_d2}"     DIRECTORY)
  set(GZ_ROS2_CONTROL_LIB "${_gz_ctrl_prefix}/lib")
  message(STATUS "gz_ros2_control lib dir for env-hook: ${GZ_ROS2_CONTROL_LIB}")
else()
  set(GZ_ROS2_CONTROL_LIB "")
  message(STATUS "gz_ros2_control not found; GZ_SIM_SYSTEM_PLUGIN_PATH not set via env-hook")
endif()

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz_ros2_control_manager.sh.in"
)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ros2_control_discovery
    test/test_ros2_control_discovery.cpp
    src/Ros2ControlDiscovery.cc
  )
  target_include_directories(test_ros2_control_discovery
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_ros2_control_discovery
    rclcpp
    controller_manager_msgs
  )

  # Pure-logic tests over the static helpers in ControllerManagerClient
  # (actionFor + buildSwitchRequest).  Compile only the helpers themselves;
  # we don't need the rclcpp graph code paths in this test binary.
  ament_add_gtest(test_controller_action
    test/test_controller_action.cpp
    src/ControllerManagerClient.cc
  )
  target_include_directories(test_controller_action
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_controller_action
    rclcpp
    controller_manager_msgs
    rcl_interfaces
  )
endif()

ament_package()
