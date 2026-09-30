# Install script for directory: /root/ros_ws/src/px4_wall_ceiling_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/ros_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_wall_ceiling_control/msg" TYPE FILE FILES
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/AttachmentControlCandidate.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/AttachmentMechanismCommand.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/AttachmentMechanismStatus.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/CeilingAttachmentStatus.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/OperatorCommand.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/SensorHealth.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/WallPerchStatus.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/BehaviorCommand.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/ControllerStatus.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/ControlSetpoint.msg"
    "/root/ros_ws/src/px4_wall_ceiling_control/msg/SupervisorState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_wall_ceiling_control/cmake" TYPE FILE FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/px4_wall_ceiling_control-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/ros_ws/devel/include/px4_wall_ceiling_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/root/ros_ws/devel/share/roseus/ros/px4_wall_ceiling_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/ros_ws/devel/share/common-lisp/ros/px4_wall_ceiling_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/root/ros_ws/devel/share/gennodejs/ros/px4_wall_ceiling_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/root/ros_ws/devel/lib/python3/dist-packages/px4_wall_ceiling_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/root/ros_ws/devel/lib/python3/dist-packages/px4_wall_ceiling_control" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/root/ros_ws/devel/lib/python3/dist-packages/px4_wall_ceiling_control" FILES_MATCHING REGEX "/root/ros_ws/devel/lib/python3/dist-packages/px4_wall_ceiling_control/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/px4_wall_ceiling_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_wall_ceiling_control/cmake" TYPE FILE FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/px4_wall_ceiling_control-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_wall_ceiling_control/cmake" TYPE FILE FILES
    "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/px4_wall_ceiling_controlConfig.cmake"
    "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/px4_wall_ceiling_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_wall_ceiling_control" TYPE FILE FILES "/root/ros_ws/src/px4_wall_ceiling_control/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/ceiling_motor_bench_output_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/ceiling_attachment_decision_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/wall_perch_decision_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/operator_gate_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/terminal_operator_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/flight_mode_manager_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/attachment_mechanism_output_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/attachment_output_mux_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/lidar_distance_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/sensor_range_manager_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/ceiling_attachment_output_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/mock_attachment_mechanism_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/mock_flight_inputs_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/ceiling_attachment_state_monitor.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/wall_perch_state_monitor.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/ceiling_controller_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/ceiling_fsm_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/flight_supervisor_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/setpoint_mux_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/wall_controller_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/px4_wall_ceiling_control" TYPE PROGRAM FILES "/root/ros_ws/build/px4_wall_ceiling_control/catkin_generated/installspace/wall_fsm_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_wall_ceiling_control" TYPE DIRECTORY FILES
    "/root/ros_ws/src/px4_wall_ceiling_control/config"
    "/root/ros_ws/src/px4_wall_ceiling_control/docs"
    "/root/ros_ws/src/px4_wall_ceiling_control/launch"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_wall_ceiling_control" TYPE FILE FILES "/root/ros_ws/src/px4_wall_ceiling_control/README.md")
endif()

