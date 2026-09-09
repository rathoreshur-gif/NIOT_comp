cmake_minimum_required(VERSION 3.8)
project(auv_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)  # Added this dependency
find_package(rosidl_default_generators REQUIRED)

# Generate message and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Orientation.msg"
  "msg/AuvState.msg"
  "msg/DVLVel.msg"
  "msg/ThrusterForces.msg"
  "msg/BoundingBox.msg"
  "msg/GlobalForces.msg"
  "msg/Pose.msg"
  "msg/DVLBeam.msg"
  "msg/KinematicData.msg"
  "msg/PsData.msg"
  "msg/DVLBeamsArr.msg"
  "msg/NavStatus.msg"
  "msg/PwmData.msg"
  "msg/DVLOrient.msg"
  "msg/ObjectDetected.msg"
  "msg/ObjectDetections.msg"
  "msg/TaskDetected.msg"
  "msg/VisionObjects.msg"
  "msg/VisionObjectsConfidence.msg"
  "msg/StereoVisionFrame.msg"
  "msg/BBoxes.msg"
  "msg/MapTask.msg"
  "msg/MapTasks.msg"
  "msg/MapObject.msg"
  "msg/MapObjects.msg"
  "msg/ObjectConfidence.msg"
  "srv/AlignService.srv"
  "srv/PingerAcq.srv"
  "srv/PwmCap.srv"
  "srv/SetpointService.srv"
  "srv/GripperState.srv"
  "srv/ScanService.srv"
  "srv/TorpedoState.srv"
  "srv/MarkerDropperState.srv"
  "srv/SetDetector.srv"
  "srv/SetTorpedo.srv"
  "srv/VisionModel.srv"
  "srv/MapService.srv"
  "srv/BuilderService.srv"
  "srv/AllocatorStrength.srv"
  "srv/ManualAllocatorForce.srv"
  "action/Interrupt.action"
  "msg/CurrentNavId.msg"
  "msg/BMP.msg"
  "msg/MapGui.msg"
  "msg/RGBDFrame.msg"
  "msg/RunState.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs unique_identifier_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
