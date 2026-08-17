cmake_minimum_required(VERSION 3.10)
project(auv_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Without this AUTOMOC silently skips .hh headers, so the Q_OBJECT in
# Scoreboard.hh never gets a meta-object and the plugin builds fine but has no
# properties or signals at runtime. gz-gui's own plugins use .hh for the same
# reason, and set the same policy.
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)

# gz-gui plugins are Qt objects with QML shipped inside the library as a Qt
# resource, so both the meta-object compiler and the resource compiler run here.
set(CMAKE_AUTOMOC ON)
qt5_add_resources(SCOREBOARD_RCC src/Scoreboard.qrc)

add_library(Scoreboard SHARED src/Scoreboard.cc ${SCOREBOARD_RCC})
target_include_directories(Scoreboard PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(Scoreboard PRIVATE
  gz-common::core
  gz-gui::core
  gz-msgs::core
  gz-plugin::register
  gz-transport::core
  Qt5::Core
  Qt5::Quick
  Qt5::QuickControls2
)

# Gazebo finds GUI plugins on GZ_GUI_PLUGIN_PATH; the launch file points that
# at this directory.
# Named AuvTeleop, not Teleop: gz-gui ships its own libTeleop.so and a name
# clash on the plugin search path is decided by search order, not by us.
qt5_add_resources(TELEOP_RCC src/AuvTeleop.qrc)
add_library(AuvTeleop SHARED src/AuvTeleop.cc ${TELEOP_RCC})
target_include_directories(AuvTeleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(AuvTeleop PRIVATE
  gz-common::core
  gz-gui::core
  gz-msgs::core
  gz-plugin::register
  gz-transport::core
  Qt5::Core
  Qt5::Quick
  Qt5::QuickControls2
)

install(TARGETS Scoreboard AuvTeleop DESTINATION lib/${PROJECT_NAME})

ament_package()
