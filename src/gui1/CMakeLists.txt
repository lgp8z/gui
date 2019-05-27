cmake_minimum_required(VERSION 2.8.3)
project(gui1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rviz
  agent
)
find_package(yaml-cpp REQUIRED)

find_package(Qt5  REQUIRED COMPONENTS Core Gui Widgets PrintSupport Network)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/gui1_node.cpp
  src/mainwindow.cpp
  src/mycp.cpp
  src/qcustomplot.cpp
  src/vizlib.cpp
  src/leftwidget.cpp
  src/rosnode.cpp
  src/customwidget.cpp
)

set(FORMS
  src/leftwidget.ui
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qt_app
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/gui1
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${SOURCES} ${FORMS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node yaml-cpp)
target_link_libraries(${PROJECT_NAME}_node Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport Qt5::Network)
