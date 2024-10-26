cmake_minimum_required(VERSION 2.8.3)
project(siyi_ros_sdk)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(PYTHON_EXECUTABLE "/usr/bin/python3.8")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS
		Core
		Gui
		Widgets
)
find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
		message_filters
		tf
		cv_bridge
       std_msgs
	   sensor_msgs
	   geometry_msgs
	   image_transport
)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

catkin_package(
	 INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	  DEPENDS system_lib
)

#qt_standard_project_setup()

include_directories(
       SYSTEM
		include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
		${Qt5Core_INCLUDE_DIRS}
		${Qt5Gui_INCLUDE_DIRS}
		${Qt5Widgets_INCLUDE_DIRS}
)

add_executable(rtsp_driver_node
		src/rtsp_driver_node.cpp)
target_link_libraries(rtsp_driver_node
		${catkin_LIBRARIES}
)

add_executable(qt_test
		include/mainwindow.h
		src/mainwindow.ui
		src/mainwindow.cpp
		src/main.cpp
		${QT_RESOURCES_CPP}
		${QT_FORMS_HPP}
		${QT_MOC_HPP}
)
target_link_libraries(qt_test
		${catkin_LIBRARIES}
		${QT_LIBRARIES}
		Qt5::Core
		Qt5::Widgets
)

#catkin_install_python(PROGRAMS
#		scripts/rtsp_driver_node.py
#		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
