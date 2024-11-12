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
		tf
		cv_bridge
       std_msgs
	   sensor_msgs
		nav_msgs
)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

catkin_package(
	 INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
#	  DEPENDS system_lib
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

add_executable(siyi_ros_sdk
		include/mainwindow.h
		src/mainwindow.ui
		src/mainwindow.cpp
		src/main_node.cpp
		${QT_RESOURCES_CPP}
		${QT_FORMS_HPP}
		${QT_MOC_HPP}
)
target_link_libraries(siyi_ros_sdk
		${catkin_LIBRARIES}
		Eigen3::Eigen
		${QT_LIBRARIES}
		Qt5::Core
		Qt5::Widgets
)

catkin_install_python(PROGRAMS
		scripts/fancy_round_cv.py
		scripts/rtcp_viewer.py
		scripts/test_center_gimbal.py
		scripts/test_from_rtsp_to_rtmp.py
		scripts/test_get_hw_id.py
		scripts/test_record_video.py
		scripts/test_zoom.py
		scripts/gimbal_angle_control.py
		scripts/ml_object_tracker.py
		scripts/siyi_control.py
		scripts/test_follow_mode.py
		scripts/test_get_fw_ver.py
		scripts/test_gimbal_rotation.py
		scripts/test_rtmp_stream.py
		scripts/udp_server.py
		scripts/gimbal_control.py
		scripts/object_tracker.py
		scripts/siyi_tracker.py
		scripts/test_fpv_mode.py
		scripts/test_get_gimbal_info.py
		scripts/test_lock_mode.py
		scripts/test_rtsp.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
