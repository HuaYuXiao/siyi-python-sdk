#include <thread>
#include <chrono>
#include <atomic>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <QApplication>

#include "mainwindow.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "rtsp_driver_node");

    QApplication a(argc, argv);
    SIYI_ROS_SDK siyi_manager;
    siyi_manager.show();

    return a.exec();
}
