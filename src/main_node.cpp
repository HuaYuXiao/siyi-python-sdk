#include <QApplication>
#include "mainwindow.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "siyi_ros_sdk");

    QApplication a(argc, argv);
    SIYI_ROS_SDK siyi_ros_sdk;
    siyi_ros_sdk.show();

    return a.exec();
}
