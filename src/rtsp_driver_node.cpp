#include <QApplication>
#include "mainwindow.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "rtsp_driver_node");

    QApplication a(argc, argv);
    SIYI_ROS_SDK siyi_manager;
    siyi_manager.show();

    return a.exec();
}
