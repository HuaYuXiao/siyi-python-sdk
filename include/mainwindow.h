#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <thread>
#include <chrono>
#include <atomic>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QLabel>
#include <QTimer>
#include <QImage>
#include <QDateTime>
#include <QDir>


QT_BEGIN_NAMESPACE
namespace Ui { class SIYI_ROS_SDK; }
QT_END_NAMESPACE

class SIYI_ROS_SDK : public QMainWindow {
    Q_OBJECT

public:
    SIYI_ROS_SDK(QWidget *parent = nullptr);
    ~SIYI_ROS_SDK();

private slots:
    void updateFrame();
    void saveFrame();

private:
    cv::VideoCapture cap;
    cv::Mat cv_image;  // Store the current frame

    std::string resource, camera_name, camera_frame, image_raw_topic, camera_info_topic;

    ros::Publisher image_pub;
    
    Ui::SIYI_ROS_SDK *ui;
    QTimer *timer;
};

#endif // MAINWINDOW_H
