#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <thread>
#include <chrono>
#include <ctime>
#include <atomic>
#include <iostream>
#include <filesystem>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

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
    enum class saveType {
        SURE,
        MANUAL,
        DUBIOUS
    };

    SIYI_ROS_SDK(QWidget *parent = nullptr);
    inline ~SIYI_ROS_SDK(){ delete ui; };

private slots:
    void updateFrame();
    void saveFrameBtn();

private:
    std::string video_resource;
    cv::VideoCapture cap;
    QString video_info_q;
    // Store the current frame
    cv::Mat image_raw_mat;
    sensor_msgs::ImagePtr image_raw_msg;
    std::string camera_name, camera_frame, image_raw_topic, camera_info_topic;
    ros::Publisher image_raw_pub;

    std::string image_yolo_topic, yolo_box_topic;
    ros::Subscriber image_yolo_sub, yolo_box_sub;
    cv::Mat image_yolo_mat;
    // NOTE: avoid using "detection_msgs::BoundingBox yolo_boxes[]"
    // because it is not allowed to define an array with flexible size but not at the end of the class.
    std::vector<detection_msgs::BoundingBox> yolo_boxes;
    QString yolo_boxes_q;

    cv::Mat image_display_mat;
    QImage img_display_q;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
    double odom_roll_, odom_pitch_, odom_yaw_;
    ros::Subscriber odom_sub;
    // Overlay text with position information
    std::stringstream odom_info;
    QString odom_info_q;

    cv::Mat image_save_mat;
    std::string save_path;
    QString save_path_q;
    std::filesystem::path save_path_fs;
    QDir save_path_type_q;

    Ui::SIYI_ROS_SDK *ui;
    QTimer *timer;

    void makeDir(std::string&);
    void yoloImageCallback(const sensor_msgs::Image::ConstPtr&);
    void yoloBoxCallback(const detection_msgs::BoundingBoxes::ConstPtr&);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr&);
    void saveFrame(const saveType);
};

#endif // MAINWINDOW_H
