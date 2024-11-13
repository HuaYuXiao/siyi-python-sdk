#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <thread>
#include <chrono>
#include <atomic>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>
#include <QMainWindow>
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QLabel>
#include <QTimer>
#include <QImage>
#include <QDateTime>
#include <QDir>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>


QT_BEGIN_NAMESPACE
namespace Ui { class SIYI_ROS_SDK; }
QT_END_NAMESPACE

class SIYI_ROS_SDK : public QMainWindow {
    Q_OBJECT

public:
    SIYI_ROS_SDK(QWidget *parent = nullptr);
    inline ~SIYI_ROS_SDK(){ delete ui; };

private slots:
    void updateFrame();
    void yoloImageCallback(const sensor_msgs::Image::ConstPtr&);
    void yoloBoxCallback(const detection_msgs::BoundingBoxes::ConstPtr&);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr&);
    void saveFrame();

private:
    cv::VideoCapture cap;
    // Store the current frame
    cv::Mat cv_image_raw;
    std::string video_resource, camera_name, camera_frame, image_raw_topic, camera_info_topic;
    ros::Publisher image_pub;

    ros::Subscriber yolo_image_sub, yolo_box_sub;
    cv::Mat cv_yolo_image;
    std::string image_yolo_topic, yolo_box_topic;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
    double odom_roll_, odom_pitch_, odom_yaw_;
    ros::Subscriber odom_sub;

    std::string save_path;
    QDir save_path_Q;

    Ui::SIYI_ROS_SDK *ui;
    QTimer *timer;
};

#endif // MAINWINDOW_H
