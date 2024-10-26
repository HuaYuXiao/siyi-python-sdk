#include "mainwindow.h"
#include "ui_mainwindow.h"


using namespace std;


SIYI_ROS_SDK::SIYI_ROS_SDK(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::SIYI_ROS_SDK), timer(new QTimer(this)) {
    ui->setupUi(this);

    // Initialize ROS NodeHandle
    ros::NodeHandle nh("~");

    nh.param("rtsp_resource", resource, std::string("rtsp://192.168.1.222:8554/main.264"));
    nh.param("camera_name", camera_name, std::string("siyi_camera"));
    nh.param("camera_frame", camera_frame, std::string("siyi_camera_link"));
    nh.param("image_raw_topic", image_raw_topic, std::string("image_raw"));
    nh.param("camera_info_topic", camera_info_topic, std::string("camera_info"));

    image_pub = nh.advertise<sensor_msgs::Image>(image_raw_topic, 1);

    // Open the RTSP stream
    cap.open(resource);
    if (!cap.isOpened()) {
        QMessageBox::critical(this, "Error", QString("Could not open RTSP stream: %1").arg(QString::fromStdString(resource)));
        return;
    }

    connect(timer, &QTimer::timeout, this, &SIYI_ROS_SDK::updateFrame);
    connect(ui->saveButton, &QPushButton::clicked, this, &SIYI_ROS_SDK::saveFrame);

    timer->start(30); // Update every 30 ms
}

SIYI_ROS_SDK::~SIYI_ROS_SDK() {
    delete ui;
}

void SIYI_ROS_SDK::updateFrame() {
    if (cap.read(cv_image)) {
        if(ros::ok()){
            sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
            image_msg->header.frame_id = camera_frame;
            image_msg->header.stamp = ros::Time::now();
            image_pub.publish(image_msg);
        }

        cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
        QImage img((const uchar*)cv_image.data, cv_image.cols, cv_image.rows, cv_image.step[0], QImage::Format_RGB888);
        ui->videoLabel->setPixmap(QPixmap::fromImage(img).scaled(ui->videoLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

// Store the current frame
void SIYI_ROS_SDK::saveFrame() {
    if (cv_image.empty()) {
        QMessageBox::warning(this, "Error", "No frame available to save.");
        return;
    }

    // Ensure the directory exists
    QDir dir(QDir::homePath() + "/easondrone_ws/vision/siyi-ros-sdk/img/");
    if (!dir.exists()) {
        dir.mkpath(".");  // Create the directory if it does not exist
    }

    // Generate a filename based on the current date and time
    QString currentTime = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString file = currentTime + ".png";
    QFileInfo fi(dir, file);
    QString fullPath = fi.absoluteFilePath();

    // Save the frame as an image
    cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
    cv::imwrite(fullPath.toStdString(), cv_image);
}
