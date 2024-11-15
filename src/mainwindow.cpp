#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;


void SIYI_ROS_SDK::updateFrame() {
    if (cap.read(image_raw_mat)) {
        image_raw_msg = cv_bridge::CvImage(std_msgs::Header(),
                                           "bgr8",
                                           image_raw_mat).toImageMsg();
        image_raw_msg->header.frame_id = camera_frame;
        image_raw_msg->header.stamp = ros::Time::now();
        image_raw_pub.publish(image_raw_msg);

        if(!yolo_boxes.empty()){
            image_display_mat = image_yolo_mat.clone();
        }
        else{
            cv::cvtColor(image_raw_mat, image_display_mat, cv::COLOR_BGR2RGB);
        }

        img_display_q = QImage((const uchar*)image_display_mat.data,
                               image_display_mat.cols,
                               image_display_mat.rows,
                               image_display_mat.step[0],
                               QImage::Format_RGB888);

        ui->videoLabel->setPixmap(QPixmap::fromImage(img_display_q).scaled(ui->videoLabel->size(),
                                                                           Qt::KeepAspectRatio,
                                                                           Qt::SmoothTransformation));
    }

    ros::spinOnce();
}

void SIYI_ROS_SDK::yoloImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if(!yolo_boxes.empty()){
        // Convert the ROS image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

        // Now you have the image as a cv::Mat object
        image_yolo_mat = cv_ptr->image;
    }
}

void SIYI_ROS_SDK::yoloBoxCallback(const detection_msgs::BoundingBoxes::ConstPtr& msg) {
    // Clear the previous bounding boxes
    yolo_boxes.clear();

    // Iterate through the received bounding boxes and add them to the vector
    for (const auto& box : msg->bounding_boxes) {
        yolo_boxes.push_back(box);
    }
}

// 保存无人机当前里程计信息，包括位置、速度和姿态
void SIYI_ROS_SDK::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odom_pos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;

    odom_vel_ << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    // 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    tf::Quaternion odom_q_(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
    );

    tf::Matrix3x3(odom_q_).getRPY(odom_roll_, odom_pitch_, odom_yaw_);
}

// Store the current frame
void SIYI_ROS_SDK::saveFrame() {
    if(odom_pos_[0] * odom_pos_[1] * odom_pos_[2] * odom_yaw_ == 0.0){
        odom_info << "odometry unavailable";
    }
    else {
        odom_info << fixed << setprecision(2);
        odom_info << " X: " << odom_pos_[0] <<
                    " Y: " << odom_pos_[1] <<
                    " Z: " << odom_pos_[2] <<
                    " yaw: " << odom_yaw_ * 180 / M_PI;
    }

    // Generate a filename based on the current date and time
    QString currentTime = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString file = currentTime + ".png";
    QFileInfo fi(save_path_Q, file);
    QString fullPath = fi.absoluteFilePath();

    // Save the frame as an image
    if(!yolo_boxes.empty()) {
        cv::cvtColor(image_yolo_mat, image_save_mat, cv::COLOR_RGB2BGR);
    }
    else{
        cv::cvtColor(image_raw_mat, image_save_mat, cv::COLOR_RGB2BGR);
    }

    cv::putText(image_save_mat,
                odom_info.str(),
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(255, 255, 255),
                2);

    cv::imwrite(fullPath.toStdString(), image_save_mat);

    ROS_INFO("frame MANNUALly saved to %s", fullPath.toStdString().c_str());
}


SIYI_ROS_SDK::SIYI_ROS_SDK(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::SIYI_ROS_SDK), timer(new QTimer(this)) {

    // Initialize ROS NodeHandle
    ros::NodeHandle nh("~");

    nh.param("video_resource", video_resource, std::string(""));
    nh.param("camera_name", camera_name, std::string("siyi_a8_mini"));
    nh.param("camera_frame", camera_frame, std::string("siyi_camera_link"));
    nh.param("image_raw_topic", image_raw_topic, std::string("image_raw"));
    nh.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
    nh.param("image_yolo_topic", image_yolo_topic, std::string("/yolov5/image_out"));
    nh.param("yolo_box_topic", yolo_box_topic, std::string("/yolov5/detections"));
    nh.param("save_path", save_path, std::string("/.siyi-cache/"));

    image_yolo_sub = nh.subscribe<sensor_msgs::Image>
            (image_yolo_topic, 10, &SIYI_ROS_SDK::yoloImageCallback, this);
    yolo_box_sub = nh.subscribe<detection_msgs::BoundingBoxes>
            (yolo_box_topic, 10, &SIYI_ROS_SDK::yoloBoxCallback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/local_position/odom", 30, &SIYI_ROS_SDK::odometryCallback, this);

    image_raw_pub = nh.advertise<sensor_msgs::Image>
            (image_raw_topic, 10);

    ROS_INFO("ROS node initialized!");

    // Open video stream
    cap.open(video_resource);
    if (!cap.isOpened()) {
        QMessageBox::critical(this, "Error", QString("Could not open video stream: %1").arg(QString::fromStdString(video_resource)));
        return;
    }

    ROS_INFO("video stream connected!");

    // Ensure the directory exists
    save_path_Q = QDir::homePath() + save_path.c_str();
    // Create the directory if it does not exist
    if (!save_path_Q.exists()) {
        save_path_Q.mkpath(".");
    }

    ui->setupUi(this);

    connect(timer, &QTimer::timeout, this, &SIYI_ROS_SDK::updateFrame);
    connect(ui->saveButton, &QPushButton::clicked, this, &SIYI_ROS_SDK::saveFrame);

    // Update every 30 ms
    timer->start(30);
}
