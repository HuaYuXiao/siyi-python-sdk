#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
using namespace filesystem;


void SIYI_ROS_SDK::makeDir(std::string& save_path) {
    string test_cmd = "test -d ~/" + save_path + "exp0/";

    if(std::system(test_cmd.c_str())){
        save_path += "exp0/";
        save_path_q = QDir::homePath() + QString::fromStdString("/" + save_path);

        string mkdir_cmd = "mkdir -p ~/" + save_path;
        if(!std::system(mkdir_cmd.c_str())){
            ROS_INFO("dir ~/%s not exist, mkdir", save_path.c_str());

            return;
        }
        else{
            ROS_ERROR("fail to mkdir ~/%s", save_path.c_str());
        }
    }
    else{
        ROS_INFO("dir ~/%s already exist", save_path.c_str());

        // Get all directories starting with "exp"
        vector<int> exp_indexes;

        for (const directory_entry& entry : directory_iterator(save_path_fs)) {
            const string& filename = entry.path().filename().string();

            if (!std::filesystem::is_directory(entry)) {
                ROS_WARN("file %s is not a directory", filename.c_str());
            }
            else if(filename.size() <= 3) {
                ROS_WARN("file %s is not a valid directory", filename.c_str());
            }
            else if(filename.substr(0, 3) == "exp"){
                int index_int = stoi(filename.substr(3));
                exp_indexes.push_back(index_int);
            }
        }

        // find newest number
        int last_exp = *max_element(exp_indexes.begin(), exp_indexes.end());

        // Create a new dir with las_exp+1
        save_path += "exp" + to_string(last_exp + 1) + "/";
        save_path_q = QDir::homePath() + QString::fromStdString("/" + save_path);

        string mkdir_cmd = "mkdir -p ~/" + save_path;

        if(!std::system(mkdir_cmd.c_str())){
            ROS_INFO("dir %s not exist, mkdir", save_path.c_str());
        }
        else{
            ROS_ERROR("fail to mkdir: %s", save_path.c_str());
        }
    }
}

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

        ui->videoPannel->setPixmap(QPixmap::fromImage(img_display_q).scaled(ui->videoPannel->size(),
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
    yolo_boxes_q = "YOLO activated! detection result:\n";

    // Iterate through the received bounding boxes and add them to the vector
    for (const auto& box : msg->bounding_boxes) {
        yolo_boxes.push_back(box);

        yolo_boxes_q.append(QString::fromStdString(box.Class) + " " + QString::number(box.probability) + "\n");
    }

    if(yolo_boxes.empty()){
        yolo_boxes_q.append("no object detected\n");
    }

    ui->yoloPannel->setText(yolo_boxes_q);
}

// 保存无人机当前里程计信息，包括位置、速度和姿态
void SIYI_ROS_SDK::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odom_info.clear();

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

    odom_info << "X: " << odom_pos_[0] << " m\n" <<
              "Y: " << odom_pos_[1] << " m\n" <<
              "Z: " << odom_pos_[2] << " m\n" <<
              "yaw: " << odom_yaw_ * 180 / M_PI << " deg\n";
    odom_info_q = QString::fromStdString(odom_info.str());

    ui->odomPannel->setText(odom_info_q);
}

// Store the current frame
void SIYI_ROS_SDK::saveFrame() {
    // Generate a filename based on the current date and time
    QString current_time_q = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString full_name_q = current_time_q + ".png";
    QFileInfo file_info_q(save_path_q, full_name_q);
    QString full_path_q = file_info_q.absoluteFilePath();

    // Save the frame as an image
    if(!yolo_boxes.empty()) {
        cv::cvtColor(image_yolo_mat, image_save_mat, cv::COLOR_RGB2BGR);
    }
    else{
        image_save_mat = image_raw_mat.clone();
    }

    cv::putText(image_save_mat,
                odom_info.str(),
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(255, 255, 255),
                2);

    cv::imwrite(full_path_q.toStdString(), image_save_mat);

    ROS_INFO("frame MANNUALly saved to %s", full_path_q.toStdString().c_str());
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
        QMessageBox::critical(this,
                              "Error",
                              QString("Could not open video stream: %1").arg(QString::fromStdString(video_resource)));

        return;
    }
    ROS_INFO("video stream connected!");

    odom_info << fixed << setprecision(2);

    // Expand the tilde (~) to the user's home directory
    save_path_fs = std::filesystem::path(std::getenv("HOME")) / save_path;
    // Ensure the directory exists
    makeDir(save_path);

    ui->setupUi(this);

    connect(timer, &QTimer::timeout, this, &SIYI_ROS_SDK::updateFrame);

    odom_info_q = "odom unavailable\n";
    ui->odomPannel->setText(odom_info_q);

    yolo_boxes_q = "YOLO unavailable\n";
    ui->yoloPannel->setText(yolo_boxes_q);

    connect(ui->saveBtn, &QPushButton::clicked, this, &SIYI_ROS_SDK::saveFrame);

    // Update every 30 ms
    timer->start(30);
}
