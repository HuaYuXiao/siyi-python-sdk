#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <atomic>


int main(int argc, char** argv) {
    ros::init(argc, argv, "rtsp_driver_node");
    ros::NodeHandle nh("~");

    std::string resource, camera_name, camera_frame, image_raw_topic, camera_info_topic;
    nh.param("rtsp_resource", resource, std::string("rtsp://192.168.1.222:8554/main.264"));
    nh.param("camera_name", camera_name, std::string(""));
    nh.param("camera_frame", camera_frame, std::string(""));
    nh.param("image_raw_topic", image_raw_topic, std::string("image_raw"));
    nh.param("camera_info_topic", camera_info_topic, std::string(""));

    cv::VideoCapture cap(resource);
    if (!cap.isOpened()) {
        ROS_ERROR("Error opening resource `%s`. Please check.", resource.c_str());
        return 0;
    }
    ROS_INFO("Resource successfully opened");

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>(image_raw_topic, 1);

    cv::Mat cv_image;
    ros::Rate loop_rate(30);
    while (ros::ok() && cap.read(cv_image)) {
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
        image_msg->header.frame_id = camera_frame;
        image_msg->header.stamp = ros::Time::now();
        image_pub.publish(image_msg);

        loop_rate.sleep();
    }

    return 0;
}
