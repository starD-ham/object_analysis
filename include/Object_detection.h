//
// Created by hamada on 23/01/26.
//

#ifndef OBJECT_ANALYSIS_OBJECT_DETECTION_H
#define OBJECT_ANALYSIS_OBJECT_DETECTION_H
#include "ros/ros.h"
#include "Original_msgs/Ping360.h"
#include "Original_msgs/BoundingBox.h"
#include "Original_msgs/BoundingBoxes.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"

class Object_detection_node {
public:
    Object_detection_node();
    ros::NodeHandle m_nh;
    ros::Subscriber m_imgSub;
    void saveToJpegCb(const sensor_msgs::ImageConstPtr& msg);
    void cv_Gamma(double gamma);

    cv::Mat m_original_img;
    cv::Mat m_gray_img;
    cv::Mat m_gamma_img;




};


#endif //OBJECT_ANALYSIS_OBJECT_DETECTION_H
