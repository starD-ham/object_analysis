//
// Created by hamada on 23/01/26.
//

#ifndef OBJECT_ANALYSIS_OBJECT_DETECTION_H
#define OBJECT_ANALYSIS_OBJECT_DETECTION_H
#include "ros/ros.h"
#include "Original_msgs/Ping360.h"
#include "Original_msgs/BoundingBox.h"
#include "Original_msgs/BoundingBoxes.h"
#include "Original_msgs/ObjectStats.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"

class Object_detection_node {
public:
    Object_detection_node();
    ros::NodeHandle m_nh;
    ros::Subscriber m_imgSub;
    ros::Subscriber m_sonarSub;
    ros::Publisher m_sonarPub;
    ros::Publisher m_medianSonarPub;
    ros::Publisher m_labelPub;
    void saveToJpegCb(const sensor_msgs::ImageConstPtr& msg);//画像保存用
    void imgCb(const sensor_msgs::ImageConstPtr& msg);//画像処理用
    void sonarCb(const sensor_msgs::ImageConstPtr& msg);
    void cv_Gamma(cv::Mat& in,cv::Mat& dst,double gamma);//ガンマ処理
    int cv_Label(cv::Mat& in, cv::Mat& dst,cv::Mat& label_img, cv::Mat& stats, cv::Mat& centroids,Original_msgs::BoundingBoxes& imgBoundingBoxes);//ラベリング処理

    /*前処理用メンバ*/
    cv::Mat m_original_img;
    cv::Mat m_gray_img;//グレースケール
    cv::Mat m_otsu_bw_img;//おおつの手法による白黒2値化画像
    cv::Mat m_hist_eq_img;//ヒストグラム平均化
    int m_img_height;
    int m_img_width;

    cv::Mat m_gamma_img;//ガンマ補正
    float gamma;//ガンマ補正パラメータ
    cv::Mat m_bw_img;//2値化
    int m_tsd_bw;//2値化しきい値。グレースケールの輝度0~255で指定

    /*ラベル付用メンバ*/
    cv::Mat m_label_img;//ラベル付した画像。座標にラベル番号が格納されている
    cv::Mat m_stats;//パラメータ
    /* int *param =stats.ptr<int>(i);のように受け取る
     * param[cv::ConnectedComponentsTypes::CC_STAT_AREA]でピクセルのサイズ
     * CC_STAT_LEFT,CC_STAT_TOP, CC_STAT_MAX, CC_STAT_HEIGHT, CC_STAT_WIDTHにすれば各パラメータを取得
     */
    cv::Mat m_centroids;//中心
    int m_nLabel;//ラベル付けしたオブジェクトの数
    int m_tsd_label_max;//ラベル付けしたオブジェクトの最大ピクセル数
    int m_tsd_label_min;//ラベル付けしたオブジェクトの最小ピクセル数

    Original_msgs::BoundingBoxes m_img_boundingBoxes;
    Original_msgs::BoundingBoxes m_sonar_boundingBoxes;
    std::vector<Original_msgs::ObjectStats>  m_objs;

    /*ソナー画像のラベル付用メンバ*/
    cv::Mat m_sonar_img;//sonar image グレースケールで出力させる
    const float x_diff=0.1;//カメラとソナーのx距離
    const float y_diff=0;//カメラとソナーのy距離

    const double unit_mmPerPixel=10/7;//深度1mにおけるピクセルの長さ
    

};


#endif //OBJECT_ANALYSIS_OBJECT_DETECTION_H
