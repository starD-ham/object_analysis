//
// Created by hamada on 23/01/26.
//

#include "../include/Object_detection.h"
Object_detection_node::Object_detection_node() {
    this->m_imgSub=this->m_nh.subscribe("/image_rect_color", 1000, &Object_detection_node::saveToJpegCb, this);
    cv::Mat img;
}

void Object_detection_node::saveToJpegCb(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat mat_image;
    cv_bridge::CvImagePtr bridge;
    bridge=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);//ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
    mat_image=bridge->image;
    cv::imwrite("/home/hamada/catkin_ws/src/object_analysis/img/"+std::to_string(ros::Time::now().sec)+".jpg",mat_image);
}

//---------------------------------------------------------------
//【関数名　】：cv_Gamma
//【処理概要】：ガンマ補正
//【引数　　】：src        = 入力画像
//　　　　　　：dst        = 出力画像
//　　　　　　：gamma   = ガンマ補正値
//【戻り値　】：なし
//【備考　　】：モノクロ/カラー対応可
//　　　　　　：カラーの場合はRGB全て同じガンマ補正値
//---------------------------------------------------------------
void Object_detection_node::cv_Gamma(double gamma){

    int i;

    uchar LUT[256];

    //ガンマ補正テーブルの作成
    for (i = 0; i < 256; i++)
    {
        LUT[i] = (int)(pow((double)i / 255.0, 1.0 / gamma) * 255.0);
    }

    //CvMatへ変換
    cv::Mat lut_mat = cv::Mat(1, 256, CV_8UC1, LUT);

    //ルックアップテーブル変換
    cv::LUT(m_gray_img,cv::Mat(cv::Size(256,1),CV_8U,LUT),m_gamma_img);
    cv::imshow("gamma",m_gamma_img);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"objectDetection_node");
    Object_detection_node objectDetectionNode;
//    ros::Rate rate(5);//5 hz
//    while(ros::ok()){
//        ros::spinOnce();
//        rate.sleep();
//    }
    cv::Mat bw_img;
    objectDetectionNode.m_gray_img=cv::imread("/home/hamada/catkin_ws/src/object_analysis/img/1674806139.jpg",cv::IMREAD_GRAYSCALE);
    objectDetectionNode.m_original_img=cv::imread("/home/hamada/catkin_ws/src/object_analysis/img/1674806139.jpg");
    int tsd_v=123;
    //std::cin>>tsd_v;
    objectDetectionNode.cv_Gamma(0.65);
    cv::threshold(objectDetectionNode.m_gamma_img,bw_img,tsd_v,255,cv::THRESH_BINARY);
    cv::imshow("original",objectDetectionNode.m_original_img);
    cv::imshow("gray",objectDetectionNode.m_gray_img);
    cv::imshow("gamma",objectDetectionNode.m_gamma_img);
    cv::imshow("bw",bw_img);

    //cv::imshow("ro_title",bw_img);

//    cv::Mat hist_img;
//    // ヒストグラムを生成するために必要なデータ
//    int image_num = 1;      // 入力画像の枚数
//    int channels[] = { 0 }; // cv::Matの何番目のチャネルを使うか　今回は白黒画像なので0番目のチャネル以外選択肢なし
//    cv::MatND hist;         // ここにヒストグラムが出力される
//    int dim_num = 1;        // ヒストグラムの次元数
//    int bin_num = 256;       // ヒストグラムのビンの数
//    int bin_nums[] = { bin_num };      // 今回は1次元のヒストグラムを作るので要素数は一つ
//    float range[] = { 0, 256 };        // 扱うデータの最小値、最大値　今回は輝度データなので値域は[0, 255]
//    const float *ranges[] = { range }; // 今回は1次元のヒストグラムを作るので要素数は一つ
//
//    // 白黒画像から輝度のヒストグラムデータ（＝各binごとの出現回数をカウントしたもの）を生成
//    cv::calcHist(&gray_img, image_num, channels, cv::Mat(), hist, dim_num, bin_nums, ranges);
//    // histogramを描画するための画像領域を確保
//    int img_width = 512;
//    int img_height = 512;
//    hist_img = cv::Mat(cv::Size(img_width, img_height), CV_8UC3);
//
//    // ヒストグラムのスケーリング
//    // ヒストグラムのbinの中で、頻度数最大のbinの高さが、ちょうど画像の縦幅と同じ値になるようにする
//    double max_val = 0.0;
//    cv::minMaxLoc(hist, 0, &max_val);
//    hist = hist * (max_val ? img_height / max_val : 0.0);
//
//    // ヒストグラムのbinの数だけ矩形を書く
//    for (int j = 0; j < bin_num; ++j){
//        // saturate_castは、安全に型変換するための関数。桁あふれを防止
//        int bin_w = cv::saturate_cast<int>((double)img_width / bin_num);
//        cv::rectangle(
//                hist_img,
//                cv::Point(j*bin_w, hist_img.rows),
//                cv::Point((j + 1)*bin_w, hist_img.rows - cv::saturate_cast<int>(hist.at<float>(j))),
//                cv::Scalar::all(0), -1);
//    }
//    std::cout<<hist<<std::endl;
//    cv::imshow("histgram",hist_img);
    cv::waitKey(-1);
}