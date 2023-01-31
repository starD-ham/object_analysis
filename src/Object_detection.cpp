//
// Created by hamada on 23/01/26.
//

#include "../include/Object_detection.h"
Object_detection_node::Object_detection_node() {
    this->m_imgSub=this->m_nh.subscribe("/image_rect_color", 1000, &Object_detection_node::saveToJpegCb, this);
    cv::Mat img;
}

/// JPEGに保存するCallback
/// \param msg
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
    ros::Rate rate(5);//5 hz
//    while(ros::ok()){
//        ros::spinOnce();
//        rate.sleep();
//    }
    cv::Mat bw_img;
    cv::Mat hist_eq_img;
    cv::Mat sovel_image,output_image;
    cv::Mat label_image,stats,centroids;
    objectDetectionNode.m_gray_img=cv::imread("/home/hamada/catkin_ws/src/object_analysis/img/1675080818.jpg",cv::IMREAD_GRAYSCALE);
    int img_height,img_width;
    img_height=objectDetectionNode.m_gray_img.cols;
    img_width=objectDetectionNode.m_gray_img.rows;
    std::cout<<"height: "<<img_height<<std::endl;
    std::cout<<"width: "<<img_width<<std::endl;
    objectDetectionNode.m_original_img=cv::imread("/home/hamada/catkin_ws/src/object_analysis/img/1675080818.jpg");
    cv::equalizeHist(objectDetectionNode.m_gray_img,hist_eq_img);
    int tsd_v=30;
    //std::cin>>tsd_v;
    objectDetectionNode.cv_Gamma(0.65);
    cv::threshold(hist_eq_img,bw_img,tsd_v,255,cv::THRESH_BINARY);
    cv::bitwise_not(bw_img,bw_img);//白黒の反転
    //cv::Sobel(bw_img,sovel_image,CV_32F,1,1,1,5);
    //cv::Sobel(objectDetectionNode.m_gray_img,sovel_image,CV_32F,0,1,3);
    //cv::convertScaleAbs(sovel_image,output_image,1,0);

    //画像の収縮を行う
//    cv::Mat erode_kernel;
//    cv::UMat dst,dst2;
//    cv::erode(bw_img,dst,erode_kernel);
//    cv::erode(dst,dst2,erode_kernel);

    // 画像の膨張を行う
    cv::Mat dilate_kernel; // 3x3
    cv::Point center = cv::Point(-1, -1); // アンカーはカーネル中心
    int iteration = 1; // 膨張実施回数
    cv::Mat destination,destination2;
    dilate(bw_img, destination, dilate_kernel, center, iteration);
    dilate(destination, destination2, dilate_kernel, center, iteration);

    // 輪郭抽出を行い、輪郭の内部を塗りつぶす
//    cv::Mat contours,filled;
//    std::vector<cv::Vec4i> hierarchy;
//    cv::findContours(destination2,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE);
//    for(int idx=0;idx>=0;idx=hierarchy[idx][0]){
//        cv::drawContours(filled,contours,idx,0,1,8,hierarchy);
//    }
//    cv::imshow("filled",filled);

    // ラベル付け
    int nLabel=cv::connectedComponentsWithStats(destination2,label_image,stats,centroids);//戻り値：ラベルの数

    cv::imshow("original",objectDetectionNode.m_original_img);
    cv::imshow("gray",objectDetectionNode.m_gray_img);
    cv::imshow("hist_eq",hist_eq_img);
    cv::imshow("gamma",objectDetectionNode.m_gamma_img);
    cv::imshow("bw",bw_img);
    cv::imshow("dilate_1",destination);
    cv::imshow("dilate_2",destination2);
    //cv::imshow("erode1",dst);
    //cv::imshow("erode2",dst2);
//    cv::imshow("sovel",sovel_image);
//    cv::imshow("output",output_image);

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
//    cv::calcHist(&objectDetectionNode.m_gray_img, image_num, channels, cv::Mat(), hist, dim_num, bin_nums, ranges);
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

// =====ラベリング処理=====
    // ラベリング結果の描画色を決定
    std::vector<cv::Vec3b> colors(nLabel);
    colors[0] = cv::Vec3b(0, 0, 0);//背景の色
    //小さすぎるオブジェクトと大きすぎるオブジェクトを除去する
    int th_max,th_min;
    th_max=img_height*img_width*0.03;//3%
    th_min=img_height*img_width*0.001;//0.01%
    std::vector<int> thNLabel;//しきい値範囲内のラベル付したオブジェクトの番号
    for (int i = 1; i < nLabel; ++i) {
        int *param = stats.ptr<int>(i);
        //std::cout << "area "<< i <<" = " << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << std::endl;
        if(param[cv::ConnectedComponentsTypes::CC_STAT_AREA]>th_min&&param[cv::ConnectedComponentsTypes::CC_STAT_AREA]<th_max){
            colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
        }
        else{
            thNLabel.push_back(i);
            colors[i] = cv::Vec3b(0, 0, 0);//背景の色
        }
    }

    // ラベリング結果の描画
    cv::Mat Dst(bw_img.size(), CV_8UC3);
    for (int i = 0; i < Dst.rows; ++i) {
        int *lb = label_image.ptr<int>(i);//指定した行の先頭へのポインタを返す
        cv::Vec3b *pix = Dst.ptr<cv::Vec3b>(i);
        for (int j = 0; j < Dst.cols; ++j) {
            pix[j] = colors[lb[j]];
        }
    }

    //ROIの設定
    for (int i = 1; i < nLabel; ++i) {
        Original_msgs::BoundingBox boundingBox;
        int *param = stats.ptr<int>(i);

        int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
        int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

        std::cout<<"Obuject number:"<<i<<std::endl;
        std::cout<<"left top "<<x<<","<<y<<"Height"<<height<<"Width"<<width<<std::endl;
        if((std::find(thNLabel.begin(), thNLabel.end(),i)==std::end(thNLabel))){
            cv::rectangle(Dst, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
        }
    }

    //重心の出力
    for (int i = 1; i < nLabel; ++i) {
        double *param = centroids.ptr<double>(i);
        int x = static_cast<int>(param[0]);
        int y = static_cast<int>(param[1]);

        if((std::find(thNLabel.begin(), thNLabel.end(),i)==std::end(thNLabel))){
            cv::circle(Dst,cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
        }
    }

    //面積値の出力
    for (int i = 1; i < nLabel; ++i) {
        int *param = stats.ptr<int>(i);
        std::cout << "area "<< i <<" = " << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << std::endl;

        //ROIの左上に番号を書き込む
        int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        std::stringstream num;
        num << i;
        //cv::putText(Dst, num.str(), cv::Point(x+5, y+20), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        if((std::find(thNLabel.begin(), thNLabel.end(),i)==std::end(thNLabel))){
            cv::putText(Dst, num.str(), cv::Point(x+5, y+20), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }
    }

    //cv::imshow("Src", src);
    cv::imshow("Labels", Dst);
    cv::waitKey(-1);
}