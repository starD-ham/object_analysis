//
// Created by hamada on 23/01/26.
//

#include "../include/Object_detection.h"
Object_detection_node::Object_detection_node() {
    this->m_imgSub=this->m_nh.subscribe("/image_raw", 1000, &Object_detection_node::imgCb, this);
    this->m_labelPub=this->m_nh.advertise<sensor_msgs::Image>("/image/labeled",1000);
    this->m_sonarSub=this->m_nh.subscribe("/sonar_img/raw",1000,&Object_detection_node::sonarCb,this);
    this->m_sonarPub=this->m_nh.advertise<sensor_msgs::Image>("/sonar_img/labeled",1000);
    this->m_medianSonarPub=this->m_nh.advertise<sensor_msgs::Image>("/sonar_img/median",1000);
}

void Object_detection_node::sonarCb(const sensor_msgs::ImageConstPtr &msg) {
        //ping360画像の処理
    cv_bridge::CvImagePtr bridge,median_bridge;
    //bridge=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);//ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
    //this->m_gray_img=bridge->image;
    bridge=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    median_bridge=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    this->m_sonar_img=bridge->image;
    //cv::Mat sonar_draw=objectDetectionNode.m_sonar_img;
    float depth=1.0;//DVLから取得する
    double x_diff=100;//100mm
    std::cout<<m_nLabel<<std::endl;
//    cv::Mat sonar_draw;
//    sonar_draw=m_sonar_img.clone();
    double x_mini=(1200-x_diff/1.5)-(m_gray_img.cols/2)*(1.43/depth)/1.5;
    double x_max=(1200-x_diff/1.5)+((m_gray_img.cols/2)*(1.43/depth))/1.5;
    double width_max=x_max-x_mini;//610pixel
    std::cout<<"image_rows,x_mini, width_max"<<m_gray_img.cols<<","<<x_mini<<","<<width_max<<std::endl;
    cv::rectangle(m_sonar_img, cv::Rect(x_mini, 1200-depth*1000*2/3, width_max, 15), cv::Scalar(255, 255, 255), 2);
    for(int i=0;i<m_nLabel;i++){
        // x,y位置m換算
        double x_pos=(1200-x_diff/1.5)+(m_img_boundingBoxes.bounding_boxes[i].xmin-m_gray_img.cols/2)*(1.43/depth)/1.5;// 深度1mで1pixel0.0143m
        //double y_pos=(-m_img_boundingBoxes.bounding_boxes[i].ymin+(m_original_img.cols/2))*(0.143/depth);
        // 左上の位置-画像中央
        double x_reso=2400;//設定したい距離*上下2倍*センチメートル換算
        double obj_x_size=(m_img_boundingBoxes.bounding_boxes[i].xmax-m_img_boundingBoxes.bounding_boxes[i].xmin)*(1.43/depth)/1.5;
        //x_pos=x_pos-x_diff;//ソナー画像上のx位置
        std::cout<<"obj_size,xmin:"<<obj_x_size<<","<<x_pos<<std::endl;
        //ここでcm換算　xmax-xmin->x方向のピクセル数*0.143/1m
        Original_msgs::BoundingBox sonarBoundingBox;
        sonarBoundingBox.xmin=x_pos;
        sonarBoundingBox.xmax=sonarBoundingBox.xmin+obj_x_size;
        //int width=int(sonarBoundingBox.xmax-sonarBoundingBox.xmin);
        std::cout<<"xmin,xmax:"<<sonarBoundingBox.xmin<<","<<sonarBoundingBox.xmax<<","<<obj_x_size<<std::endl;
        sonarBoundingBox.ymin=x_reso/2-depth*1000*2/3;
        sonarBoundingBox.ymax= sonarBoundingBox.ymin+50;
        m_sonar_boundingBoxes.bounding_boxes.push_back(sonarBoundingBox);
        cv::Mat mat(m_sonar_img,cv::Rect(sonarBoundingBox.xmin,sonarBoundingBox.ymin,obj_x_size,50));
        //resize(mat,mat,cv::Size(),10,10);
        //cv::imshow("clip"+ std::to_string(thNLabel[i]),mat);

        cv::Scalar ave=cv::mean(mat);
        m_objs[i].sonar_bounding_box=sonarBoundingBox;
        m_objs[i].acoustic_ave=ave[2]+ave[1]+ave[0];
        //std::cout<<thNLabel[i]<<":"<<objs->acoustic_ave<<std::endl;
        cv::rectangle(m_sonar_img, cv::Rect(sonarBoundingBox.xmin, sonarBoundingBox.ymin, obj_x_size, 15), cv::Scalar((rand() & 255), (rand() & 255), (rand() & 255)), 2);
        //cv::resize(sonar_draw,sonar_draw,cv::Size(),1000/sonar_draw.cols,1000/sonar_draw.rows);
        //cv::imshow("sonar"+ std::to_string(i),sonar_draw);
    }
    bridge->image=m_sonar_img;
    bridge->encoding="bgr8";
    bridge->header.stamp=ros::Time::now();
    m_sonarPub.publish(bridge);
    ROS_INFO("sonar image publish");

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

void Object_detection_node::imgCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr bridge;
    bridge=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);//ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
    this->m_gray_img=bridge->image;
    this->m_bw_img=bridge->image;
    bridge=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    this->m_original_img=bridge->image;
    cv::Mat sovel_image,output_image;
    //cv::Mat label_image,stats,centroids;

    int img_height,img_width;//元画像のサイズ
    img_height=this->m_gray_img.cols;
    img_width=this->m_gray_img.rows;

    m_img_boundingBoxes.image_header.stamp=ros::Time::now();
    cv::equalizeHist(this->m_gray_img,this->m_hist_eq_img);
    m_tsd_bw=30;
    //std::cin>>tsd_v;
    this->cv_Gamma(m_gray_img,m_gamma_img,0.65);
    cv::threshold(this->m_hist_eq_img,m_bw_img,m_tsd_bw,255,cv::THRESH_BINARY);
    cv::threshold(this->m_gray_img,m_otsu_bw_img,0,255,cv::THRESH_OTSU);
    cv::bitwise_not(m_bw_img,m_bw_img);//白黒の反転
    cv::bitwise_not(m_otsu_bw_img,m_otsu_bw_img);

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
    dilate(m_bw_img, destination, dilate_kernel, center, iteration);
    dilate(destination, destination2, dilate_kernel, center, iteration);

    // 輪郭抽出を行い、輪郭の内部を塗りつぶす
//    cv::Mat contours,filled;
//    std::vector<cv::Vec4i> hierarchy;
//    cv::findContours(destination2,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE);
//    for(int idx=0;idx>=0;idx=hierarchy[idx][0]){
//        cv::drawContours(filled,contours,idx,0,1,8,hierarchy);
//    }
//    cv::imshow("filled",filled);

    cv::Mat Dst(m_bw_img.size(),CV_8UC3);
    m_nLabel=cv_Label(m_bw_img,Dst,m_label_img,m_stats,m_centroids,m_img_boundingBoxes);

    bridge->image=Dst;
    bridge->encoding="bgr8";
    bridge->header.stamp=ros::Time::now();
    m_labelPub.publish(bridge);
    ROS_INFO("labeled image publish");
    //cv::imshow("Src", src);
    //cv::imshow("Labels", Dst);

}

/// ガンマ補正を行う関数
/// \param gamma
void Object_detection_node::cv_Gamma(cv::Mat& in, cv::Mat& dst, double gamma){

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
    cv::LUT(in,cv::Mat(cv::Size(256,1),CV_8U,LUT),m_gamma_img);
    cv::imshow("gamma",dst);
}

/// ラベル付を行う関数
/// \param in
/// \param dst
/// \param label_img
/// \param stats
/// \param centroids
/// \param imgBoundingBoxes
/// \return ラベルの数
int Object_detection_node::cv_Label(cv::Mat& in, cv::Mat& dst,cv::Mat& label_img, cv::Mat& stats, cv::Mat& centroids,Original_msgs::BoundingBoxes& imgBoundingBoxes) {
    // =====ラベリング処理=====
    // ラベル付け
    int nLabel=cv::connectedComponentsWithStats(in,label_img,stats,centroids);//戻り値：ラベルの数

    // ラベリング結果の描画色を決定
    std::vector<cv::Vec3b> colors(nLabel);
    colors[0] = cv::Vec3b(0, 0, 0);//背景の色
    //小さすぎるオブジェクトと大きすぎるオブジェクトを除去する
    int tsd_lb_max,tsd_lb_min;
    tsd_lb_max=in.rows*in.cols*0.03;//3%
    tsd_lb_min=in.rows*in.cols*0.001;//0.01%
    std::vector<int> thNLabel;//しきい値範囲内のラベル付したオブジェクトの番号
    for (int i = 1; i < nLabel; ++i) {
        int *param = m_stats.ptr<int>(i);
        //std::cout << "area "<< i <<" = " << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << std::endl;
        if(param[cv::ConnectedComponentsTypes::CC_STAT_AREA]>tsd_lb_min&&param[cv::ConnectedComponentsTypes::CC_STAT_AREA]<tsd_lb_max){
            colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
            thNLabel.push_back(i);
        }
        else{// しきい値範囲外のピクセル値を持つ場合
            colors[i] = cv::Vec3b(0, 0, 0);//背景の色
        }
    }
    Original_msgs::ObjectStats objs[nLabel];

    // ラベリング結果の描画
    //cv::Mat Dst(in.size(), CV_8UC3);
    for (int i = 0; i < dst.rows; ++i) {
        int *lb = m_label_img.ptr<int>(i);//指定した行の先頭へのポインタを返す
        cv::Vec3b *pix = dst.ptr<cv::Vec3b>(i);//出力画像のi行目の先頭ポインタ
        for (int j = 0; j < dst.cols; ++j) {
            pix[j] = colors[lb[j]];//ラベル番号に等しい色を塗る
        }
    }

    //ROIの設定。BoundingBoxを描画する

    for (int i = 1; i < nLabel; ++i) {
        int *param = m_stats.ptr<int>(i);
        int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
        int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

        //std::cout<<"Object number:"<<i<<std::endl;
        //std::cout<<"left top "<<x<<","<<y<<"Height"<<height<<"Width"<<width<<std::endl;
        if((std::find(thNLabel.begin(), thNLabel.end(),i)!=std::end(thNLabel))){
            Original_msgs::BoundingBox boundingBox;
            boundingBox.xmin=x;
            boundingBox.ymin=y;
            boundingBox.xmax=x+width;
            boundingBox.ymax=y+height;
            boundingBox.id=i;
            imgBoundingBoxes.bounding_boxes.push_back(boundingBox);
            objs[i].image_bounding_box=boundingBox;
            cv::rectangle(dst, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
        }
    }

    //重心の出力
    for (int i = 1; i < nLabel; ++i) {
        double *param = m_centroids.ptr<double>(i);
        int x = static_cast<int>(param[0]);
        int y = static_cast<int>(param[1]);

        if((std::find(thNLabel.begin(), thNLabel.end(),i)!=std::end(thNLabel))){
            objs->center_x=x;
            objs->center_y=y;
            cv::circle(dst,cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
        }
    }

    //面積値の出力
    for (int i = 1; i < nLabel; ++i) {
        int *param = m_stats.ptr<int>(i);
        //std::cout << "area "<< i <<" = " << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << std::endl;

        //ROIの左上に番号を書き込む
        int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        std::stringstream num;
        num << i;
        //cv::putText(Dst, num.str(), cv::Point(x+5, y+20), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        if((std::find(thNLabel.begin(), thNLabel.end(),i)!=std::end(thNLabel))){
            objs->size=param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
            m_objs.push_back(objs[i]);
            cv::putText(dst, num.str(), cv::Point(x+5, y+20), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }
        m_objs.push_back(objs[i]);
    }
    return thNLabel.size();
}


int main(int argc, char **argv){
    ros::init(argc,argv,"objectDetection_node");
    Object_detection_node objectDetectionNode;
    ros::Rate rate(5);//5 hz
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

//    //ping360画像の処理
//    objectDetectionNode.m_sonar_img=cv::imread("/home/hamada/catkin_ws/src/object_analysis/img/1674634360.635032.jpg");
//    //cv::Mat sonar_draw=objectDetectionNode.m_sonar_img;
//    float depth=1.0;//DVLから取得する
//    double x_diff=0.1;
//    std::cout<<thNLabel.size()<<std::endl;
//    for(int i=0;i<thNLabel.size();i++){
//        cv::Mat sonar_draw=objectDetectionNode.m_sonar_img.clone();
//        // x,y位置cm換算
//        double x_pos=(img_boundingBoxes.bounding_boxes[i].xmin-(objectDetectionNode.m_original_img.rows/2))*(0.143/depth);
//        double y_pos=(img_boundingBoxes.bounding_boxes[i].ymin-(objectDetectionNode.m_original_img.cols/2))*(0.143/depth);
//        // 左上の位置-画像中央
//        double x_reso=3.6*2*50;//設定したい距離*上下2倍*センチメートル換算
//        float obj_x_size=round((img_boundingBoxes.bounding_boxes[i].xmax-img_boundingBoxes.bounding_boxes[i].xmin)*(0.143/depth));
//        //ここでcm換算　xmax-xmin->x方向のピクセル数*0.143/1m
//        x_pos=x_pos+x_diff;//ソナー画像上のx位置
//        Original_msgs::BoundingBox sonarBoundingBox;
//        sonarBoundingBox.xmin=round(-x_pos)+x_reso/2;
//        sonarBoundingBox.xmax=sonarBoundingBox.xmin+obj_x_size+10;
//        sonarBoundingBox.ymin=x_reso/2-depth*100;
//        sonarBoundingBox.ymax= sonarBoundingBox.ymin+60;
//        sonar_boundingBoxes.bounding_boxes.push_back(sonarBoundingBox);
//        cv::Mat mat(objectDetectionNode.m_sonar_img,cv::Rect(sonarBoundingBox.xmin,sonarBoundingBox.ymin,obj_x_size,50));
//        resize(mat,mat,cv::Size(),10,10);
//        cv::imshow("clip"+ std::to_string(thNLabel[i]),mat);
//
//        cv::Scalar ave=cv::mean(mat);
//        objs->sonar_bounding_box=sonarBoundingBox;
//        objs->acoustic_ave=ave[2]+ave[1]+ave[0];
//        std::cout<<thNLabel[i]<<":"<<objs->acoustic_ave<<std::endl;
//        cv::rectangle(sonar_draw, cv::Rect(sonarBoundingBox.xmin, sonarBoundingBox.ymin, obj_x_size+10, 15), cv::Scalar((rand() & 255), (rand() & 255), (rand() & 255)), 2);
//        //cv::resize(sonar_draw,sonar_draw,cv::Size(),1000/sonar_draw.cols,1000/sonar_draw.rows);
//        cv::imshow("sonar"+ std::to_string(thNLabel[i]),sonar_draw);
//    }
    //
    /*x_reso=echo_msg.range*2*50;*/



    cv::waitKey(-1);
}