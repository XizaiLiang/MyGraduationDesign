#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Twist.h>
#include <tactile_paving/signalLamp.h>
#include <termios.h>
#include "std_msgs/String.h"

ros::Publisher vel_pub;    //机器狗控制节点
ros::Publisher face_pub;    //light
bool IsRedLight = false;   //当前红灯状态
bool IsRadarControl = false; //当前雷达状态
bool PaddleReturnTrack = false; //当前下巴盲道识别状态

//膨胀腐蚀算子 3*3
cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
//TrackBar发生改变的回调函数
void onChangeTrackBar(int pos, void* userdata){};

void ImageCb(cv::Mat frame);

//计算盲道的中线
float getPavingYellow(cv::Mat dst_erode_image, int row){
    cv::Mat color = dst_erode_image.row(row);
    int white_end = 0;
    int white_start = -1;

    for(int i=0;i<color.cols;i++){
        if (int(color.at<uchar>(i) == 0)){
            if (white_start == -1)white_start = i;
            white_end=i;
        }
        //debug
        dst_erode_image.at<uchar>(row, i) = 0;
    }
    if (white_end == 0)white_end=1;
    float center = (white_end-1+white_start)/2 - (dst_erode_image.cols/2);
    return center;
}


void GetImageChin(std::string cam_down,std::string img_sleep,std::string IpLastSegment){
    std::cout<<"cam_down:"<<cam_down<<"\nIpLastSegment:"<<IpLastSegment<<"\n"<<std::endl;
    std::string udpstrPrevData = "udpsrc address=192.168.123."+ IpLastSegment + " port=";
    //端口：前方，下巴，左，右，腹部
	std::array<int,5> udpPORT = std::array<int, 5>{9201, 9202, 9203, 9204, 9205};
    std::string udpstrBehindData = " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
    std::string udpSendIntegratedPipe = udpstrPrevData +  std::to_string(udpPORT[stoi(cam_down)-1]) + udpstrBehindData;
    std::cout<<"udpSendIntegratedPipe:"<<udpSendIntegratedPipe<<std::endl;
    cv::VideoCapture cap("/home/lhy/C715228BBC959C4D5927324853D6DA5D.mp4");
    if(!cap.isOpened()){
        ROS_ERROR("Failed to open video stream");
        return;
    }
    cv::Mat frame;
    while (ros::ok())
    {
        cap >> frame;
        if(frame.empty())
            break;
        cv::Mat image_resize;
        cv::resize(frame,image_resize,cv::Size(640,640));
        ImageCb(image_resize);
        char key = cv::waitKey(stoi(img_sleep));
        if(key == 'q') // press ESC key
           break;
        ros::spinOnce();
    }
    cap.release();
    ROS_ERROR("ros error");
}

long FPS_ImageCb = 0;
int RedLightWaiting = 30;     //避免误检测
void ImageCb(cv::Mat frame){
    FPS_ImageCb += 1;
    ROS_INFO("fps:%ld",FPS_ImageCb);
    geometry_msgs::Twist twist;
    if(IsRedLight){
        RedLightWaiting -= 1;
        if(RedLightWaiting < 0){
            std_msgs::String lightModel;
            lightModel.data = "1";
            face_pub.publish(lightModel);   
            RedLightWaiting = 30;
            IsRedLight = false;
        }else{
            ROS_INFO("等红灯");
            vel_pub.publish(twist);
            return;
        }
    }
    if(IsRadarControl){
        ROS_INFO("出现障碍物，雷达已接管");
        return;
    }
    //初始化cvBar
    int u_h = cv::getTrackbarPos("UH", "output");
    int u_s = cv::getTrackbarPos("US", "output");
    int u_v = cv::getTrackbarPos("UV", "output");

    int dilate_ = cv::getTrackbarPos("dilate", "output"); //膨胀
    int erode_ = cv::getTrackbarPos("erode", "output"); //腐蚀
    int centerUp_ = cv::getTrackbarPos("centerUp", "output");  //上检测线
    int centerDown_ = cv::getTrackbarPos("centerDown", "output"); //下检测线

    cv::Mat mask,hsv,ret;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar scalarL = cv::Scalar(0,0,0);
    cv::Scalar scalarH = cv::Scalar(u_h,u_s,u_v);
    cv::inRange(hsv, scalarL, scalarH, mask);
    cv::bitwise_and(frame, frame,ret,mask);
    cv::Mat hsv_bgr,gray_hsv;
    cv::cvtColor(ret, hsv_bgr, cv::COLOR_HSV2BGR);
    cv::cvtColor(hsv_bgr,gray_hsv, cv::COLOR_BGR2GRAY);
    cv::Mat threshold_image;
    cv::threshold(gray_hsv,threshold_image,1,255, cv::THRESH_BINARY);
    cv::Mat dst_dilate_image;
    cv::dilate(threshold_image,dst_dilate_image,element,cv::Point(-1,-1),dilate_);
    cv::Mat dst_erode_image;
    cv::erode(dst_dilate_image,dst_erode_image,element,cv::Point(-1,-1),erode_);

    //寻迹可调   
    float centerUp = getPavingYellow(dst_erode_image,centerUp_);
    float centerDown = getPavingYellow(dst_erode_image,centerDown_);
    cv::Mat show_image;
    cv::cvtColor(dst_erode_image,show_image,cv::COLOR_GRAY2BGR);
    cv::Mat output_image;
    cv::hconcat(show_image,frame,output_image);

    cv::resize(output_image,output_image,cv::Size(640,320));
    cv::imshow("dst_erode_image",output_image);

    //确定调整角度后确定走向  角度调整时不向前行走

    if(abs(centerUp)>250 && abs(centerDown)>250){
        //寻找不到盲道 ,此时应该用飞桨查找盲道，然后向盲道靠近
        if (!PaddleReturnTrack){
            PaddleReturnTrack = true;
            vel_pub.publish(twist);
        }
        ROS_INFO("寻找不到盲道");
        return;
    }
    PaddleReturnTrack = false;
    
    //保证一条有效的检测线
    float  centerAngle;   //旋转角度
    if (abs(centerUp)>250){
        centerUp = centerDown;
        centerAngle = 0.0;
    }else if(abs(centerDown)>250){
        centerDown = centerUp;
        centerAngle = 0.0;
    }else{
        centerAngle =  centerUp-centerDown;
    }

    /* 角度调整 */
    bool adjustAngle=1;
    if(centerAngle>20){
        //角度向右倾斜，狗向右转
        twist.angular.z = centerAngle/-340;
        ROS_INFO("狗向右转!");
    }else if (centerAngle<-20){
        //角度向左倾斜，狗向左转
        twist.angular.z = centerAngle/-340;
        ROS_INFO("狗向左转!");
    }else{
        //角度小不调整
        adjustAngle = 0;
        ROS_INFO("角度小不调整");
    }

    /* 左右微调并向前行走  */
    if (centerUp<-10 && !adjustAngle){
        //盲道偏左
        centerUp = (centerUp<-70)?-70:centerUp;
        twist.linear.y = centerUp/500;
        twist.linear.x = 0.1;
        ROS_INFO("盲道偏左");
    }else if(centerUp>10 && !adjustAngle){
        //盲道偏右
        centerUp = (centerUp>70)?70:centerUp;
        twist.linear.y = centerUp/500;
        twist.linear.x = 0.1;
        ROS_INFO("盲道偏右");
    }else if(!adjustAngle){
        //偏移小不调整
        twist.linear.x = 0.1;
        ROS_INFO("偏移小不调整!");
    }
    //发送
    vel_pub.publish(twist);
}

long FPS_Paddle = 0;
void red_stop(const tactile_paving::signalLamp::ConstPtr &msg){
    ROS_INFO("%s,%d,%d,%d,%d",
        msg->lamp_class.c_str(),msg->xmin,msg->ymin,msg->xmax,msg->ymax);
    FPS_Paddle = (long)msg->fps;

    if (msg->lamp_class == "red_light"){
        //红灯
        std_msgs::String lightModel;
        lightModel.data = "1";
        face_pub.publish(lightModel);
        IsRedLight = true;
    }

    if(msg->lamp_class == "green_light"){
        //绿灯
        std_msgs::String lightModel;
        lightModel.data = "2";
        face_pub.publish(lightModel);
        IsRedLight = false;
    }

    if(PaddleReturnTrack && msg->lamp_class == "blind_path"){
        geometry_msgs::Twist twist;
        float xmiddle = (msg->xmin+msg->xmax)/2;
        if(xmiddle<310){
            twist.linear.y = 0.2;
        }else if(xmiddle>330){
            twist.linear.y = -0.2;
        }else{
            twist.linear.x = 0.1;
        }
        vel_pub.publish(twist);
    }
    
}

void ReturnOriginalTtack(const std_msgs::String::ConstPtr& msg){
    //订阅雷达检测完毕信息控制狗返回盲道
    std::string dataString = msg->data;
    std_msgs::String lightModel;
    if (dataString == "start"){
        lightModel.data = "0";
        face_pub.publish(lightModel);
        IsRadarControl = false;
    }else if (dataString == "stop"){
        lightModel.data = "4";
        face_pub.publish(lightModel);
        IsRadarControl = true;
    }
}

void initTrackBar(){
    //TrackBar初始化
    int L_min = 0;
    int U_min = 255;
    int max_lowThreshold = 255;
    int us_init = 150;
    cv::namedWindow("output", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("UH", "output", &U_min, max_lowThreshold, onChangeTrackBar);
    cv::createTrackbar("US", "output", &us_init, max_lowThreshold, onChangeTrackBar);
    cv::createTrackbar("UV", "output", &U_min, max_lowThreshold, onChangeTrackBar);
    int _dilate = 4;
    int _erode = 10;
    int  _centerUp = 290;
    int _centerDown = 630;
    cv::createTrackbar("dilate", "output", &_dilate, 10, onChangeTrackBar);
    cv::createTrackbar("erode", "output", &_erode, 10, onChangeTrackBar);
    cv::createTrackbar("centerUp", "output", &_centerUp, 639, onChangeTrackBar);
    cv::createTrackbar("centerDown", "output", &_centerDown, 639, onChangeTrackBar);
}

// bool Search_path(){
//     //Blind channel identification failure

//     return true;
// }


int main(int argc,char *argv[])
{
    ros::init(argc, argv, "example_tracing");
    ros::NodeHandle nh;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);    //控制机器狗
    face_pub = nh.advertise<std_msgs::String>("/face_light",10);
    // startTracing = n.advertise<std_msgs::String>("/startTracing", 10000);  

    ros::Subscriber signal_lamp = nh.subscribe("signalLamp", 10, &red_stop);  //获取paddle检测结果
    ros::Subscriber tracing_sub = nh.subscribe("/startTracing",10,&ReturnOriginalTtack);  //获取雷达状态

    std::string cam_down;
    std::string img_sleep;
    std::string IpLastSegment;

    nh.param<std::string>("/cam_down",cam_down,"0");
    nh.param<std::string>("/img_sleesp",img_sleep,"50");
    nh.param<std::string>("/IpLastSegment",IpLastSegment,"15");
    initTrackBar();
    GetImageChin(cam_down,img_sleep,IpLastSegment);
    return 0;
}
