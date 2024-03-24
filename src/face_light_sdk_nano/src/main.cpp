#include "FaceLightClient.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

int MODE = 0;

void faceLight(const std_msgs::String::ConstPtr& msg){
    std::string dataString = msg->data;
    MODE = stoi(dataString);
}


int main(int argc,char *argv[]){
    ros::init(argc, argv, "face_light");
    ros::NodeHandle nh;
    ros::Subscriber signal_lamp = nh.subscribe("/face_light", 10, &faceLight);  //获取paddle检测结果
    // client.setAllLed(client.red);
    // client.sendCmd();
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        FaceLightClient client;
        switch (MODE){
        case 1:
            /* Same Color Test */
            client.setAllLed(client.red);
            client.sendCmd();
            usleep(2000000);
            break;
            
        case 2:
            /* Custom Setting */
            client.setAllLed(client.green);
            client.sendCmd();
            usleep(2000000);
            // loop_rate.sleep();
            break;
        //hong lan jiaoti 
        case 3:
            client.setLedColor(0,client.red);
            client.setLedColor(1,client.red);
            client.setLedColor(2,client.red);
            client.setLedColor(3,client.red);
            client.setLedColor(4,client.red);
            client.setLedColor(5,client.red);
            client.setLedColor(6,client.black);
            client.setLedColor(7,client.black);
            client.setLedColor(8,client.black);
            client.setLedColor(9,client.black);
            client.setLedColor(10,client.black);
            client.setLedColor(11,client.black);
            client.sendCmd();
            usleep(150000);
            client.setLedColor(0,client.black);
            client.setLedColor(1,client.black);
            client.setLedColor(2,client.black);
            client.setLedColor(4,client.black);
            client.setLedColor(5,client.black);
            client.setLedColor(6,client.blue);
            client.setLedColor(7,client.blue);
            client.setLedColor(8,client.blue);
            client.setLedColor(9,client.blue);
            client.setLedColor(10,client.blue);
            client.setLedColor(11,client.blue);
            client.sendCmd();
            usleep(150000);
            break;
        //huang se suang shan 
        case 4:
            client.setLedColor(0,client.yellow);
            client.setLedColor(1,client.yellow);
            client.setLedColor(2,client.yellow);
            client.setLedColor(3,client.yellow);
            client.setLedColor(4,client.yellow);
            client.setLedColor(5,client.yellow);
            client.setLedColor(6,client.black);
            client.setLedColor(7,client.black);
            client.setLedColor(8,client.black);
            client.setLedColor(9,client.black);
            client.setLedColor(10,client.black);
            client.setLedColor(11,client.black);
            client.sendCmd();
            usleep(150000);
            client.setLedColor(0,client.black);
            client.setLedColor(1,client.black);
            client.setLedColor(2,client.black);
            client.setLedColor(4,client.black);
            client.setLedColor(5,client.black);
            client.setLedColor(6,client.red);
            client.setLedColor(7,client.red);
            client.setLedColor(8,client.red);
            client.setLedColor(9,client.red);
            client.setLedColor(10,client.red);
            client.setLedColor(11,client.red);
            client.sendCmd();
            usleep(150000);
            break;

        case 0:
            client.setAllLed(client.black);
            client.sendCmd();
            break;
        default:
            break;
        }
        ROS_INFO("model:%d",MODE);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}