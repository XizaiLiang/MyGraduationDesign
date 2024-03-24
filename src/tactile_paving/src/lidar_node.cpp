#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

ros::Publisher vel_pub;    //机器狗控制节点
ros::Publisher tracing_pub;   //控制寻迹开关   

std_msgs::String stop_str;   
std_msgs::String start_str;

//雷达运行场景 1单纯雷达避障 2盲道辅助避障 0空闲
int LIDARMODEL = 2;

//记录旋转次数和每次旋转角度 上限50次
int Rotation_angle_index = 0;
double Rotation_angle_list[360];
//当前需要向左转避障还是右转 0为不转
int left_or_right = 0;
//检测范围，角度
int Detection_range = 30;
//开始回正，此时不再检测障碍物
bool startRighting = false;
int moving = 26;

bool LateralOBSAvoid(const sensor_msgs::LaserScan msg,int left_or_right,bool aheadOBS){
    if(aheadOBS)
        return false;
    float LateralMinDistance = 1.5;
    bool LateralOBS = false;
    int angle = (left_or_right == 1)?90:270;
    for(int i=angle-15;i<angle+15;i++){
        float distance = msg.ranges[i];
        if(distance<LateralMinDistance){
            LateralMinDistance = distance; 
            LateralOBS = true;
        }
    }
    ROS_INFO("侧方障碍物距离%f",LateralMinDistance);
    return LateralOBS;
}


void LidarCallback(const sensor_msgs::LaserScan msg)
{
        //雷达订阅
    if (LIDARMODEL == 0){
        ROS_INFO("雷达状态关闭");
        return;
    }
    geometry_msgs::Twist vel_cmd;
    bool aheadOBS = false;
    float aheadMinDistance = 1.5;
    float coordinateMinDistance = 0;

    for(int i = 180-(int)(Detection_range/2);i<180+(int)(Detection_range/2);i++){
        //扫描狗前方障碍物
        float distance = msg.ranges[i];
        if(distance<aheadMinDistance){
            aheadMinDistance = distance; 
            coordinateMinDistance = i;
            aheadOBS = true;
        }
    }
    //动态变化扫描角度
    if (aheadMinDistance<0.6){
        Detection_range = 85;
    }else if(aheadMinDistance<1){
        Detection_range = 60;
    }else{
        Detection_range = 30;
    }
    ROS_INFO("前方障碍物距离%f",aheadMinDistance);
    if(aheadOBS && !startRighting){
        //发现障碍物，根据最近距离位置反向旋转避障
        if(left_or_right == 1 || coordinateMinDistance<=180){
            //左转
            float RotationAngle = 0.5/aheadMinDistance;
            if(RotationAngle>0.8){
                RotationAngle = 0.8;
            }else{
                vel_cmd.linear.x = 0.6*aheadMinDistance;
            }
            vel_cmd.angular.z = RotationAngle;
            Rotation_angle_list[Rotation_angle_index] = RotationAngle;
            Rotation_angle_index += 1;
            left_or_right = 1;
        }else if(left_or_right == 2 || coordinateMinDistance>180){
            //右转
            float RotationAngle = -0.5/aheadMinDistance;
            if(RotationAngle<-0.8){
                RotationAngle = -0.8;
            }else{
                vel_cmd.linear.x = 0.6*aheadMinDistance;
            }
            vel_cmd.angular.z = RotationAngle;
            Rotation_angle_list[Rotation_angle_index] = RotationAngle;
            Rotation_angle_index += 1;
            left_or_right = 2;
        }
        if(LIDARMODEL == 2)
            tracing_pub.publish(stop_str);
        vel_pub.publish(vel_cmd); 
    }else if(left_or_right==2 || left_or_right == 1){
        //回正方向，并不会恢复原轨迹
        if(moving>=0){
            //继续向前走，避免回正时重新检测到障碍物
            vel_cmd.linear.x = 0.3;
            moving -= 1;
            startRighting = true;   //进入回正时候不能继续检测障碍物
        }else if(Rotation_angle_index == -1){
            //回正完成，重新初始化变量
            if(LateralOBSAvoid(msg,left_or_right,aheadOBS)){
                vel_cmd.linear.x = 0.3;
                vel_pub.publish(vel_cmd); 
            }else{
                startRighting = false;
                left_or_right = 0;
                moving = 25;
                if(LIDARMODEL == 2)
                    tracing_pub.publish(start_str);
            }
        }else if (Rotation_angle_index>=0){
            //开始回正
            vel_cmd.angular.z = Rotation_angle_list[Rotation_angle_index] * -1.0;
            Rotation_angle_index -= 1;
        }
        vel_pub.publish(vel_cmd); 
    }
    if (LIDARMODEL == 1){
        //开阔场景继续直行
        vel_cmd.linear.x = 0.3;
        vel_pub.publish(vel_cmd); 
    }
    ROS_INFO("检测角度%d",Detection_range);
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"lidar_node");
    ros::NodeHandle n;

    std::string LIDARMODEL_str;
    n.param<std::string>("/lidar_modle",LIDARMODEL_str,"0");
    LIDARMODEL = stoi(LIDARMODEL_str);

    ros::Subscriber lidar_sub = n.subscribe("/slamware_ros_sdk_server_node/scan",10,&LidarCallback);   //雷达订阅
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);    //控制机器狗
    tracing_pub = n.advertise<std_msgs::String>("/startTracing",1000);   //控制盲道开关
    stop_str.data = "stop";
    start_str.data = "start";
    ros::spin();
    return 0;
}
