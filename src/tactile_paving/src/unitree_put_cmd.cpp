#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

ros::Publisher pub_high;

long cmd_vel_count = 0;
bool unitree_pub_look = false;


void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if(unitree_pub_look){
        return;
    }
    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);
    printf("cmd_x_vel = %f\n", msg->linear.x);
    printf("cmd_y_vel = %f\n", msg->linear.y);
    printf("cmd_yaw_vel = %f\n", msg->angular.z);
    printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
    unitree_legged_msgs::HighCmd high_cmd_ros;
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gaitType = 0;
    high_cmd_ros.speedLevel = 0;
    high_cmd_ros.footRaiseHeight = -0.005;
    high_cmd_ros.bodyHeight = -0.03;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yawSpeed = 0.0f;
    high_cmd_ros.reserve = 0;

    high_cmd_ros.velocity[0] = msg->linear.x;
    high_cmd_ros.velocity[1] = msg->linear.y;
    high_cmd_ros.yawSpeed = msg->angular.z;

    high_cmd_ros.mode = 2;
    high_cmd_ros.gaitType = 1;
    pub_high.publish(high_cmd_ros);
}

void cmdVelCallbackVoice(const geometry_msgs::Twist::ConstPtr &msg)
{
    if(!unitree_pub_look){
        return;
    }
    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);
    printf("cmd_x_vel = %f\n", msg->linear.x);
    printf("cmd_y_vel = %f\n", msg->linear.y);
    printf("cmd_yaw_vel = %f\n", msg->angular.z);
    printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
    unitree_legged_msgs::HighCmd high_cmd_ros;
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gaitType = 0;
    high_cmd_ros.speedLevel = 0;
    high_cmd_ros.footRaiseHeight = -0.005;
    high_cmd_ros.bodyHeight = -0.03;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yawSpeed = 0.0f;
    high_cmd_ros.reserve = 0;

    high_cmd_ros.velocity[0] = msg->linear.x;
    high_cmd_ros.velocity[1] = msg->linear.y;
    high_cmd_ros.yawSpeed = msg->angular.z;

    high_cmd_ros.mode = 2;
    high_cmd_ros.gaitType = 1;
    pub_high.publish(high_cmd_ros);
}

void start_look(const std_msgs::String::ConstPtr& msg){
    std::string dataString = msg->data;
    if (dataString == "start"){
        unitree_pub_look = false;
    }else if (dataString == "stop"){
        unitree_pub_look = true;
    }
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"lidar_node");
    ros::NodeHandle n;
    pub_high = n.advertise<unitree_legged_msgs::HighCmd>("/high_cmd", 1000);
    ros::Subscriber lidar_sub = n.subscribe("/cmd_vel",10,&cmdVelCallback);
    ros::Subscriber lidar_sub2 = n.subscribe("/cmd_vel_voice",10,&cmdVelCallbackVoice);

    ros::Subscriber lidar_sub3 = n.subscribe("/cmd_vel_start",10,&start_look);

    ros::spin();
    return 0;
}
