#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "math.h"

double theta;
double count=0.0;

mavros_msgs  ::State current_state;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped c_p;
geometry_msgs::TwistStamped vs;

void state_cb(const mavros_msgs::State::ConstPtr& msg0){current_state = *msg0;}
void pose_cb(const  geometry_msgs::PoseStamped::ConstPtr& msg1){c_p = *msg1;}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    ros::ServiceClient set_mode_client= nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Rate rate(40.0);
    while(ros::ok() && !current_state.connected ){ros::spinOnce();rate.sleep();
    ROS_INFO("connecting to FCT...");}ROS_INFO("CONNECTED");
    
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 4;
    for(int i = 100; ros::ok() && i > 0; --i){local_pos_pub.publish(pose);ROS_INFO("PUBLISHIG 1 SETPOINS"); ros::spinOnce(); rate.sleep(); } 

    mavros_msgs::SetMode offb_set_mode;    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;      arm_cmd.request.value = true;
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    ros::Time last_request = ros::Time::now();

     while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
          if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ ROS_INFO("Offboard enabled");} last_request = ros::Time::now();} 
        else{if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
              if( arming_client.call(arm_cmd) && arm_cmd.response.success){ROS_INFO("Vehicle armed");}last_request = ros::Time::now();}}
    theta = 0.2*count*0.05; pose.pose.position.x = 2*sin(theta); pose.pose.position.y = 2*cos(theta); pose.pose.position.z = 4; count++;
    local_pos_pub.publish(pose); ros::spinOnce(); rate.sleep(); }
    ROS_INFO("Landing");
    while (!(land_client.call(land_cmd) && land_cmd.response.success )){local_pos_pub.publish(pose); ROS_INFO("trying to land");ros::spinOnce();rate.sleep();}
    ROS_INFO("Landed");
    return 0;
}
