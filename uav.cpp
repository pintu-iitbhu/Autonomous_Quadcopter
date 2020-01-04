#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
# include <mavros_msgs/SetMode.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped p_s;
geometry_msgs::TwistStamped vel;

void state_cb(const mavros_msgs::State::ConstPtr& msg){current_state= *msg;}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg1){p_s= *msg1;}

int main(int argc,char **argv)
{
 ros::init(argc,argv,"uav");
 ros::NodeHandle nh;

 ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
 ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
 ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/setpoint_position/pose",10, pose_cb);
 ros::Publisher vel_pub=nh.advertise<geometry_msgs::TwistStamped>("mavros/cmd_vel",10);
 ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
 ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/landing");
 ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

ros::Rate rate(40);


while(ros::ok() && !current_state.connected)
{
   ros::spinOnce();
   rate.sleep();
   ROS_INFO("CONNECTING TO FCU");
}


pose.pose.position.x=0;
pose.pose.position.y=0;
pose.pose.position.z=2;

for(int i = 100; ros::ok() && i > 0; --i)
{
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("PUBLISHING WAY POINT");

}


mavros_msgs::SetMode offb_set_mode;   offb_set_mode.request.custom_mode= "OFFBOARD";
mavros_msgs::CommandBool  arm_cmd;    arm_cmd.request.value= true;

 mavros_msgs::CommandTOL land_cmd;
land_cmd.request.yaw = 0;
land_cmd.request.latitude = 0;
land_cmd.request.longitude = 0;
land_cmd.request.altitude = 0;


 ros::Time last_request = ros::Time::now();

while(ros::ok() && !current_state.armed)
{
 if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0)))
    {
      if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {  ROS_INFO("Offboard enabled"); }
    } 

 else
   {
     if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
          if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                 {ROS_INFO("Vehicle armed");}
                 last_request = ros::Time::now();
        }
   }


pose.pose.position.x=0;
pose.pose.position.y=0;
pose.pose.position.z=2;
       local_pos_pub.publish(pose);

       ros::spinOnce();
       rate.sleep();
       ROS_INFO("WHILE");      
}


    // go to the first waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    ROS_INFO("Taking Flight");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub.publish(pose); ros::spinOnce();rate.sleep();}
    ROS_INFO("flight taken");
    // go to the first waypoint
    pose.pose.position.x = 3;
    pose.pose.position.y = 3;
    pose.pose.position.z = 2;
    ROS_INFO("going to first way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub.publish(pose); ros::spinOnce();rate.sleep();}
    ROS_INFO("first way point finished!");
     // go to the second waypoint
    pose.pose.position.x = -3;
    pose.pose.position.y = 3;
    pose.pose.position.z = 2;
    ROS_INFO("going to second way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub.publish(pose); ros::spinOnce();rate.sleep();}
    ROS_INFO("second way point finished!");
     // go to the third waypoint
    pose.pose.position.x = -3;
    pose.pose.position.y = -3;
    pose.pose.position.z = 2;
    ROS_INFO("going to third way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub.publish(pose); ros::spinOnce();rate.sleep();}
    ROS_INFO("third way point finished!");
     // go to the forth waypoint
    pose.pose.position.x = 3;
    pose.pose.position.y = -3;
    pose.pose.position.z = 2;
    ROS_INFO("going to forth way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub.publish(pose); ros::spinOnce();rate.sleep();}
    ROS_INFO("forth way point finished!");
    // go to the landing  waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    ROS_INFO("going to landing way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub.publish(pose); ros::spinOnce();rate.sleep();}
    
 
    
    ROS_INFO("Landing");
    while (ros::ok() && current_state.armed)
    {
        if(landing_client.call(land_cmd) && land_cmd.response.success)
         {ROS_INFO(" LANDING---" );}
    
    ROS_INFO("trying to land");
    ros::spinOnce();
    rate.sleep();}
    ROS_INFO("Landed");

     
    return 0;


}

