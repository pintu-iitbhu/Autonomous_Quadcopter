#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define FLIGHT_ALTITUDE 3.0f

mavros_msgs::State current_state_uav0;
geometry_msgs::PoseStamped pose_uav0;

void state_cb_uav0(const mavros_msgs::State::ConstPtr& msg0){current_state_uav0 = *msg0;}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task1");
    ros::NodeHandle nh_uav0;

    ros::Subscriber state_sub_uav0 = nh_uav0.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, state_cb_uav0);
    ros::Publisher local_pos_pub_uav0 = nh_uav0.advertise<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_uav0 = nh_uav0.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming");
    ros::ServiceClient land_client_uav0 = nh_uav0.serviceClient<mavros_msgs::CommandTOL>("uav0/mavros/cmd/land");
    ros::ServiceClient set_mode_client_uav0 = nh_uav0.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode");

    ros::Rate rate(40.0);
    while(ros::ok() && current_state_uav0.connected ){ros::spinOnce();rate.sleep();ROS_INFO("connecting to FCT...");}ROS_INFO("CONNECTED");

    pose_uav0.pose.position.x = 0;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    for(int i = 100; ros::ok() && i > 0; --i){local_pos_pub_uav0.publish(pose_uav0); ROS_INFO("PUBLISHIG 1 SETPOINS"); ros::spinOnce(); rate.sleep(); }

    mavros_msgs::SetMode offb_set_mode_uav0;    offb_set_mode_uav0.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd_uav0;      arm_cmd_uav0.request.value = true;
    mavros_msgs::CommandTOL land_cmd_uav0;
    land_cmd_uav0.request.yaw = 0;
    land_cmd_uav0.request.latitude = 0;
    land_cmd_uav0.request.longitude = 0;
    land_cmd_uav0.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && !current_state_uav0.armed){
        if( current_state_uav0.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){ROS_INFO(current_state_uav0.mode.c_str());
        if(( set_mode_client_uav0.call(offb_set_mode_uav0) &&  offb_set_mode_uav0.response.mode_sent)){ROS_INFO("Offboard enabled");}
            last_request = ros::Time::now();}
        else {if( !current_state_uav0.armed && (ros::Time::now() - last_request > ros::Duration(2.0))){
                if(( arming_client_uav0.call(arm_cmd_uav0) && arm_cmd_uav0.response.success)){ ROS_INFO("Vehicle armed");}
                last_request = ros::Time::now();}}
        local_pos_pub_uav0.publish(pose_uav0); ros::spinOnce(); rate.sleep();ROS_INFO("WHILE");
        }

    // go to the first waypoint
    pose_uav0.pose.position.x = 3;
    pose_uav0.pose.position.y = 3;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub_uav0.publish(pose_uav0); ros::spinOnce();rate.sleep();}
    ROS_INFO("first way point finished!");
    // go to the second waypoint
    pose_uav0.pose.position.x = -3;
    pose_uav0.pose.position.y = 3;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    ROS_INFO("going to second way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub_uav0.publish(pose_uav0); ros::spinOnce();rate.sleep();}
    ROS_INFO("second way point finished!");
    // go to the third waypoint
    pose_uav0.pose.position.x = -3;
    pose_uav0.pose.position.y = -3;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    //send setpoints for 10 seconds
    ROS_INFO("going to third way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub_uav0.publish(pose_uav0); ros::spinOnce();rate.sleep();}
    ROS_INFO("third way point finished!");
    // go to the forth waypoint
    pose_uav0.pose.position.x = 3;
    pose_uav0.pose.position.y = -3;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    ROS_INFO("going to forth way point");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub_uav0.publish(pose_uav0); ros::spinOnce();rate.sleep();}
    ROS_INFO("forth way point finished!");
    pose_uav0.pose.position.x = 0;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    ROS_INFO("going back to the first point!");
    for(int i = 0; ros::ok() && i < 10*20*2; ++i){local_pos_pub_uav0.publish(pose_uav0); ros::spinOnce();rate.sleep();}
     ROS_INFO("Landing");
    while (!(land_client_uav0.call(land_cmd_uav0) && land_cmd_uav0.response.success )){local_pos_pub_uav0.publish(pose_uav0); ROS_INFO("trying to land");ros::spinOnce();rate.sleep();}
    ROS_INFO("Landed");
    return 0;
}
