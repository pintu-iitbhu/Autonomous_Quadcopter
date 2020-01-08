#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define FLIGHT_ALTITUDE 2.0f

mavros_msgs::State current_state_uav1;
mavros_msgs::State current_state_uav2;
mavros_msgs::State current_state_uav3;
mavros_msgs::State current_state_uav0;

geometry_msgs::PoseStamped pose_uav1;
geometry_msgs::PoseStamped pose_uav2;
geometry_msgs::PoseStamped pose_uav3;
geometry_msgs::PoseStamped pose_uav0;


void state_cb_uav1(const mavros_msgs::State::ConstPtr& msg1){current_state_uav1 = *msg1;}

void state_cb_uav2(const mavros_msgs::State::ConstPtr& msg2){current_state_uav2 = *msg2;}

void state_cb_uav3(const mavros_msgs::State::ConstPtr& msg3){current_state_uav3 = *msg3;}

void state_cb_uav0(const mavros_msgs::State::ConstPtr& msg0){current_state_uav0 = *msg0;}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh_uav1;
    ros::NodeHandle nh_uav2;
    ros::NodeHandle nh_uav3;
    ros::NodeHandle nh_uav0;
    
    ros::Subscriber state_sub_uav1 = nh_uav1.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, state_cb_uav1);
    ros::Publisher local_pos_pub_uav1 = nh_uav1.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_uav1 = nh_uav1.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
    ros::ServiceClient land_client_uav1 = nh_uav1.serviceClient<mavros_msgs::CommandTOL>("uav1/mavros/cmd/land");
    ros::ServiceClient set_mode_client_uav1 = nh_uav1.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");

    ros::Subscriber state_sub_uav2 = nh_uav2.subscribe<mavros_msgs::State>("uav2/mavros/state", 10, state_cb_uav2);
    ros::Publisher local_pos_pub_uav2 = nh_uav2.advertise<geometry_msgs::PoseStamped>("uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_uav2 = nh_uav2.serviceClient<mavros_msgs::CommandBool>("uav2/mavros/cmd/arming");
    ros::ServiceClient land_client_uav2 = nh_uav2.serviceClient<mavros_msgs::CommandTOL>("uav2/mavros/cmd/land");
    ros::ServiceClient set_mode_client_uav2 = nh_uav2.serviceClient<mavros_msgs::SetMode>("uav2/mavros/set_mode");
    
    ros::Subscriber state_sub_uav3 = nh_uav3.subscribe<mavros_msgs::State>("uav3/mavros/state", 10, state_cb_uav3);
    ros::Publisher local_pos_pub_uav3 = nh_uav3.advertise<geometry_msgs::PoseStamped>("uav3/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_uav3 = nh_uav3.serviceClient<mavros_msgs::CommandBool>("uav3/mavros/cmd/arming");
    ros::ServiceClient land_client_uav3 = nh_uav3.serviceClient<mavros_msgs::CommandTOL>("uav3/mavros/cmd/land");
    ros::ServiceClient set_mode_client_uav3 = nh_uav3.serviceClient<mavros_msgs::SetMode>("uav3/mavros/set_mode");

    ros::Subscriber state_sub_uav0 = nh_uav0.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, state_cb_uav0);
    ros::Publisher local_pos_pub_uav0 = nh_uav0.advertise<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_uav0 = nh_uav0.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming");
    ros::ServiceClient land_client_uav0 = nh_uav0.serviceClient<mavros_msgs::CommandTOL>("uav0/mavros/cmd/land");
    ros::ServiceClient set_mode_client_uav0 = nh_uav0.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode");

    ros::Rate rate(40.0);
    while(ros::ok() && current_state_uav1.connected && current_state_uav2.connected && current_state_uav3.connected && current_state_uav0.connected){
        ros::spinOnce();rate.sleep();ROS_INFO("connecting to FCT...");}ROS_INFO("CONNECTED");

    pose_uav0.pose.position.x = 1;
    pose_uav0.pose.position.y = 1;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav1.pose.position.x = -1;
    pose_uav1.pose.position.y = 1;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav2.pose.position.x = -1;
    pose_uav2.pose.position.y = -1;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav3.pose.position.x = 1;
    pose_uav3.pose.position.y = -1;
    pose_uav3.pose.position.z = FLIGHT_ALTITUDE;



    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){

        local_pos_pub_uav3.publish(pose_uav3);
        local_pos_pub_uav2.publish(pose_uav2);
        local_pos_pub_uav1.publish(pose_uav1);
        local_pos_pub_uav0.publish(pose_uav0);
        ROS_INFO("PUBLISHIG 1 SETPOINS");
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode_uav0;
    offb_set_mode_uav0.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd_uav0;
    arm_cmd_uav0.request.value = true;

    mavros_msgs::CommandTOL land_cmd_uav0;
    land_cmd_uav0.request.yaw = 0;
    land_cmd_uav0.request.latitude = 0;
    land_cmd_uav0.request.longitude = 0;
    land_cmd_uav0.request.altitude = 0;


//uav1 mode and commands
    mavros_msgs::SetMode offb_set_mode_uav1;
    offb_set_mode_uav1.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd_uav1;
    arm_cmd_uav1.request.value = true;
    mavros_msgs::CommandTOL land_cmd_uav1;
    land_cmd_uav1.request.yaw = 0;
    land_cmd_uav1.request.latitude = 0;
    land_cmd_uav1.request.longitude = 0;
    land_cmd_uav1.request.altitude = 0;

//uav2 mode and commands
    mavros_msgs::SetMode offb_set_mode_uav2;
    offb_set_mode_uav2.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd_uav2;
    arm_cmd_uav2.request.value = true;

    mavros_msgs::CommandTOL land_cmd_uav2;
    land_cmd_uav2.request.yaw = 0;
    land_cmd_uav2.request.latitude = 0;
    land_cmd_uav2.request.longitude = 0;
    land_cmd_uav2.request.altitude = 0;

//uav3 mode and commands

    mavros_msgs::SetMode offb_set_mode_uav3;
    offb_set_mode_uav3.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd_uav3;
    arm_cmd_uav3.request.value = true;

    mavros_msgs::CommandTOL land_cmd_uav3;
    land_cmd_uav3.request.yaw = 0;
    land_cmd_uav3.request.latitude = 0;
    land_cmd_uav3.request.longitude = 0;
    land_cmd_uav3.request.altitude = 0;


    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm
    while(ros::ok() && !current_state_uav2.armed && 
        !current_state_uav1.armed && !current_state_uav3.armed && !current_state_uav0.armed){
        if( current_state_uav3.mode != "OFFBOARD" && 
            current_state_uav1.mode != "OFFBOARD" && 
            current_state_uav2.mode != "OFFBOARD" &&current_state_uav0.mode != "OFFBOARD"&&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
          ROS_INFO(current_state_uav1.mode.c_str());
          ROS_INFO(current_state_uav2.mode.c_str());
            if(( set_mode_client_uav3.call(offb_set_mode_uav3) &&
                offb_set_mode_uav3.response.mode_sent) &&
                    ( set_mode_client_uav0.call(offb_set_mode_uav0) &&
                                    offb_set_mode_uav0.response.mode_sent) &&
                (set_mode_client_uav1.call(offb_set_mode_uav1) &&
             offb_set_mode_uav1.response.mode_sent) && 
                set_mode_client_uav2.call(offb_set_mode_uav2) &&
                offb_set_mode_uav2.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state_uav1.armed && !current_state_uav2.armed && !current_state_uav3.armed && !current_state_uav0.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( ( arming_client_uav3.call(arm_cmd_uav3) &&
                    arm_cmd_uav3.response.success) && 
                        ( arming_client_uav0.call(arm_cmd_uav0) &&
                                            arm_cmd_uav0.response.success) &&
                    ( arming_client_uav1.call(arm_cmd_uav1)
                     && arm_cmd_uav1.response.success) &&
                      arming_client_uav2.call(arm_cmd_uav2) &&
                    arm_cmd_uav2.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub_uav1.publish(pose_uav1);
        local_pos_pub_uav2.publish(pose_uav2);
        local_pos_pub_uav3.publish(pose_uav3);
        local_pos_pub_uav0.publish(pose_uav0);

        ros::spinOnce();
        rate.sleep();
    }

    // go to the first waypoint
    pose_uav0.pose.position.x = 5;
    pose_uav0.pose.position.y = 5;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav1.pose.position.x = -5;
    pose_uav1.pose.position.y = 5;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav2.pose.position.x =-5;
    pose_uav2.pose.position.y = -5;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav3.pose.position.x = 5;
    pose_uav3.pose.position.y = -5;
    pose_uav3.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);
      local_pos_pub_uav3.publish(pose_uav3);
      local_pos_pub_uav0.publish(pose_uav0);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");


    // go to the second waypoint
    pose_uav0.pose.position.x = -5;
    pose_uav0.pose.position.y = 5;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav1.pose.position.x = -5;
    pose_uav1.pose.position.y = -5;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav2.pose.position.x =  5;
    pose_uav2.pose.position.y = -5;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav3.pose.position.x =  5;
    pose_uav3.pose.position.y = 5;
    pose_uav3.pose.position.z = FLIGHT_ALTITUDE;

    //send setpoints for 10 seconds
    ROS_INFO("going to second way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);
      local_pos_pub_uav3.publish(pose_uav3);
      local_pos_pub_uav0.publish(pose_uav0);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("second way point finished!");

    // go to the third waypoint
    pose_uav0.pose.position.x = -5;
    pose_uav0.pose.position.y = -5;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav1.pose.position.x = 5;
    pose_uav1.pose.position.y = -5;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav2.pose.position.x =  5;
    pose_uav2.pose.position.y = 5;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav3.pose.position.x =-5;
    pose_uav3.pose.position.y = 5;
    pose_uav3.pose.position.z = FLIGHT_ALTITUDE;

    //send setpoints for 10 seconds
    ROS_INFO("going to third way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);
      local_pos_pub_uav3.publish(pose_uav3);
      local_pos_pub_uav0.publish(pose_uav0);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("third way point finished!");
    
    // go to the forth waypoint
    pose_uav0.pose.position.x = 5;
    pose_uav0.pose.position.y = -5;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav1.pose.position.x = 5;
    pose_uav1.pose.position.y = 5;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav2.pose.position.x =  -5;
    pose_uav2.pose.position.y = 5;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav3.pose.position.x =-5;
    pose_uav3.pose.position.y = -5;
    pose_uav3.pose.position.z = FLIGHT_ALTITUDE;

    //send setpoints for 10 seconds
    ROS_INFO("going to forth way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);
      local_pos_pub_uav3.publish(pose_uav3);
      local_pos_pub_uav0.publish(pose_uav0);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("forth way point finished!");

    pose_uav0.pose.position.x = 1;
    pose_uav0.pose.position.y = 1;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav1.pose.position.x = -1;
    pose_uav1.pose.position.y = 1;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav2.pose.position.x = -1;
    pose_uav2.pose.position.y = -1;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;
    pose_uav3.pose.position.x = 1;
    pose_uav3.pose.position.y = -1;
    pose_uav3.pose.position.z = FLIGHT_ALTITUDE;
    ROS_INFO("going back to the first point!");
    //send setpoints for 10 seconds
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);
      local_pos_pub_uav3.publish(pose_uav3);
      local_pos_pub_uav0.publish(pose_uav0);

      ros::spinOnce();
      rate.sleep();
    }

    ROS_INFO("trying to land");
 /* As to land all UAV simultaneously the not landing condition should be checked combined instead of 
   individual UAV as In that case Only first land and other entered into failsafe model*/
    while (!(land_client_uav1.call(land_cmd_uav1) && land_cmd_uav1.response.success &&
             land_client_uav1.call(land_cmd_uav2) && land_cmd_uav2.response.success &&
             land_client_uav3.call(land_cmd_uav3) && land_cmd_uav3.response.success)){
      /* ( !(land_client_uav1.call(land_cmd_uav1) && land_cmd_uav1.response.success)) &&
        ( !(land_client_uav2.call(land_cmd_uav2) && land_cmd_uav2.response.success) )&&
            ( !(land_client_uav0.call(land_cmd_uav0) && land_cmd_uav0.response.success) )&&
        ( !(land_client_uav3.call(land_cmd_uav3) && land_cmd_uav3.response.success))){*/

      local_pos_pub_uav1.publish(pose_uav1);  
      local_pos_pub_uav2.publish(pose_uav2);  
      local_pos_pub_uav3.publish(pose_uav3);
      local_pos_pub_uav0.publish(pose_uav0);
      ROS_INFO("trying to land");
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
