#include <ros/ros.h>
#include <cstdlib>
#include "math.h"

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

long double lat1=(25.2593945); long double long1=(82.9880905);                                         //    1               2                 W
long double lat2=(25.2597385);  long double long2=(82.9879240);                                        //                                      |
long double lat3=(25.2595144); long double long3=(82.9884665);                                         //    3               4                 |
long double lat4=(25.2598519); long double long4=(82.9883528);                                         //                                      |
//////////first drone                                                                                                              S <-------------------> N
int m=4;                                                                                         //                                            |
long double dflatmu=(lat1-lat2)/m;        long double dflongmu=(long1-long2)/m;                  //                                            |
long double dflatmd=(lat3-lat4)/m;        long double dflongmd=(long3-long4)/m;                 //                                             |
long double d1lat1=lat1;                  long double d1long1= long1;                           // 1     2                                     E
long double d1lat2=d1lat1-dflatmu;        long double d1long2= d1long1-dflongmu;
long double d1lat3=lat3;                  long double d1long3= long3;                           // 3     4
long double d1lat4=d1lat3-dflatmd;        long double d1long4= d1long3-dflongmd;
//////////////no of times /////////////////////////
int n=5;
long double  dflatnu=dflatmu/n;    long double  dflongnu=dflongmu/n;                // 1    2      3      4     5 
long double  dflatnd=dflatmd/n;    long double  dflongnd=dflongmd/n; 
long double  d1plat1=(d1lat1);             long double  d1plong1= (d1long1);
long double  d1plat2=(d1plat1-dflatnu);    long double  d1plong2= (d1plong1-dflongnu);
long double  d1plat3=(d1plat2-dflatnu);    long double  d1plong3= (d1plong2-dflongnu);
long double  d1plat4=(d1plat3-dflatnu);    long double  d1plong4= (d1plong3-dflongnu);
long double  d1plat5=(d1plat4-dflatnu);    long double  d1plong5= (d1plong4-dflongnu);
long double  d1plat6=(d1lat3);             long double  d1plong6=  (d1long3);                       //  6    7       8      9      10
long double  d1plat7=(d1plat6-dflatnd);    long double  d1plong7= (d1plong6-dflongnd);
long double  d1plat8=(d1plat7-dflatnd);    long double  d1plong8= (d1plong7-dflongnd);
long double  d1plat9=(d1plat8-dflatnd);    long double  d1plong9= (d1plong8-dflongnd);
long double  d1plat10=(d1plat9-dflatnd);   long double  d1plong10= (d1plong9-dflongnd);
////////////////////// final cordinates ///////////////////////////////////////////////
long double p11lat1=((d1plat6+d1plat7)/2);  long double p11long1=((d1plong6+d1plong7)/2);
long double p11lat2=((d1plat1+d1plat2)/2);  long double p11long2=((d1plong1+d1plong2)/2);
long double p11lat3=((d1plat2+d1plat3)/2);  long double p11long3=((d1plong2+d1plong3)/2);
long double p11lat4=((d1plat7+d1plat8)/2);  long double p11long4=((d1plong7+d1plong8)/2);
long double p11lat5=((d1plat8+d1plat9)/2);  long double p11long5=((d1plong8+d1plong9)/2);
long double p11lat6=((d1plat3+d1plat4)/2);  long double p11long6=((d1plong3+d1plong4)/2);
long double p11lat7=((d1plat4+d1plat5)/2);  long double p11long7=((d1plong4+d1plong5)/2);
long double p11lat8=((d1plat9+d1plat10)/2); long double p11long8=((d1plong9+d1plong10)/2);

sensor_msgs::NavSatFix global_pose_0;
void global_pose_cb_0(const sensor_msgs::NavSatFixConstPtr& msg0){global_pose_0 = *msg0;}//ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);}
mavros_msgs::State current_state_0;
void state_cb_0(const mavros_msgs::State::ConstPtr& msg1){current_state_0 = *msg1;}//ROS_INFO("got status: %d, %d", (int)msg->armed, (int)msg->connected);}

std_msgs::Int8 Boxstatus0;
void state_box0(const std_msgs::Int8::ConstPtr& msg2){Boxstatus0= *msg2;ROS_INFO("I heard: [%d]", Boxstatus0);}

int main(int argc, char **argv)
{
  int rate_hz = 10;
  ros::init(argc, argv, "uav1");
  ros::NodeHandle n_0; 
  ros::NodeHandle& nh_0 = n_0;
  ros::Rate rate(rate_hz);
  ros::Subscriber s_box0 = n_0.subscribe<std_msgs::Int8>("/opencv1", 10, state_box0);
  ros::Subscriber state_sub_0 = nh_0.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, state_cb_0);
  ros::Publisher local_pos_pub_0 = nh_0.advertise<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client_0 = nh_0.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming");
  ros::ServiceClient set_mode_client_0 = nh_0.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode", 1);
  ros::Subscriber global_pose_sub_0 = nh_0.subscribe<sensor_msgs::NavSatFix>("uav0/mavros/global_position/global", 1, global_pose_cb_0);
  ros::ServiceClient arming_cl_0 = n_0.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming");
  ros::ServiceClient wp_clear_client_0 = n_0.serviceClient<mavros_msgs::WaypointClear>("uav0/mavros/mission/clear");
  ros::ServiceClient client_wp_0 = n_0.serviceClient<mavros_msgs::WaypointPush>("uav0/mavros/mission/push");
  ros::ServiceClient cl_0 = n_0.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode");

  global_pose_0.header.seq = 0;
 
  while(ros::ok() && (!current_state_0.connected || global_pose_0.header.seq == 0)) {ros::spinOnce(); rate.sleep(); }
 // ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);
/////////////////////////////////////////////////////////
  mavros_msgs::CommandBool srv_0; srv_0.request.value = true;
  if(arming_cl_0.call(srv_0)){ ROS_INFO("ARM send ok %d", srv_0.response.success);}else{ROS_ERROR("Failed arming or disarming");}
//////////CLEAR MISSION /////////////////////////////////
  mavros_msgs::WaypointClear wp_clear_srv_0;
  if (wp_clear_client_0.call(wp_clear_srv_0)){ ROS_INFO("Waypoint list was cleared"); } else{ROS_ERROR("Waypoint list couldn't been cleared");}
///////// MISSION///////////////////////////////////////////////////////
  mavros_msgs::WaypointPush srv_wp_0; srv_wp_0.request.start_index = 0; mavros_msgs::CommandHome set_home_srv_0; mavros_msgs::Waypoint wp_0;
  double cur_pose_0_lat  = global_pose_0.latitude;
  double cur_pose_0_long = global_pose_0.longitude;
  // fill wp
  wp_0.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp_0.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
  wp_0.is_current   = true;
  wp_0.autocontinue = true;
  wp_0.param1       = 3;
  wp_0.z_alt        = 4.5;
  wp_0.x_lat        = global_pose_0.latitude;
  wp_0.y_long       = global_pose_0.longitude;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp_0.is_current   = false;
  wp_0.autocontinue = true;
  wp_0.param1       = 2;          // hold time sec
  wp_0.z_alt        = 4.5;
  wp_0.x_lat        = p11lat1;              //1    
  wp_0.y_long       = p11long1;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.x_lat        = p11lat2;              //2
  wp_0.y_long       = p11long2;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.x_lat        = p11lat3;            //3
  wp_0.y_long       = p11long3;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.x_lat        = p11lat4;             //4  
  wp_0.y_long       = p11long4;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.x_lat        = p11lat5;            //5
  wp_0.y_long       = p11long5;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.x_lat        = p11lat6;              //6 
  wp_0.y_long       = p11long6;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.x_lat        = p11lat7;            //7
  wp_0.y_long       = p11long7;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.x_lat        = p11lat8;             //8
  wp_0.y_long       = p11long8;
  srv_wp_0.request.waypoints.push_back(wp_0);
  wp_0.command      = mavros_msgs::CommandCode::NAV_LAND;
  wp_0.z_alt        = 0;
  wp_0.x_lat        = cur_pose_0_lat ;
  wp_0.y_long       = cur_pose_0_long;
  srv_wp_0.request.waypoints.push_back(wp_0);

  if (client_wp_0.call(srv_wp_0)){ROS_INFO("Uploaded WPs!");mavros_msgs::State current_state_0;}else{ROS_ERROR("Upload Failed");}
////////////////////////AUTO MISSION MODE ///////////////////////////////////
  mavros_msgs::SetMode srv_setMode_0;srv_setMode_0.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  srv_setMode_0.request.custom_mode = "AUTO.MISSION";
  if(cl_0.call(srv_setMode_0)){ROS_INFO("setmode send ok %d value:", srv_setMode_0.response.mode_sent);}else{ROS_ERROR("Failed SetMode");return -1;}
/////////////////////////WAIT TILL DISARM ////////////////////////////////////
  current_state_0.armed = true;
  while(ros::ok() && current_state_0.armed ){
ros::spinOnce();rate.sleep();
  if(Boxstatus0.data ==1){
   ROS_INFO("BOXdetected  [%d]: %f, %f, %f", global_pose_0.header.seq, global_pose_0.latitude, global_pose_0.longitude);
      ROS_INFO("RTL");
      mavros_msgs::SetMode srv_setMode_0;
      srv_setMode_0.request.base_mode = 0;
      srv_setMode_0.request.custom_mode = "AUTO.RTL";
      if(cl_0.call(srv_setMode_0)){ROS_INFO("setmode send ok %d value:", srv_setMode_0.response.mode_sent);}
      else{ROS_ERROR("Failed SetMode");return -1;}
      break;}}
  ROS_INFO("Drone disarmed");
  return 0;
}


