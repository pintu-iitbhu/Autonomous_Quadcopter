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
long double lat4=(25.2598519); long double long4=(82.9883528);                                             //                                      |
//////////second drone                                                                                                              S <-------------------> N
int m=4;                                                                                          //                                            |
long double dflatmu=(lat1-lat2)/m;         long double dflongmu=(long1-long2)/m;                  //                                            |
long double dflatmd=(lat3-lat4)/m;         long double dflongmd=(long3-long4)/m;                 //                                             |
long double d2lat1= lat1-(1*(dflatmu));    long double d2long1= long1-(1*(dflongmu));                          // 1     2                                     E
long double d2lat2= (d2lat1-dflatmu);      long double d2long2= (d2long2-dflongmu);
long double d2lat3= lat3-(1*(dflatmd));    long double d2long3= long3-(1*(dflongmd));                           // 3     4
long double d2lat4= (d2lat3-dflatmd);      long double d2long4= (d2long3-dflongmd);
//////////////no of times /////////////////////////
int n=5;
long double  dflatnu=dflatmu/n;    long double  dflongnu=dflongmu/n;                // 1    2      3      4     5 
long double  dflatnd=dflatmd/n;    long double  dflongnd=dflongmd/n; 
long double  d2plat1=(d2lat1);             long double  d2plong1= (d2long1);
long double  d2plat2=(d2plat1-dflatnu);    long double  d2plong2= (d2plong1-dflongnu);
long double  d2plat3=(d2plat2-dflatnu);    long double  d2plong3= (d2plong2-dflongnu);
long double  d2plat4=(d2plat3-dflatnu);    long double  d2plong4= (d2plong3-dflongnu);
long double  d2plat5=(d2plat4-dflatnu);    long double  d2plong5= (d2plong4-dflongnu);
long double  d2plat6=(d2lat3);             long double  d2plong6=  (d2long3);                       //  6    7       8      9      10
long double  d2plat7=(d2plat6-dflatnd);    long double  d2plong7= (d2plong6-dflongnd);
long double  d2plat8=(d2plat7-dflatnd);    long double  d2plong8= (d2plong7-dflongnd);
long double  d2plat9=(d2plat8-dflatnd);    long double  d2plong9= (d2plong8-dflongnd);
long double  d2plat10=(d2plat9-dflatnd);   long double  d2plong10= (d2plong9-dflongnd);
///// final cordinates ///////////////////////////////////////////////
long double p12lat1=((d2plat6+d2plat7)/2);  long double p12long1=((d2plong6+d2plong7)/2);
long double p12lat2=((d2plat1+d2plat2)/2);  long double p12long2=((d2plong1+d2plong2)/2);
long double p12lat3=((d2plat2+d2plat3)/2);  long double p12long3=((d2plong2+d2plong3)/2);
long double p12lat4=((d2plat7+d2plat8)/2);  long double p12long4=((d2plong7+d2plong8)/2);
long double p12lat5=((d2plat8+d2plat9)/2);  long double p12long5=((d2plong8+d2plong9)/2);
long double p12lat6=((d2plat3+d2plat4)/2);  long double p12long6=((d2plong3+d2plong4)/2);
long double p12lat7=((d2plat4+d2plat5)/2);  long double p12long7=((d2plong4+d2plong5)/2);
long double p12lat8=((d2plat9+d2plat10)/2); long double p12long8=((d2plong9+d2plong10)/2);


sensor_msgs::NavSatFix global_pose;
void global_pose_cb(const sensor_msgs::NavSatFixConstPtr& msg){global_pose = *msg;}//ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);}
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){current_state = *msg;}//ROS_INFO("got status: %d, %d", (int)msg->armed, (int)msg->connected);}

std_msgs::Int8 Boxstatus2;
void state_box2(const std_msgs::Int8::ConstPtr& msg2){Boxstatus2= *msg2;ROS_INFO("I heard: [%d]", Boxstatus2);}

int main(int argc, char **argv)
{
  int rate_hz = 10;
  ros::init(argc, argv, "global_pos_mission");
  ros::NodeHandle n;
  ros::NodeHandle& nh = n;
  ros::Rate rate(rate_hz);
  ros::Subscriber s_box2 = n.subscribe<std_msgs::Int8>("/opencv2", 10, state_box2);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode", 1);
  ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("uav1/mavros/global_position/global", 1, global_pose_cb);
  ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
  ros::ServiceClient wp_clear_client = n.serviceClient<mavros_msgs::WaypointClear>("uav1/mavros/mission/clear");
  ros::ServiceClient client_wp = n.serviceClient<mavros_msgs::WaypointPush>("uav1/mavros/mission/push");
  ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");

  global_pose.header.seq = 0;
  
  while(ros::ok() && (!current_state.connected || global_pose.header.seq == 0)) {ros::spinOnce(); rate.sleep(); }
  ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);

  
  mavros_msgs::CommandBool srv; srv.request.value = true;
  if(arming_cl.call(srv)){ ROS_INFO("ARM send ok %d", srv.response.success);}else{ROS_ERROR("Failed arming or disarming");}
//////////CLEAR MISSION ///////////////////////////////
  mavros_msgs::WaypointClear wp_clear_srv;
  if (wp_clear_client.call(wp_clear_srv)){ ROS_INFO("Waypoint list was cleared"); } else{ROS_ERROR("Waypoint list couldn't been cleared");}
///////// MISSION///////////////////////////////////////////////////////
  
  mavros_msgs::WaypointPush srv_wp; srv_wp.request.start_index = 0; mavros_msgs::CommandHome set_home_srv; mavros_msgs::Waypoint wp;
  double cur_pose_lat  = global_pose.latitude;
  double cur_pose_long = global_pose.longitude;
  // fill wp
  wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
  wp.is_current   = true;
  wp.autocontinue = true;
  wp.param1       = 3;
  wp.z_alt        = 5;
  wp.x_lat        = global_pose.latitude;
  wp.y_long       = global_pose.longitude;
  srv_wp.request.waypoints.push_back(wp);
  /*wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp.is_current   = false;
  wp.autocontinue = true;
  wp.param1       = 2;          // hold time sec
  wp.z_alt        = 4.5;
  wp.x_lat        = p12lat1;              //1    
  wp.y_long       = p12long1;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = p12lat2;              //2
  wp.y_long       = p12long2;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = p12lat3;            //3
  wp.y_long       = p12long3;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = p12lat4;             //4  
  wp.y_long       = p12long4;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = p12lat5;            //5
  wp.y_long       = p12long5;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = p12lat6;              //6 
  wp.y_long       = p12long6;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = p12lat7;            //7
  wp.y_long       = p12long7;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = p12lat8;             //8
  wp.y_long       = p12long8;
  srv_wp.request.waypoints.push_back(wp);*/
  wp.command      = mavros_msgs::CommandCode::NAV_LAND;
  wp.is_current   = false;
  wp.autocontinue = true;
  wp.z_alt        = 0;
  wp.x_lat        = cur_pose_lat ;
  wp.y_long       = cur_pose_long;
  srv_wp.request.waypoints.push_back(wp);

  if (client_wp.call(srv_wp)){ROS_INFO("Uploaded WPs!");mavros_msgs::State current_state;}else{ROS_ERROR("Upload Failed");}
////////////////////////AUTO MISSION MODE ///////////////////////////////////
  mavros_msgs::SetMode srv_setMode;srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  srv_setMode.request.custom_mode = "AUTO.MISSION";
  if(cl.call(srv_setMode)){ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);}else{ROS_ERROR("Failed SetMode");return -1;}
/////////////////////////WAIT TILL DISARM ////////////////////////////////////
  current_state.armed = true;
  while(ros::ok() && current_state.armed ){ros::spinOnce(); rate.sleep(); 
  if(Boxstatus2.data == 1){
   ROS_INFO("BOXdetected  [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude);
      ROS_INFO("RTL");
      mavros_msgs::SetMode srv_setMode;
      srv_setMode.request.base_mode = 0;
      srv_setMode.request.custom_mode = "AUTO.RTL";
      if(cl.call(srv_setMode)){ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);}
      else{ROS_ERROR("Failed SetMode");return -1;}
      break;}}
  ROS_INFO("Drone disarmed");
  return 0;
}


