#include <ros/ros.h>
#include <ros/time.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <keyboard/Key.h>
#include "controller/SetpointsConfig.h"
#include "controller/FloatList.h"


////////////////////////////////////
// Private Vars
////////////////////////////////////

// Primary controller input & output variables
bool enabled;
//std::vector<double> setpoints(6, 0);
std::vector<double> setpoints(4, 0);  // x, y, z, yaw
double frequency;

// Topic and node names and message objects
std::string topic_from_keyboard, topic_setpoints, topic_enable;
controller::FloatList setpoints_msg;
std_msgs::Bool enabled_msg;
ros::Publisher setpoints_pub;
ros::Publisher enable_pub;


void keyboard_callback(const keyboard::Key& key_msg)
{
   uint16_t code;

   code = key_msg.code;
   
   if (code == 271) {  // alphanumeric "enter"
      enabled = !enabled;
      enabled_msg.data = enabled;
      enable_pub.publish(enabled_msg);
   }
}


void reconfigure_callback(controller::SetpointsConfig &config, uint32_t level) 
{
   // consider to ignore first call to reconfigure which happens at startup
   
   setpoints[0] = config.x_sp;
   setpoints[1] = config.y_sp;
   setpoints[2] = config.z_sp;
   //setpoints[3] = config.roll_sp;
   //setpoints[4] = config.pitch_sp;
   //setpoints[5] = config.yaw_sp;
   setpoints[3] = config.yaw_sp;

   /*setpoints_msg = controller::FloatList();
   setpoints_msg.data = setpoints;
   setpoints_pub.publish(setpoints_msg);*/

   ROS_INFO("Setpoints reconf: %2.4f, %2.4f, %2.4f, %2.4f", setpoints[0], setpoints[1], setpoints[2], setpoints[3]);
}


int main(int argc, char **argv)
{
   // Initialize ROS stuff
   ros::init(argc, argv, "setpoints");
   ros::NodeHandle node;
   ros::NodeHandle node_priv("~");

   while (ros::Time(0) == ros::Time::now()) {
      ROS_INFO("controller spinning waiting for time to become non-zero");
      sleep(1);
   }

   // Get params if specified in launch file or as params on command-line, set defaults
   node_priv.param<double>("SPx", setpoints[0], 0.0);
   node_priv.param<double>("SPy", setpoints[1], 0.0);
   node_priv.param<double>("SPz", setpoints[2], 0.0);
   node_priv.param<double>("SPyaw", setpoints[3], 0.0);
   node_priv.param<double>("frequency", frequency, 10);
   node_priv.param<std::string>("topic_from_keyboard", topic_from_keyboard, "keyup");
   node_priv.param<std::string>("topic_setpoints", topic_setpoints, "setpoints");
   node_priv.param<std::string>("topic_enable", topic_enable, "general_enable");

   // Instantiate publishers and subscribers
   setpoints_pub = node.advertise<controller::FloatList>(topic_setpoints, 1);
   enable_pub = node.advertise<std_msgs::Bool>(topic_enable, 1);

   ros::Subscriber key_sub = node.subscribe(topic_from_keyboard, 1, keyboard_callback);

   // Configure dynamic reconfiguration
   dynamic_reconfigure::Server<controller::SetpointsConfig> config_server;
   dynamic_reconfigure::Server<controller::SetpointsConfig>::CallbackType f;
   f = boost::bind(&reconfigure_callback, _1, _2);
   config_server.setCallback(f);

   //ROS_INFO("Spinning sp_node");

   ros::Rate loop_rate(frequency);
   while (ros::ok()) {
      setpoints_msg = controller::FloatList();
      setpoints_msg.data = setpoints;
      setpoints_pub.publish(setpoints_msg);

      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}
