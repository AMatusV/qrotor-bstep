#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include "controller/FloatList.h"
#include "controller/ControllerConfig.h"


////////////////////////////////////
// Private Vars
////////////////////////////////////

// Primary controller input & output variables
double setpoint[3];
double plant_state[3];  // x, y, z
bool controller_enabled = false;  // Position controller is not enabled to run
std::vector<double> control_effort(3, 0);

// Controller gains
double Kr[3], Kv[3], Gm[3];

// Node frequency
double frequency = 20;

// Used in the program logic
bool first_reconfig = true;
ros::Duration delta_t;
ros::Time prev_time;

// Cutoff frequency for the derivative calculation in Hz.
// Negative -> Has not been set by the user yet, so use a default.
double cutoff_frequency = -1;

// Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
// 1/4 of the sample rate.
double c = 1.;

// Used to check for tan(0)==>NaN in the filter calculation
double tan_filt = 1.;

// Upper and lower saturation limits
double upper_limit[3], lower_limit[3];

// Anti-windup term. Limits the absolute value of the integral terms
double windup_limit[3];

// Initialize filter data with zeros
double error[3][3], filtered_error[3][3], error_deriv[3][3], filtered_error_deriv[3][3];
//double euler[3][3], filtered_euler[3][3];
double error_vel_int[3];

// Topic and node names and message objects
std::string topic_from_position, topic_from_plant, setpoints_topic, controller_enable_topic, node_name = "position_node";
//std::string topic_virtual_control;
ros::Publisher control_effort_pub;
//ros::Publisher virtual_control_pub;
controller::FloatList control_msg;

// Dynamical system parameters
double mass;



void setpoint_callback(const controller::FloatList& setpoint_msg)
{
   setpoint[0] = setpoint_msg.data[0];  // desired x
   setpoint[1] = setpoint_msg.data[1];  // desired y
   setpoint[2] = setpoint_msg.data[2];  // desired z
}


void plant_state_callback(const geometry_msgs::TransformStamped& state_msg)
{
   plant_state[0] = state_msg.transform.translation.x;
   plant_state[1] = state_msg.transform.translation.y;
   plant_state[2] = state_msg.transform.translation.z;

   /*tf::Quaternion quater;
   tf::quaternionMsgToTF(state_msg.transform.rotation, quater);
   tf::Matrix3x3(quater).getRPY(plant_state[3], plant_state[4], plant_state[5]);*/

   //ROS_INFO("XYZ: %f, %f, %f", plant_state[0], plant_state[1], plant_state[2]);
}


/*void angles_callback(const controller::FloatList& angles_msg)
{
   plant_state[3] = angles_msg.data[0];
   plant_state[4] = angles_msg.data[1];
   plant_state[5] = angles_msg.data[2];

   //ROS_INFO("RPY: %f, %f, %f, plant_state[3], plant_state[4], plant_state[5]);
}*/


void controller_enable_callback(const std_msgs::Bool& controller_enable_msg)
{
   controller_enabled = controller_enable_msg.data;
}


void get_params(double in, double &value)
{
   value = in;
}


void reconfigure_callback(controller::ControllerConfig &config, uint32_t level)
{
   if (first_reconfig) {
      get_params(Kr[0], config.Kp1);
      get_params(Kr[1], config.Kp2);
      get_params(Kr[2], config.Kp3);
      get_params(Kv[0], config.Kd1);
      get_params(Kv[1], config.Kd2);
      get_params(Kv[2], config.Kd3);
      get_params(Gm[0], config.Gm1);
      get_params(Gm[1], config.Gm2);
      get_params(Gm[2], config.Gm3);
      first_reconfig = false;
      return;  // Ignore the first call to reconfigure which happens at startup
   }

   Kr[0] = config.Kp1;
   Kr[1] = config.Kp2;
   Kr[2] = config.Kp3;
   Kv[0] = config.Kd1;
   Kv[1] = config.Kd2;
   Kv[2] = config.Kd3;
	Gm[0] = config.Gm1;
   Gm[1] = config.Gm2;
   Gm[2] = config.Gm3;
	ROS_INFO("Position reconf");
   ROS_INFO("Kr: %2.4f, %2.4f, %2.4f", Kr[0], Kr[1], Kr[2]);
   ROS_INFO("Kv: %2.4f, %2.4f, %2.4f", Kv[0], Kv[1], Kv[2]);
   ROS_INFO("Gm: %2.4f, %2.4f, %2.4f", Gm[0], Gm[1], Gm[2]);
}


////////////////////////////////////
// Error checking
////////////////////////////////////
bool validate_parameters()
{
	for (int i = 0; i < 3; i++){
      if ( lower_limit[i] > upper_limit[i] ) {
         ROS_ERROR("The lower saturation limit cannot be greater than the upper saturation limit.");
         return(false);
      }
   }

   return true;
}


////////////////////////////////////
// Display parameters
////////////////////////////////////
void print_parameters()
{
   std::cout<< std::endl<<"CONTROLLER PARAMETERS"<<std::endl<<"-----------------------------------------"<<std::endl;
   std::cout << "Kr: { " << Kr[0] << ", " << Kr[1] << ", " << Kr[2] << " }" << std::endl;
   std::cout << "Kv: { " << Kv[0] << ", " << Kv[1] << ", " << Kv[2] << " }" << std::endl;
   if ( cutoff_frequency== -1) // If the cutoff frequency was not specified by the user
      std::cout<<"LPF cutoff frequency: 1/4 of sampling rate"<<std::endl;
   else
      std::cout<<"LPF cutoff frequency: "<< cutoff_frequency << std::endl;
   std::cout << "Controller node name: " << ros::this_node::getName() << std::endl;
   std::cout << "Name of topic from controller: " << topic_from_position << std::endl;
   std::cout << "Name of topic from the plant: " << topic_from_plant << std::endl;
   std::cout << "Name of setpoint topic: " << setpoints_topic << std::endl;
   std::cout << "Integral-windup limits: " << windup_limit[0] << ", " << windup_limit[1] << ", " 
				 << windup_limit[2] << std::endl;
   //std::cout << "Saturation limits angle: " << upper_limit[0] << "/" << lower_limit[0] << std::endl;
   std::cout << "X-axis saturation limits: " << upper_limit[0] << "/" << lower_limit[0] << std::endl;
	std::cout << "Y-axis saturation limits: " << upper_limit[1] << "/" << lower_limit[1] << std::endl;
	std::cout << "Z-axis saturation limits: " << upper_limit[2] << "/" << lower_limit[2] << std::endl;
   std::cout << "-----------------------------------------" << std::endl;

   return;
}


////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
   ROS_INFO("Starting pos. controller with node name %s", node_name.c_str());

   // Initialize ROS stuff
   ros::init(argc, argv, node_name);  // Note node_name can be overidden by launch file
   ros::NodeHandle node;
   ros::NodeHandle node_priv("~");

   while (ros::Time(0) == ros::Time::now()) {
      ROS_INFO("controller spinning waiting for time to become non-zero");
      sleep(1);
   }

   // Get params if specified in launch file or as params on command-line, set defaults
   node_priv.param<double>("Kr1", Kr[0], 1.0);
   node_priv.param<double>("Kr2", Kr[1], 0.0);
   node_priv.param<double>("Kr3", Kr[2], 0.0);
   node_priv.param<double>("Kv1", Kv[0], 1.0);
   node_priv.param<double>("Kv2", Kv[1], 0.0);
   node_priv.param<double>("Kv3", Kv[2], 0.0);
   node_priv.param<double>("Gm1", Gm[0], 0.0);
   node_priv.param<double>("Gm2", Gm[1], 0.0);
   node_priv.param<double>("Gm3", Gm[2], 0.0);
   node_priv.param<double>("cutoff_frequency", cutoff_frequency, -1.0);
   node_priv.param<double>("frequency", frequency, 20.0);
   //node_priv.param<double>("upper_limit_angle", upper_limit[0], 0.35);
   //node_priv.param<double>("lower_limit_angle", lower_limit[0], -0.35);
	node_priv.param<double>("x_windup_limit", windup_limit[0], 1.0);
   node_priv.param<double>("y_windup_limit", windup_limit[1], 1.0);
   node_priv.param<double>("z_windup_limit", windup_limit[2], 1.0);
   node_priv.param<double>("x_upper_limit", upper_limit[0], 1.0);
   node_priv.param<double>("x_lower_limit", lower_limit[0], -1.0);
	node_priv.param<double>("y_upper_limit", upper_limit[1], 1.0);
   node_priv.param<double>("y_lower_limit", lower_limit[1], -1.0);
	node_priv.param<double>("z_upper_limit", upper_limit[2], 1.0);
   node_priv.param<double>("z_lower_limit", lower_limit[2], -1.0);
   node_priv.param<double>("mass", mass, 0.72);
   node_priv.param<std::string>("topic_from_plant", topic_from_plant, "state");
   node_priv.param<std::string>("setpoints_topic", setpoints_topic, "setpoints");
   node_priv.param<std::string>("controller_enable_topic", controller_enable_topic, "position_enable");
   node_priv.param<std::string>("topic_from_position", topic_from_position, "position_control");
   //node_priv.param<std::string>("topic_virtual_control", topic_virtual_control, "virtual_control");

   // Update params if specified as command-line options, & print settings
   print_parameters();
   if (not validate_parameters()) {
      std::cout << "Error: invalid parameter\n";
   }

   // instantiate publishers & subscribers
   control_effort_pub = node.advertise<controller::FloatList>(topic_from_position, 1);

   ros::Subscriber sub = node.subscribe(topic_from_plant, 1, plant_state_callback );
   //ros::Subscriber angles_sub = node.subscribe("angles", 1, angles_callback );
   ros::Subscriber setpoint_sub = node.subscribe(setpoints_topic, 1, setpoint_callback );
   ros::Subscriber controller_enabled_sub = node.subscribe(controller_enable_topic, 1, controller_enable_callback );

   // configure dynamic reconfiguration
   dynamic_reconfigure::Server<controller::ControllerConfig> config_server;
   dynamic_reconfigure::Server<controller::ControllerConfig>::CallbackType f;
   f = boost::bind(&reconfigure_callback, _1, _2);
   config_server.setCallback(f);

   // Wait for first messages
   while( !ros::topic::waitForMessage<controller::FloatList>(setpoints_topic, ros::Duration(10.)) )
      ROS_WARN_STREAM("Waiting for the setpoint to be published.");
   while( !ros::topic::waitForMessage<geometry_msgs::TransformStamped>(topic_from_plant, ros::Duration(10.)) )
      ROS_WARN_STREAM("Waiting for a msg on the state of the plant.");
   /*while( !ros::topic::waitForMessage<controller::FloatList>("angles", ros::Duration(10.)) )
      ROS_WARN_STREAM("Waiting for a msg on the angles.");*/

   ros::Rate loop_rate(frequency);

   while (ros::ok()) {
      if ( !(Kr[0]>=0. && Kr[1]>=0. && Kr[2]>=0. && Kv[0]>=0. && Kv[1]>=0. && Kv[2]>=0.) ) { // All 3 gains should have the same sign
         ROS_WARN("All gains (Ke, Kv) should have the same sign for stability.");
      }

      // Calculate delta_t
      if (!prev_time.isZero()) {  // Not first time through the program
         delta_t = ros::Time::now() - prev_time;
         prev_time = ros::Time::now();
         if (0 == delta_t.toSec()) {
            ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
            continue;
         }
      }
      else {
         ROS_INFO("prev_time is 0, doing nothing");
         prev_time = ros::Time::now();
         continue;
      }

      // Filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
      if (cutoff_frequency != -1) {
         // Check if tan(_) is really small, could cause c = NaN
         tan_filt = tan( (cutoff_frequency*6.2832)*delta_t.toSec()/2 );

         // Avoid tan(0) ==> NaN
         if ( (tan_filt<=0.) && (tan_filt>-0.01) )
            tan_filt = -0.01;
         if ( (tan_filt>=0.) && (tan_filt<0.01) )
            tan_filt = 0.01;

         c = 1/tan_filt;
      }

      for (int i = 0; i < 3; i++) {  // x, y, z
         error[2][i] = error[1][i];
         error[1][i] = error[0][i];
         error[0][i] = setpoint[i] - plant_state[i];

         filtered_error[2][i] = filtered_error[1][i];
         filtered_error[1][i] = filtered_error[0][i];
         filtered_error[0][i] = (1/(1+c*c+1.414*c))*(error[2][i]+2*error[1][i]+error[0][i]-(c*c-1.414*c+1)*filtered_error[2][i]-(-2*c*c+2)*filtered_error[1][i]);

         // Take derivative of error
         // First the raw, unfiltered data:
         error_deriv[2][i] = error_deriv[1][i];
         error_deriv[1][i] = error_deriv[0][i];
         error_deriv[0][i] = (error[0][i]-error[1][i])/delta_t.toSec();

         filtered_error_deriv[2][i] = filtered_error_deriv[1][i];
         filtered_error_deriv[1][i] = filtered_error_deriv[0][i];
         filtered_error_deriv[0][i] = (1/(1+c*c+1.414*c))*(error_deriv[2][i]+2*error_deriv[1][i]+error_deriv[0][i]-(c*c-1.414*c+1)*filtered_error_deriv[2][i]-(-2*c*c+2)*filtered_error_deriv[1][i]);

			// Integrate the velocity error
			error_vel_int[i] += filtered_error_deriv[0][i];

         // Apply windup limit to limit the size of the integral term
  			if ( error_vel_int[i] > fabsf(windup_limit[i]))
    			error_vel_int[i] = fabsf(windup_limit[i]);

  			if ( error_vel_int[i] < -fabsf(windup_limit[i]))
    			error_vel_int[i] = -fabsf(windup_limit[i]);
      }
 
      double err[3] = {filtered_error[0][0], filtered_error[0][1], filtered_error[0][2]};
      double derr[3] = {filtered_error_deriv[0][0], filtered_error_deriv[0][1], filtered_error_deriv[0][2]};
    
      /*error_vel_int[0] += derr[0];
      error_vel_int[1] += derr[1];
      error_vel_int[2] += derr[2];*/

      control_effort[0] = Gm[0]/mass*error_vel_int[0] + mass*( (1 + Kr[0]*Kv[0])*err[0] + Kv[0]*derr[0] );
      control_effort[1] = Gm[1]/mass*error_vel_int[1] + mass*( (1 + Kr[1]*Kv[1])*err[1] + Kv[1]*derr[1] );
      control_effort[2] = Gm[2]/mass*error_vel_int[2] + mass*( (1 + Kr[2]*Kv[2])*err[2] + Kv[2]*derr[2] );

      // Apply saturation limits
      for (int i = 0; i < 3; i++) {
         if (control_effort[i] > upper_limit[i])
           control_effort[i] = upper_limit[i];
         else if (control_effort[i] < lower_limit[i])
            control_effort[i] = lower_limit[i];
      }

      if (controller_enabled) {
         control_msg = controller::FloatList();
         control_msg.data = control_effort;
         control_effort_pub.publish(control_msg);
      }
      else {
         error_vel_int[0] = 0;
         error_vel_int[1] = 0;
         error_vel_int[2] = 0;
      }

      ros::spinOnce();
      loop_rate.sleep();
   }
   return 0;
}

