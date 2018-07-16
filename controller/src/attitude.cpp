#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include "controller/ControllerConfig.h"
#include "controller/FloatList.h"


////////////////////////////////////
// Private Vars
////////////////////////////////////

// Primary controller input & output variables
double setpoint[3];
double plant_state[3][3];
double pos_control[3];
bool controller_enabled = false;  // Attitude controller is not enabled to run
std::vector<double> control_effort(4, 0);

ros::Time prev_time;
ros::Duration delta_t;
bool first_reconfig = true;

// Controller gains
double Ke[3], Ko[3], Gm[3];

// Parameters for error calc. with disconinuous input
bool angle_error = false;
double angle_wrap = 2.0*3.14159;

// Cutoff frequency for the derivative calculation in Hz.
// Negative -> Has not been set by the user yet, so use a default.
double cutoff_frequency = -1;

// Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
// 1/4 of the sample rate.
double c = 1.;

// Used to check for tan(0)==>NaN in the filter calculation
double tan_filt = 1.;

// Upper and lower saturation limits
double upper_limit[4], lower_limit[4];

// Anti-windup term. Limits the absolute value of the integral terms
double windup_limit[3];

// Initialize filter data with zeros
double error[3][3], filtered_error[3][3], error_deriv[3][3], filtered_error_deriv[3][3];
double euler_deriv[3][3], filtered_euler_deriv[3][3];
double ang_vel[3], error_omega[3], error_omega_int[3];

// Topic and node names and message objects
std::string topic_from_attitude, topic_from_plant, setpoints_topic;
std::string topic_from_position, controller_enable_topic, node_name = "attitude_node";
ros::Publisher control_effort_pub;
controller::FloatList control_msg;

// Diagnostic objects
//double min_loop_frequency = 1, max_loop_frequency = 1000;
//int measurements_received = 0;

// Dynamical system parameters
double inertia[3], mass;
double gravity = 9.81;



void setpoints_callback(const controller::FloatList& setpoint_msg)
{
    setpoint[2] = setpoint_msg.data[3];  // desired yaw
}


void position_callback(const controller::FloatList& position_msg)
{
	pos_control[0] = position_msg.data[0];
 	pos_control[1] = position_msg.data[1];
	pos_control[2] = position_msg.data[2];
}


void controller_enable_callback(const std_msgs::Bool& controller_enable_msg)
{
   controller_enabled = controller_enable_msg.data;
}


void plant_state_callback(const controller::FloatList& state_msg)
{
   if ( !(Ke[0]>=0. && Ke[1]>=0. && Ke[2]>=0. && Ko[0]>=0. && Ko[1]>=0. && Ko[2]>=0.) ) { // All 3 gains should have the same sign
      ROS_WARN("All gains (Ke, Kv) should have the same sign for stability.");
   }

   // Calculate delta_t
   if (!prev_time.isZero()) {  // Not first time through the program
      delta_t = ros::Time::now() - prev_time;
      prev_time = ros::Time::now();
      if (0 == delta_t.toSec()) {
         ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
         return;
      }
   }
   else {
      ROS_INFO("prev_time is 0, doing nothing");
      prev_time = ros::Time::now();
      return;
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

   // Compute roll and pitch setpoints and reference thrust
   setpoint[1] = atan( (pos_control[1]*sin(state_msg.data[2]) + pos_control[0]*cos(state_msg.data[2])) / (pos_control[2] + mass*gravity) );
	setpoint[0] = atan( (pos_control[0]*sin(state_msg.data[2]) - pos_control[1]*cos(state_msg.data[2]))*cos(setpoint[1]) / (pos_control[2] + mass*gravity) );
	control_effort[0] = (pos_control[2] + mass*gravity)/( cos(setpoint[1])*cos(setpoint[0]) );

	// B transpose
	double BT[3][3] = { { 1, 0, -sin(state_msg.data[1]) }, 
  							 { 0, cos(state_msg.data[0]), cos(state_msg.data[1])*sin(state_msg.data[0]) },
							 { 0, -sin(state_msg.data[0]), cos(state_msg.data[0])*cos(state_msg.data[1]) } }; 

   for (int i = 0; i < 3; i++) {  // roll, pitch, yaw
      plant_state[2][i] = plant_state[1][i];
      plant_state[1][i] = plant_state[0][i];
      plant_state[0][i] = state_msg.data[i];

      error[2][i] = error[1][i];
      error[1][i] = error[0][i];
      error[0][i] = setpoint[i] - plant_state[0][i];

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

      // apply filter to euler angles
      /*filtered_euler[2][i] = filtered_euler[1][i];
      filtered_euler[1][i] = filtered_euler[0][i];
      filtered_euler[0][i] = (1/(1+c*c+1.414*c))*(euler[2][i]+2*euler[1][i]+euler[0][i]-(c*c-1.414*c+1)*filtered_euler[2][i]-(-2*c*c+2)*filtered_euler[1][i]);*/

      // Estimating euler angles derivatives
      euler_deriv[2][i] = euler_deriv[1][i];
      euler_deriv[1][i] = euler_deriv[0][i];
      euler_deriv[0][i] = (plant_state[0][i] - plant_state[1][i])/delta_t.toSec();
      filtered_euler_deriv[2][i] = filtered_euler_deriv[1][i];
      filtered_euler_deriv[1][i] = filtered_euler_deriv[0][i];
      filtered_euler_deriv[0][i] = (1/(1+c*c+1.414*c))*(euler_deriv[2][i]+2*euler_deriv[1][i]+euler_deriv[0][i]-(c*c-1.414*c+1)*filtered_euler_deriv[2][i]-(-2*c*c+2)*filtered_euler_deriv[1][i]);

		// angular_velocity = B^T*euler_angle_derivative
		ang_vel[i]  = BT[i][0]*filtered_euler_deriv[0][0] + BT[i][1]*filtered_euler_deriv[0][1] + BT[i][2]*filtered_euler_deriv[0][2];
		error_omega[i] = BT[i][0]*filtered_error_deriv[0][0] + BT[i][1]*filtered_error_deriv[0][1] + BT[i][2]*filtered_error_deriv[0][2];

		// Integrate the velocity error
		error_omega_int[i] += error_omega[i]*delta_t.toSec();

		// Apply windup limit to limit the size of the integral terms
		if ( error_omega_int[i] > fabsf(windup_limit[i]))
   		error_omega_int[i] = fabsf(windup_limit[i]);

  		if ( error_omega_int[i] < -fabsf(windup_limit[i]))
    		error_omega_int[i] = -fabsf(windup_limit[i]);
   }

	//double roll = plant_state[0][0], pitch = plant_state[0][1], yaw = plant_state[0][2];
   double err[3] = {filtered_error[0][0], filtered_error[0][1], filtered_error[0][2]};
   double derr[3] = {filtered_error_deriv[0][0], filtered_error_deriv[0][1], filtered_error_deriv[0][2]};
   //double deuler[3] = {filtered_euler_deriv[0][0], filtered_euler_deriv[0][1], filtered_euler_deriv[0][2]};
	//double ang_vel[3], error_omega[3], B11, B13, B22, B23, B32, B33;

   // error_omega = inverse(B)*derror; B12, B21, B31 = 0
   //B11 = 1;  
   /*B13 = -sin(pitch);  // without filtering
   B22 = cos(roll);
   B23 = cos(pitch)*sin(roll);
   B32 = -sin(roll);
   B33 = cos(roll)*cos(pitch);*/
   /*B13 = -sin(filtered_euler[0][1]);  // with filtering
   B22 = cos(filtered_euler[0][0]);
   B23 = cos(filtered_euler[0][1])*sin(filtered_euler[0][0]);
   B32 = -sin(filtered_euler[0][0]);
   B33 = cos(filtered_euler[0][0])*cos(filtered_euler[0][1]);*/
   /*error_omega[0] = derr[0] + B13*derr[2];
   error_omega[1] = B22*derr[1] + B23*derr[2];
   error_omega[2] = B32*derr[1] + B33*derr[2];*/

	// Integrate angular velocity error
   /*error_omega_int[0] += error_omega[0]*delta_t.toSec();
   error_omega_int[1] += error_omega[1]*delta_t.toSec();
   error_omega_int[2] += error_omega[2]*delta_t.toSec();*/

   // Apply windup limit to limit the size of the integral terms
	/*for (int i = 0; i < 3; i++) {
		if ( error_omega_int[i] > fabsf(windup_limit))
   		error_omega_int[i] = fabsf(windup_limit);

  		if ( error_omega_int[i] < -fabsf(windup_limit))
    		error_omega_int[i] = -fabsf(windup_limit);
   }*/
  	
   // Angular velocity estimates, ang_vel = inverse(B)*euler_deriv
   /*angv[0] = euler_deriv[0][0] + B13*euler_deriv[0][2];  // without filtering
   angv[1] = B22*euler_deriv[0][1] + B23*euler_deriv[0][2];
   angv[2] = B32*euler_deriv[0][1] + B33*euler_deriv[0][2];*/
   /*ang_vel[0] = deuler[0] + B13*deuler[2];  // with filtering
   ang_vel[1] = B22*deuler[1] + B23*deuler[2];
   ang_vel[2] = B32*deuler[1] + B33*deuler[2];*/

   // BTackstepping method
   control_effort[1] = Gm[0]/inertia[0]*error_omega_int[0]
                       + ang_vel[1]*ang_vel[2]*(inertia[2] - inertia[1])
                       + inertia[0]*( (1 + Ke[0]*Ko[0])*err[0] + (1 + Ke[2]*Ko[0])*BT[0][2]*err[2] + Ko[0]*error_omega[0] );
   control_effort[2] = Gm[1]/inertia[1]*error_omega_int[1]
                       + ang_vel[0]*ang_vel[2]*(inertia[0] - inertia[2])
                       + inertia[1]*( (1 + Ke[1]*Ko[1])*BT[1][1]*err[1] + (1 + Ke[2]*Ko[1])*BT[1][2]*err[2] + Ko[1]*error_omega[1] );
   control_effort[3] = Gm[2]/inertia[2]*error_omega_int[2]
                       + ang_vel[0]*ang_vel[1]*(inertia[1] - inertia[0])
                       + inertia[2]*( (1 + Ke[1]*Ko[2])*BT[2][1]*err[1] + (1 + Ke[2]*Ko[2])*BT[2][2]*err[2] + Ko[2]*error_omega[2] );

   // Apply saturation limits
   for (int i = 0; i < 4; i++) {  // thrust, roll, pitch, yaw saturation limits
      if (control_effort[i] > upper_limit[i])
         control_effort[i] = upper_limit[i];
      else if (control_effort[i] < lower_limit[i])
         control_effort[i] = lower_limit[i];
   }

   // Publish the stabilizing control effort if the controller is enabled
   if (controller_enabled) {
      control_msg = controller::FloatList();
      control_msg.data = control_effort;
      control_effort_pub.publish(control_msg);
   }
   else {
     error_omega_int[0] = 0;
     error_omega_int[1] = 0;
     error_omega_int[2] = 0;
   }

   return;
}


void get_params(double in, double &value)
{
   value = in;
}


void reconfigure_callback(controller::ControllerConfig &config, uint32_t level)
{
   if (first_reconfig) {
      get_params(Ke[0], config.Kp1);
      get_params(Ke[1], config.Kp2);
      get_params(Ke[2], config.Kp3);
      get_params(Ko[0], config.Kd1);
      get_params(Ko[1], config.Kd2);
      get_params(Ko[2], config.Kd3);
      get_params(Gm[0], config.Gm1);
      get_params(Gm[1], config.Gm2);
      get_params(Gm[2], config.Gm3);
      first_reconfig = false;
      return;  // Ignore the first call to reconfigure which happens at startup
   }

   Ke[0] = config.Kp1;
   Ke[1] = config.Kp2;
   Ke[2] = config.Kp3;
   Ko[0] = config.Kd1;
   Ko[1] = config.Kd2;
   Ko[2] = config.Kd3;
   Gm[0] = config.Gm1;
   Gm[1] = config.Gm2;
   Gm[2] = config.Gm3;
   ROS_INFO("Attitude reconf");
   ROS_INFO("Ke: %2.4f, %2.4f, %2.4f", Ke[0], Ke[1], Ke[2]);
   ROS_INFO("Ko: %2.4f, %2.4f, %2.4f", Ko[0], Ko[1], Ko[2]);
   ROS_INFO("Gm: %2.4f, %2.4f, %2.4f", Gm[0], Gm[1], Gm[2]);
}


////////////////////////////////////
// Error checking
////////////////////////////////////
bool validate_parameters()
{
   for (int i = 0; i < 4; i++){
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
   std::cout << "Keta: { " << Ke[0] << ", " << Ke[1] << ", " << Ke[2] << " }" << std::endl;
   std::cout << "Kome: { " << Ko[0] << ", " << Ko[1] << ", " << Ko[2] << " }" << std::endl;
   if ( cutoff_frequency== -1) // If the cutoff frequency was not specified by the user
      std::cout<<"LPF cutoff frequency: 1/4 of sampling rate"<<std::endl;
   else
      std::cout<<"LPF cutoff frequency: "<< cutoff_frequency << std::endl;
   std::cout << "Controller node name: " << ros::this_node::getName() << std::endl;
   std::cout << "Name of topic from controller: " << topic_from_attitude << std::endl;
   std::cout << "Name of topic from the plant: " << topic_from_plant << std::endl;
   std::cout << "Name of setpoint topic: " << setpoints_topic << std::endl;
   std::cout << "Integral-windup limits: " << windup_limit[0] << ", " << windup_limit[1] 
				 << ", " << windup_limit[2] << std::endl;
   std::cout << "Thrust saturation limits: " << upper_limit[0] << "/" << lower_limit[0] << std::endl;
   std::cout << "Roll saturation limits: " << upper_limit[1] << "/" << lower_limit[1] << std::endl;
   std::cout << "Pitch saturation limits: " << upper_limit[2] << "/" << lower_limit[2] << std::endl;
   std::cout << "Yaw sturation limits: " << upper_limit[3] << "/" << lower_limit[3] << std::endl;
   std::cout << "-----------------------------------------" << std::endl;

   return;
}


////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
   ROS_INFO("Starting att. controller with node name %s", node_name.c_str());

   // Initialize ROS stuff
   ros::init(argc, argv, node_name);  // Note node_name can be overidden by launch file
   ros::NodeHandle node;
   ros::NodeHandle node_priv("~");

   while (ros::Time(0) == ros::Time::now()) {
      ROS_INFO("controller spinning waiting for time to become non-zero");
      sleep(1);
   }

   // Get params if specified in launch file or as params on command-line, set defaults
   node_priv.param<double>("Ke1", Ke[0], 1.0);
   node_priv.param<double>("Ke2", Ke[1], 0.0);
   node_priv.param<double>("Ke3", Ke[2], 0.0);
   node_priv.param<double>("Ko1", Ko[0], 1.0);
   node_priv.param<double>("Ko2", Ko[1], 0.0);
   node_priv.param<double>("Ko3", Ko[2], 0.0);
   node_priv.param<double>("Gm1", Gm[0], 0.0);
   node_priv.param<double>("Gm2", Gm[1], 0.0);
   node_priv.param<double>("Gm3", Gm[2], 0.0);
   node_priv.param<double>("cutoff_frequency", cutoff_frequency, -1.0);
   node_priv.param<double>("roll_windup_limit", windup_limit[0], 2.5);
   node_priv.param<double>("pitch_windup_limit", windup_limit[1], 2.5);
	node_priv.param<double>("yaw_windup_limit", windup_limit[2], 2.5);
	node_priv.param<double>("thrust_upper_limit_", upper_limit[0], 10.0);
   node_priv.param<double>("thrust_lower_limit", lower_limit[0], -10.0);
   node_priv.param<double>("roll_upper_limit", upper_limit[1], 2.5);
   node_priv.param<double>("roll_lower_limit", lower_limit[1], -2.5);
   node_priv.param<double>("pitch_upper_limit", upper_limit[2], 2.5);
   node_priv.param<double>("pitch_lower_limit", lower_limit[2], -2.5);
   node_priv.param<double>("yaw_upper_limit", upper_limit[3], 1.0);
   node_priv.param<double>("yaw_lower_limit", lower_limit[3], -1.0);
	node_priv.param<double>("mass", mass, 0.72);
	node_priv.param<double>("x_inertia", inertia[0], 0.1167);
	node_priv.param<double>("y_inertia", inertia[1], 0.1105);
	node_priv.param<double>("z_inertia", inertia[2], 0.2218);
   node_priv.param<std::string>("setpoints_topic", setpoints_topic, "setpoints");
   node_priv.param<std::string>("topic_from_plant", topic_from_plant, "state");
   node_priv.param<std::string>("topic_from_position", topic_from_position, "position_control");
   node_priv.param<std::string>("controller_enable_topic", controller_enable_topic, "attitude_enable");
   node_priv.param<std::string>("topic_from_attitude", topic_from_attitude, "attitude_control");
   //node_priv.param<double>("max_loop_frequency", max_loop_frequency, 1.0);
   //node_priv.param<double>("min_loop_frequency", min_loop_frequency, 1000.0);

   // Two parameters to allow for error calculation with discontinous value
   node_priv.param<bool>("angle_error", angle_error, false);
   node_priv.param<double>("angle_wrap", angle_wrap, 2.0*3.14159);

   // Update params if specified as command-line options, & print settings
   print_parameters();
   if (not validate_parameters()) {
      std::cout << "Error: invalid parameter\n";
   }

   // instantiate publishers & subscribers
   control_effort_pub = node.advertise<controller::FloatList>(topic_from_attitude, 1);

   ros::Subscriber sub = node.subscribe(topic_from_plant, 1, plant_state_callback );
   ros::Subscriber setpoints_sub = node.subscribe(setpoints_topic, 1, setpoints_callback );
   ros::Subscriber position_sub = node.subscribe(topic_from_position, 1, position_callback );
   ros::Subscriber controller_enabled_sub = node.subscribe(controller_enable_topic, 1, controller_enable_callback );

   // configure dynamic reconfiguration
   dynamic_reconfigure::Server<controller::ControllerConfig> config_server;
   dynamic_reconfigure::Server<controller::ControllerConfig>::CallbackType f;
   f = boost::bind(&reconfigure_callback, _1, _2);
   config_server.setCallback(f);

   // Wait for first messages
   while( !ros::topic::waitForMessage<controller::FloatList>(setpoints_topic, ros::Duration(10.)) )
      ROS_WARN_STREAM("Waiting for the setpoint to be published.");
   //while( !ros::topic::waitForMessage<geometry_msgs::TransformStamped>(topic_from_plant, ros::Duration(10.)) )
   while( !ros::topic::waitForMessage<controller::FloatList>(topic_from_plant, ros::Duration(10.)) )
      ROS_WARN_STREAM("Waiting for a msg on the state of the plant.");
   while( !ros::topic::waitForMessage<controller::FloatList>(topic_from_position, ros::Duration(10.)) )
      ROS_WARN_STREAM("Waiting for the position control msg.");

   // Respond to inputs until shut down
   ros::spin();

   return 0;
}


