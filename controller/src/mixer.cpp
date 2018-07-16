#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include "controller/MixerConfig.h"
#include "controller/FloatList.h"


////////////////////////////////////
// Private Vars
////////////////////////////////////

// Primary Mixer input & ouput variables
double control_effort[4];  // global thrust, roll, pitch, yaw torque
bool mixer_enabled = true; // Mixer node is not enable to run
uint16_t motor_command[4];

// Mixer reconfigurable parameters
double roll_c, pitch_c, yaw_c;
double c_a, c_b, c_c;
bool first_reconfig = true;

// Node frequency
double node_frequency = 200;

// Used in the publishing logic
bool cmd_flag, stop_flag;

// Topic and node names and message objects
std::string topic_from_attitude, mixer_enable_topic, node_name = "mixer_node";
//dynamic_tutorials::UIntList commands_msg;

// PCA9685 related constants
double pwm_frequency;
uint16_t pwm_minval = 1228;
uint16_t pwm_maxval = 2400;
int _controller_io_device = 4; // linux file for I2C
int _controller_io_handle; // linux file handle for I2C
#define _BASE_ADDR   0x40
#define MAX_SERVOS   4
enum pwm_regs {
  // Registers/etc.
  __MODE1              = 0x00,
  __MODE2              = 0x01,
  __PRESCALE           = 0xFE,
  __CHANNEL_ON_L       = 0x06,
  __CHANNEL_ON_H       = 0x07,
  __CHANNEL_OFF_L      = 0x08,
  __CHANNEL_OFF_H      = 0x09,
  __ALL_CHANNELS_ON_L  = 0xFA,
  __ALL_CHANNELS_ON_H  = 0xFB,
  __ALL_CHANNELS_OFF_L = 0xFC,
  __ALL_CHANNELS_OFF_H = 0xFD,
  __RESTART            = 0x80,
  __SLEEP              = 0x10,   // enable low power mode
  __ALLCALL            = 0x01,
  __OUTDRV             = 0x04
};



/*void force_callback(const std_msgs::Float64& force_msg)
{
   force = force_msg.data;
   //ROS_INFO("force: %f", force);
}*/


void attitude_callback(const controller::FloatList& attitude_msg)
{
   control_effort[0] = attitude_msg.data[0];
   control_effort[1] = attitude_msg.data[1];
	control_effort[2] = attitude_msg.data[2];
	control_effort[3] = attitude_msg.data[3];
   //ROS_INFO("torque: %f, %f, %f", torque[0], torque[1], torque[2]);
}


void mixer_enable_callback(const std_msgs::Bool& enable_msg)
{
   mixer_enabled = enable_msg.data;
   stop_flag = !mixer_enabled;
}


void get_params(double in, double &value)
{
   value = in;
}


void reconfigure_callback(controller::MixerConfig &config, uint32_t level)
{
   if (first_reconfig) {
      get_params(roll_c, config.roll_c);
      get_params(pitch_c, config.pitch_c);
      get_params(yaw_c, config.yaw_c);
		get_params(c_a, config.c_a);
      get_params(c_b, config.c_b);
      get_params(c_c, config.c_c);
      first_reconfig = false;
     return;  // Ignore the first reconfigure call which happens at startup
   }

   roll_c = config.roll_c;
   pitch_c = config.pitch_c;
   yaw_c = config.yaw_c;
   c_a = config.c_a;
   c_b = config.c_b;
   c_c = config.c_c;
   ROS_INFO("Mixer reconf request RPY_c: %2.4f, %2.4f, %2.4f", roll_c, pitch_c, yaw_c);
   ROS_INFO("Mixer reconf request abc_c: %f, %f, %f", c_a, c_b, c_c);
}


/*static void _set_pwm_interval (int servo, int start, int end)
{
      ROS_DEBUG("_set_pwm_interval enter");

   if ((servo<1) || (servo>MAX_SERVOS)) {
      ROS_ERROR("Invalid servo number %d; servo numbers must be between 1 and %d", servo, MAX_SERVOS);
      return;
   }

   // the public API is ONE based and hardware is ZERO based
   int channel = servo - 1;		// the hardware enumerates servos as 0..15
      ROS_DEBUG("_set_pwm_interval board=%d servo=%d", _BASE_ADDR, servo);

   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __CHANNEL_ON_L+4*channel, start & 0xFF))
      ROS_ERROR ("Error setting PWM start low byte on servo %d on board %d", servo, _BASE_ADDR);
   if (0 >  i2c_smbus_write_byte_data (_controller_io_handle, __CHANNEL_ON_H+4*channel, start  >> 8))
      ROS_ERROR ("Error setting PWM start high byte on servo %d on board %d", servo, _BASE_ADDR);
   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __CHANNEL_OFF_L+4*channel, end & 0xFF))
      ROS_ERROR ("Error setting PWM end low byte on servo %d on board %d", servo, _BASE_ADDR);
   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __CHANNEL_OFF_H+4*channel, end >> 8))
      ROS_ERROR ("Error setting PWM end high byte on servo %d on board %d", servo, _BASE_ADDR);
}


static void _set_pwm_interval_all (int start, int end)
{
   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __ALL_CHANNELS_ON_L, start & 0xFF))
      ROS_ERROR ("Error setting PWM start low byte for all servos on board %d", _BASE_ADDR);
   if (0 >  i2c_smbus_write_byte_data (_controller_io_handle, __ALL_CHANNELS_ON_H, start  >> 8))
      ROS_ERROR ("Error setting PWM start high byte for all servos on board %d", _BASE_ADDR);
   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __ALL_CHANNELS_OFF_L, end & 0xFF))
      ROS_ERROR ("Error setting PWM end low byte for all servos on board %d", _BASE_ADDR);
   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __ALL_CHANNELS_OFF_H, end >> 8))
      ROS_ERROR ("Error setting PWM end high byte for all servos on board %d", _BASE_ADDR);
}


static void _set_pwm_frequency (int freq)
{
    int prescale;
    char oldmode, newmode;
    int res;

    pwm_frequency = freq;   // save to global

       ROS_DEBUG("_set_pwm_frequency prescale");
    float prescaleval = 25000000.0; // 25MHz
    prescaleval /= 4096.0;
    prescaleval /= (float)freq;
    prescaleval -= 1.0;
    //ROS_INFO("Estimated pre-scale: %6.4f", prescaleval);
    prescale = floor(prescaleval + 0.5);
    // ROS_INFO("Final pre-scale: %d", prescale);

    ROS_INFO("Setting PWM frequency to %d Hz", freq);

    nanosleep ((const struct timespec[]){{1, 000000L}}, NULL); 

    oldmode = i2c_smbus_read_byte_data (_controller_io_handle, __MODE1);
    newmode = (oldmode & 0x7F) | 0x10; // sleep

    if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __MODE1, newmode)) // go to sleep
        ROS_ERROR("Unable to set PWM controller to sleep mode"); 

    if (0 >  i2c_smbus_write_byte_data(_controller_io_handle, __PRESCALE, (int)(floor(prescale))))
        ROS_ERROR("Unable to set PWM controller prescale"); 

    if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, oldmode))
        ROS_ERROR("Unable to set PWM controller to active mode"); 

    nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec,

    if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, oldmode | 0x80))
        ROS_ERROR("Unable to restore PWM controller to active mode");
}


static void _pwm_init ()
{
   char filename[20];
   snprintf(filename, 19, "/dev/i2c-%d", _controller_io_device);
   //device << "/dev/i2c-" << _controller_io_device;

   if ((_controller_io_handle = open (filename, O_RDWR)) < 0) {
      ROS_FATAL ("Failed to open I2C bus %s", filename);
      return; // exit(1)    // additional ERROR HANDLING
   }
   ROS_INFO ("I2C bus opened on %s", filename);

   if ( 0 > ioctl (_controller_io_handle, I2C_SLAVE, _BASE_ADDR) ) {
      ROS_FATAL ("Failed to acquire bus access and/or talk to I2C slave at address 0x%02X", _BASE_ADDR);
      return; // exit(1)    // additional ERROR HANDLING information is available with 'errno'
   }

   // this is guess but I believe the following needs to be done on each board only once
   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __MODE2, __OUTDRV))
      ROS_ERROR ("Failed to enable PWM outputs for totem-pole structure");

   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __MODE1, __ALLCALL))
      ROS_ERROR ("Failed to enable ALLCALL for PWM channels");

   nanosleep ((const struct timespec[]){{0, 5000000L}}, NULL);   // sleep 5microsec, wait for osci

   char mode1res;
   mode1res = i2c_smbus_read_byte_data (_controller_io_handle, __MODE1);
   mode1res = mode1res & ~__SLEEP; //                 # wake up (reset sleep)

   if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __MODE1, mode1res))
      ROS_ERROR ("Failed to recover from low power mode");

   nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

   // the first time we activate a board, we mark it and set all of its servo channels to 0
   _set_pwm_interval_all (0, 0);

   _set_pwm_frequency((int)pwm_frequency);
}*/


int main(int argc, char **argv)
{
   // Initialize ROS stuff
   ros::init(argc, argv, node_name);
   ros::NodeHandle node;
   ros::NodeHandle node_priv("~");

   // Get params if specified in launch file or pass on command-line, set defaults
   node_priv.param<double>("roll_c", roll_c, 6.2163);
   node_priv.param<double>("pitch_c", pitch_c, 6.2163);
   node_priv.param<double>("yaw_c", yaw_c, 5.0);
   node_priv.param<double>("c_a", c_a, 0.0000083438);
   node_priv.param<double>("c_b", c_b, -0.015693);
   node_priv.param<double>("c_c", c_c, 7.2688);
   node_priv.param<double>("pwm_frequency", pwm_frequency, 300.0);
   node_priv.param<double>("node_frequency", node_frequency, 200.0);
   //node_priv.param<std::string>("force_topic", force_topic, "force");
   node_priv.param<std::string>("topic_from_attitude", topic_from_attitude, "attitude_control");
   node_priv.param<std::string>("mixer_enable_topic", mixer_enable_topic, "mixer_enable");

   // Instantiate subscribers
   //ros::Subscriber force_sub = node.subscribe(force_topic, 1, force_callback);
   ros::Subscriber torque_sub = node.subscribe(topic_from_attitude, 1, attitude_callback);
   ros::Subscriber enable_sub = node.subscribe(mixer_enable_topic, 1, mixer_enable_callback);

   // Set dynamic reconfiguration
   dynamic_reconfigure::Server<controller::MixerConfig> config_server;
   dynamic_reconfigure::Server<controller::MixerConfig>::CallbackType f;
   f = boost::bind(&reconfigure_callback, _1, _2);
   config_server.setCallback(f);

   // Initialize PWM board
   //_pwm_init();

   // Wait for first messages
   /*while( !ros::topic::waitForMessage<std_msgs::Float64>(force_topic, ros::Duration(10.)) )
      ROS_WARN_STREAM("Waiting for the force msg to be published.");*/
   while( !ros::topic::waitForMessage<controller::FloatList>(topic_from_attitude, ros::Duration(10.)) )
      ROS_WARN_STREAM("Waiting for the control msg to be published.");

   ros::Rate loop_rate(node_frequency);

   while (ros::ok()) {
      if (mixer_enabled) {
         //Roll is control by M2 and M4 via U2
         //Pitch is control by M1 and M3 via U3
         double thrust[4];
         thrust[0] = (control_effort[0] - roll_c*control_effort[1] - pitch_c*control_effort[2] - yaw_c*control_effort[3])/4;
         thrust[1] = (control_effort[0] + roll_c*control_effort[1] - pitch_c*control_effort[2] + yaw_c*control_effort[3])/4;
         thrust[2] = (control_effort[0] + roll_c*control_effort[1] + pitch_c*control_effort[2] - yaw_c*control_effort[3])/4;
         thrust[3] = (control_effort[0] - roll_c*control_effort[1] + pitch_c*control_effort[2] + yaw_c*control_effort[3])/4;

         for (int i = 0; i < 4; i++) {
            if (thrust[i] < 0) {  // Thrust lower bound
               thrust[i] = 0;
            }

            // Quadratic PWM-thrust model, i.e., T = c_a*Width^2 + c_b*Width + c_c
            thrust[i] = ( -c_b + sqrt(c_b*c_b - 4*c_a*(c_c - thrust[i])) )/(2*c_a)*0.004095*pwm_frequency;

            // Simple way to convert from float to integer
            motor_command[i] = (uint16_t)(thrust[i] + 0.5);
            if (motor_command[i] > pwm_maxval) {  // PWM upper bound
               //cmd_flag = false;  // Turn off motor for safety
	       		motor_command[i] = pwm_maxval;
            }
         }

         if (cmd_flag) {
            /*for (int i = 0; i < 4; i++)
               _set_pwm_interval(i+1, 0, motor_command[i]);*/
            ROS_INFO("MC: %d, %d, %d, %d", motor_command[0], motor_command[1], motor_command[2], motor_command[3]);
         }
         else {
            mixer_enabled = false;
            stop_flag = true;
            cmd_flag = true;
         }
      }
      else {
         if (stop_flag) {
            /*for (int i = 0; i < 4; i++)
               motor_command[i] = pwm_minval;*/

            //_set_pwm_interval_all(0, pwm_minval);
            ROS_INFO("MC: %d, %d, %d, %d", motor_command[0], motor_command[1], motor_command[2], motor_command[3]);

            stop_flag = false;
         }
      }

      ros::spinOnce();
      loop_rate.sleep();
   }

   // Disarm motors
   //_set_pwm_interval_all(0, 0);

   return 0;
}
