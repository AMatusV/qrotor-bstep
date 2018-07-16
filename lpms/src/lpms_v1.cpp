
#include "lpms/lpms_v1.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>


double vicon_yaw;


void vicon_callback(const geometry_msgs::TransformStamped& vicon_msg)
{
   double dummy;

   tf::Quaternion quater;
   tf::quaternionMsgToTF(vicon_msg.transform.rotation, quater);
   tf::Matrix3x3(quater).getRPY(dummy, dummy, vicon_yaw);
   //ROS_INFO("V Y: %f", vicon_offset);
}


lpms_sensor::lpms_sensor(ros::NodeHandle nh):
    nh_(nh)
{
   ROS_INFO("Starting lpms sensor...");

   ros::NodeHandle private_nh("~");

   private_nh.param<std::string>("mac_addr_device", mac_addr_device_, "00:04:3E:9F:E1:47");

   ROS_INFO("Address device: %s",mac_addr_device_.c_str());

   angles_pub = nh_.advertise<controller::FloatList>("angles", 1);
   vicon_sub = nh_.subscribe("vicon/odro/odro", 1, vicon_callback);

   //Sensor
   manager = LpmsSensorManagerFactory();
   lpms = manager->addSensor(DEVICE_LPMS_B2, mac_addr_device_.c_str());
}


lpms_sensor::~lpms_sensor()
{
   // Removes the initialized sensor
   manager->removeSensor(lpms);

   // Deletes LpmsSensorManager object
   delete manager;
}


void lpms_sensor::run(void)
{
   /*int count;
   double internal_offset;*/
   std::vector<double> angles(3, 0);

   while (ros::ok()) {

      // Checks, if sensor is connected
      if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms->hasImuData()) {

         // Reads quaternion data
         d = lpms->getCurrentData();

         /*if (count == 0) {
            count++;
         }
         else if (count == 1) {
	    internal_offset = d.r[2];
	    count = 2;
         }*/

         angles[0] = d.r[0]*deg2rad;
         angles[1] = d.r[1]*deg2rad;
         //angles[2] = (d.r[2] - internal_offset)*deg2rad;
         angles[2] = vicon_yaw;

         angles_msg = controller::FloatList();
         angles_msg.data = angles;
         angles_pub.publish(angles_msg);

      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      //Do this.
      ros::spinOnce();
      //Added a delay so not to spam
      //sleep(2);
   }
}
