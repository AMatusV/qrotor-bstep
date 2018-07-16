

#include "lpms/lpms.h"

lpms_sensor::lpms_sensor(ros::NodeHandle nh):
    nh_(nh)
{
    ROS_INFO("Starting lpms sensor...");


    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("mac_addr_device", mac_addr_device_, "00:04:3E:9F:E1:47");

    ROS_INFO("Addess device: %s",mac_addr_device_.c_str());


	pub_acc = nh_.advertise<std_msgs::Float64MultiArray>("array", 100);

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

	while (ros::ok())
	{
		std_msgs::Float64MultiArray array;

        array.layout.dim.push_back(std_msgs::MultiArrayDimension());
        array.layout.dim[0].size = 3;
        array.layout.dim[0].stride = 1;
        array.layout.dim[0].label = "vel";

		//Clear array
		array.data.clear();

        // Checks, if sensor is connected
        if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
            lpms->hasImuData())
        {
            // Reads quaternion data
            d = lpms->getCurrentData();

            // Shows data
            printf("Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f, x=%f, y=%f, z=%f\n",
                d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3],
                d.r[0], d.r[1], d.r[2]);

            for (int i = 0; i < 3; i++)
            {
                //assign array a random number between 0 and 255.
                array.data.push_back(d.linAcc[i]);
            }
            //Publish array
            pub_acc.publish(array);

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));



		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		//sleep(2);
	}
}
