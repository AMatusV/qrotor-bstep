
#include "lpms/lpms.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "lpms_node");
   ros::NodeHandle nh;

   lpms_sensor lpms_sensor_(nh);

   lpms_sensor_.run();

   return 0;
}
