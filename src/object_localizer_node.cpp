#include "object_localizer/object_localizer.h"

/**
 * main function
 * **/
int main (int argc, char** argv)
{
  ros::init(argc, argv, "object_localizer_node");
  ros::NodeHandle nh("~");
  ObjectLocalizer localizer(&nh);
  ros::spin();
  return 0;
}
