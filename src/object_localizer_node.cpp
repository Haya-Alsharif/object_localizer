#include "object_localizer/object_localizer.h"

/**
 * main function
 * **/
int main (int argc, char** argv)
{
  ros::init(argc, argv, "object_localizer_node");
  ros::NodeHandle nh; 

  // set topic params
  int queue;
  std::string darknet_bounding_boxes, pose, camera_info, detection_class, published_pose;
  nh.param<std::string>("darknet_bounding_boxes_topic", darknet_bounding_boxes, "/darknet_ros/bounding_boxes");
  nh.param<std::string>("pose_topic", pose, "/mavros/local_position/pose");
  nh.param<std::string>("camera_info_topic", camera_info, "/zed/left/camera_info");
  nh.param<std::string>("detection_class", detection_class, "");
  nh.param<int>("queue_size", queue, 10);
  nh.param<std::string>("published_pose_topic", published_pose, "/detected_object_3d_pose");

  //nh.getParam("published_pose_topic", published_pose);

  ObjectLocalizer localizer(&nh, queue, darknet_bounding_boxes, pose, camera_info, detection_class, published_pose);
  ros::spin();
  return 0;
}