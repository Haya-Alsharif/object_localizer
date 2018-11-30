#include "object_localizer/object_localizer.h"

/** 
 * sample landing action to take if a desired object is detected 
 * **/
bool land_now(ros::ServiceClient* land_client){
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    return (!(land_client->call(land_cmd) && land_cmd.response.success));
}

/**
 * main function
 * **/
int main (int argc, char** argv)
{
  ros::init(argc, argv, "object_localizer_node");
  ros::NodeHandle nh; 
  ros::Rate r(15); // 15 hz

  // set topic params
  int q;
  std::string darknet_bounding_boxes, camera_pointcloud, detection_class;
  nh.param<std::string>("darknet_bounding_boxes_topic", darknet_bounding_boxes, "/darknet_ros/bounding_boxes");
  nh.param<std::string>("camera_pointcloud_topic", camera_pointcloud, "/camera_down/depth/points");
  nh.param<std::string>("detection_class", detection_class, "");
  nh.param<int>("queue_size", q, 10);
  //nh.getParam("camera_pointcloud_topic", camera_pointcloud);
  //nh.getParam("detection_class", detection_class);
  
  ObjectLocalizer localizer(&nh, q, darknet_bounding_boxes, camera_pointcloud);
  localizer.mission_accomplished = false;

  while (ros::ok() && !localizer.mission_accomplished){
    ros::spinOnce();
    r.sleep();
  }

  // sample action if goal is localized
  ros::ServiceClient land_client;
  land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ROS_INFO("Goal reached, landing now ...");
  while (land_now(&land_client)){
    r.sleep();
  }
  ros::shutdown();

  return 0;
}