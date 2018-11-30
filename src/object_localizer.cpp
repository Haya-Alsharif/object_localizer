#include "object_localizer/object_localizer.h"

/** 
 * Constructor 
 * **/
ObjectLocalizer::ObjectLocalizer(ros::NodeHandle* nodehandle, int& q, std::string& darknet_bounding_boxes, std::string& camera_pointcloud):
  nh_(*nodehandle),
  subBoundingBoxes_(nh_, darknet_bounding_boxes.c_str(), q),
  subDepth_(nh_, camera_pointcloud.c_str(), q),
  subPose_(nh_, "/mavros/local_position/pose", q),
  sync_(MySyncPolicy(q),  subBoundingBoxes_, subDepth_, subPose_)
{
  //sync_.setAgePenalty(1.0);
  sync_.registerCallback(boost::bind(&ObjectLocalizer::callback, this, _1, _2, _3));
  nh_.getParam("detection_class", detection_class);
  pubGoal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
}


/** 
 * Deconstructor 
 * **/
ObjectLocalizer::~ObjectLocalizer() { }


/** 
 * Convert detected bounding box message goal from detection frame frame to camera_link frame.
 *  **/
tf::StampedTransform ObjectLocalizer::get_goal_camera_transform(const darknet_ros_msgs::BoundingBox& goal_bounding_box){
  // compute object width and height
  double objectWidth = goal_bounding_box.xmax - goal_bounding_box.xmin;
  double objectHeight = goal_bounding_box.ymax - goal_bounding_box.ymin;

  // compute object center
  double xPosCenter = goal_bounding_box.xmin + objectWidth*0.5;
  double yPosCenter = goal_bounding_box.ymin + objectHeight*0.5;
  pcl::PointXYZ objectCenter = currentDepth_.at(xPosCenter, yPosCenter);

  // compute the rotation axis x y z using the 2 points on the object
  double xHorizontalAxis = xPosCenter + objectWidth*0.5;
  double yHorizontalAxis = yPosCenter;
  pcl::PointXYZ horizontalAxis = currentDepth_.at(xHorizontalAxis, yHorizontalAxis);
  double xVerticalAxis = xPosCenter;
  double yVerticalAxis = yPosCenter + objectHeight*0.5;
  pcl::PointXYZ verticalAxis = currentDepth_.at(xVerticalAxis, yVerticalAxis);

  // define a tf for the object
  tf::StampedTransform transform;
  transform.setIdentity();
  transform.child_frame_id_ = "person";
  transform.frame_id_ = currentDepth_.header.frame_id; //camera_link frame
  transform.stamp_ = currentBoundingBoxes_.header.stamp;

  // set tf center
  transform.setOrigin(tf::Vector3(objectCenter.x,objectCenter.y,objectCenter.z));

  // set tf rotation
  tf::Vector3 xAxis(horizontalAxis.x - objectCenter.x, horizontalAxis.y - objectCenter.y, horizontalAxis.z - objectCenter.z);
  tf::Vector3 yAxis(verticalAxis.x - objectCenter.x, verticalAxis.y - objectCenter.y, verticalAxis.z - objectCenter.z);
  xAxis.normalize();
  yAxis.normalize();
  tf::Vector3 zAxis = xAxis.cross(yAxis);
  zAxis.normalize();
  tf::Matrix3x3 rotationMatrix(
          xAxis.x(), yAxis.x() ,zAxis.x(),
          xAxis.y(), yAxis.y(), zAxis.y(),
          xAxis.z(), yAxis.z(), zAxis.z());
  tf::Quaternion q;
  rotationMatrix.getRotation(q);
  q = q * tf::createQuaternionFromRPY(CV_PI/2.0, CV_PI/2.0, 0);
  q = q.normalized();
  transform.setRotation(q);

  // broadcast transformation for visualization
  static tf::TransformBroadcaster tfBroadcaster_;
  tfBroadcaster_.sendTransform(transform);
  return transform;
}


/** 
 * transform goal center to /camera_link frame using the transformation computed above 
 * **/
geometry_msgs::PoseStamped ObjectLocalizer::goal_camera_frame(const darknet_ros_msgs::BoundingBox& goal_bounding_box){ 
  tf::StampedTransform transform = get_goal_camera_transform(goal_bounding_box);
  geometry_msgs::PoseStamped goal_camera;
  goal_camera.header.frame_id = transform.frame_id_;
  goal_camera.header.stamp = transform.stamp_;
  goal_camera.pose.position.x = transform.getOrigin().getX();
  goal_camera.pose.position.y = transform.getOrigin().getX();
  goal_camera.pose.position.z = transform.getOrigin().getY();
  goal_camera.pose.orientation.w = transform.getRotation().getW();
  goal_camera.pose.orientation.x = transform.getRotation().getX();
  goal_camera.pose.orientation.y = transform.getRotation().getY();
  goal_camera.pose.orientation.z = transform.getRotation().getZ();

  //ROS_INFO("goal in /camera_link frame: [%0.2f, %0.2f, %0.2f]",goal_camera.pose.position.x,goal_camera.pose.position.y,goal_camera.pose.position.z);

  return goal_camera;
}


/** 
 * transform goal center from /camera_link to /local_origin 
 * **/
geometry_msgs::PoseStamped ObjectLocalizer::goal_transform_frame(const std::string& target_frame, const geometry_msgs::PoseStamped& goal_camera){

  geometry_msgs::PoseStamped goal_target_frame;
  try{ 
    tfListener_.transformPose(target_frame.c_str(), goal_camera, goal_target_frame);
  }
  catch (tf::TransformException ex){
    ROS_ERROR_STREAM("Error: " << ex.what());
   }
  return goal_target_frame;
}


/** 
 * check if goal is close to current position using manhatin distance 
 * **/
bool ObjectLocalizer::goal_within_range(const float& tolerance, const geometry_msgs::PoseStamped& goal_local){
  return (abs(goal_local.pose.position.x - currentPose_.pose.position.x) + abs(goal_local.pose.position.y - currentPose_.pose.position.y) < tolerance);
}


/** 
 * select bounding boxes of certain class 
 * **/
darknet_ros_msgs::BoundingBoxes ObjectLocalizer::select_detection_class(const darknet_ros_msgs::BoundingBoxes& current_bounding_boxes, std::string& class_name )
{
  darknet_ros_msgs::BoundingBoxes selected_bounding_boxes;
  for(unsigned int i = 0; i < currentBoundingBoxes_.bounding_boxes.size(); ++i) {
    if(currentBoundingBoxes_.bounding_boxes.at(i).Class == class_name.c_str()) {
      selected_bounding_boxes.bounding_boxes.push_back(currentBoundingBoxes_.bounding_boxes.at(i));
    }
  }
  return selected_bounding_boxes;
}


/** 
 * select bounding box with largest probability 
 * **/
darknet_ros_msgs::BoundingBox ObjectLocalizer::select_largest_probability(const darknet_ros_msgs::BoundingBoxes& current_bounding_boxes)
{
  unsigned int target_i = 0;
  float largest_probability = 0;
  float current_probability = 0;
  for(unsigned int i = 0; i < current_bounding_boxes.bounding_boxes.size(); ++i) {
    current_probability = current_bounding_boxes.bounding_boxes.at(i).probability;
    if( current_probability > largest_probability) {
      target_i = i;
      largest_probability = current_probability;
    }
  }
  return current_bounding_boxes.bounding_boxes.at(target_i);
}


/** 
 * set private parameter when new messages are recieved
 * **/
void ObjectLocalizer::setParam(const darknet_ros_msgs::BoundingBoxes& current_bounding_box, const sensor_msgs::PointCloud2& current_depth, const geometry_msgs::PoseStamped & curent_pose){
  // set private parameters
  currentBoundingBoxes_ = current_bounding_box;
  pcl::fromROSMsg(current_depth, currentDepth_);
  pcl_conversions::toPCL(current_depth.header, currentDepth_.header);
  currentPose_ = curent_pose;

  // make sure transformation exist bewerrn camera frame and fcu or local_origin
  tfListener_.waitForTransform("/local_origin",currentDepth_.header.frame_id, currentBoundingBoxes_.header.stamp, ros::Duration(3.0));
}


/** 
 * main syncronized callback when new messages are recieved
 * **/
void ObjectLocalizer::callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &current_bounding_box, const sensor_msgs::PointCloud2ConstPtr& current_depth, const geometry_msgs::PoseStamped::ConstPtr & curent_pose){ 

  // set private paramteres
  setParam(*current_bounding_box, *current_depth, *curent_pose);

  // select certain calss, e.g. "person"
  if (!detection_class.empty()){
    currentBoundingBoxes_ = select_detection_class(currentBoundingBoxes_, detection_class);
  }
  if (currentBoundingBoxes_.bounding_boxes.size()==0){
    ROS_INFO("No detection of class type %s", detection_class.c_str());
  }
  else{
    // select instance with largest probability
    darknet_ros_msgs::BoundingBox goal_bounding_box = select_largest_probability(currentBoundingBoxes_);
    
    // convert detected goal to desired frame. You can choose "/world" frame depending on expected goal position frame used by /move_base_simple
    std::string target_frame = "/local_origin";
    geometry_msgs::PoseStamped goal_camera = goal_camera_frame(goal_bounding_box);
    geometry_msgs::PoseStamped goal_local = goal_transform_frame(target_frame, goal_camera);
    ROS_INFO( "OBJECT WAS DETECTED AND LOCALIZED===================\nDetection Class = %s.\nDetection Confidence = of %0.2f\nLocation in %s frame = [%0.2f meter, %0.2f meter, %0.2f meter]\n============================================================\n", goal_bounding_box.Class.c_str(), goal_bounding_box.probability, 
      target_frame.c_str(),
      goal_local.pose.position.x,goal_local.pose.position.y,goal_local.pose.position.z 
    );

    // if the goal is detected and reached, set mession to accomplished and exit 
    float tolerance = 1.0;
    if(!goal_within_range(tolerance, goal_local)){
      
      
      pubGoal_.publish(goal_local);
    }
    else{
      mission_accomplished = true;
    }
  }
}