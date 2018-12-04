#include "object_localizer/object_localizer.h"


/** 
 * Constructor 
 * **/
ObjectLocalizer::ObjectLocalizer(ros::NodeHandle* nodehandle):
  nh_(*nodehandle)
{
  std::string darknet_bounding_boxes, published_pose;
  int queue;
  nh_.getParam("darknet_bounding_boxes_topic", darknet_bounding_boxes);
  nh_.getParam("queue_size", queue);
  nh_.getParam("detection_class", detectionClass_);
  nh_.getParam("published_pose_topic", published_pose);
  nh_.getParam("camera_pointcloud_topic", useDepthCamera_);
  subBoundingBoxes_.subscribe(nh_, darknet_bounding_boxes.c_str(), queue);

  if (!useDepthCamera_.empty()){
    ROS_INFO("going welll ..... ");
    subDepth_.subscribe(nh_, useDepthCamera_.c_str(), queue);
    depthCameraSyncPointer_.reset(new DepthCameraSync(DepthCameraSyncPolicy(queue), subBoundingBoxes_, subDepth_));
    depthCameraSyncPointer_->registerCallback(boost::bind(&ObjectLocalizer::callback, this, _1, _2));
  }
  else{
    std::string pose, camera_info;
    nh_.getParam("camera_info_topic", camera_info);
    nh_.getParam("pose_topic", pose);
    subCameraInfo_.subscribe(nh_, camera_info.c_str(), queue);
    subPose_.subscribe(nh_, pose.c_str(), queue);

    rangeFinderSyncPointer_.reset(new RangeFinderSync(RangeFinderSyncPolicy(queue), subBoundingBoxes_, subPose_, subCameraInfo_)); //rangeFinderSyncPointer_.setAgePenalty(1.0);
    rangeFinderSyncPointer_->registerCallback(boost::bind(&ObjectLocalizer::callback, this, _1, _2, _3));
  }

  pubGoal_ = nh_.advertise<geometry_msgs::PoseStamped>(published_pose, queue);
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

  // compute object center + rotation axis x y z using the 2 points on the object
  double xPosCenter = goal_bounding_box.xmin + objectWidth*0.5;
  double yPosCenter = goal_bounding_box.ymin + objectHeight*0.5;
  double xHorizontalAxis = xPosCenter + objectWidth*0.5;
  double yHorizontalAxis = yPosCenter;
  double xVerticalAxis = xPosCenter;
  double yVerticalAxis = yPosCenter + objectHeight*0.5;
  pcl::PointXYZ objectCenter;
  pcl::PointXYZ horizontalAxis;
  pcl::PointXYZ verticalAxis;
  
  tf::StampedTransform transform; // define a tf for the object
  transform.setIdentity();
  transform.child_frame_id_ = "person";

  if (!useDepthCamera_.empty()){
    objectCenter = currentDepth_.at(xPosCenter, yPosCenter);
    horizontalAxis = currentDepth_.at(xHorizontalAxis, yHorizontalAxis);
    pcl::PointXYZ verticalAxis = currentDepth_.at(xVerticalAxis, yVerticalAxis);

    transform.frame_id_ = currentBoundingBoxes_.image_header.frame_id; //camera_link frame
    transform.stamp_ = currentBoundingBoxes_.image_header.stamp;
  }
  else{
    //set camera parameters
    float fx_ = cameraInfo_.K[0]; //focal length along the x-axis
    float fy_ = cameraInfo_.K[4]; //focal length along the y-axis
    float cx_ = cameraInfo_.K[2]; //principal point - x coordinate
    float cy_ = cameraInfo_.K[5]; //principal point - y coordinate

    objectCenter.x = ((xPosCenter-cx_)*currentPose_.pose.position.z)/fx_;
    objectCenter.y = ((yPosCenter-cy_)*currentPose_.pose.position.z)/fy_;
    objectCenter.z = currentPose_.pose.position.z;
    horizontalAxis.x = ((xHorizontalAxis-cx_)*currentPose_.pose.position.z)/fx_;
    horizontalAxis.y = ((yHorizontalAxis-cy_)*currentPose_.pose.position.z)/fy_;
    horizontalAxis.z = currentPose_.pose.position.z;
    verticalAxis.x = ((xVerticalAxis-cx_)*currentPose_.pose.position.z)/fx_;
    verticalAxis.y = ((yVerticalAxis-cy_)*currentPose_.pose.position.z)/fy_;
    verticalAxis.z = currentPose_.pose.position.z;

    transform.frame_id_ = currentBoundingBoxes_.image_header.frame_id; //camera_link frame
    transform.stamp_ = currentBoundingBoxes_.image_header.stamp;
  }

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
  catch (tf::TransformException ex){ }
  return goal_target_frame;
}


/** 
 * select bounding boxes of certain class 
 * **/
darknet_ros_msgs::BoundingBoxes ObjectLocalizer::select_detection_class(const darknet_ros_msgs::BoundingBoxes& current_bounding_boxes, std::string& class_name )
{
  darknet_ros_msgs::BoundingBoxes selected_bounding_boxes;
  for(unsigned int i = 0; i < current_bounding_boxes.bounding_boxes.size(); ++i) { 
    if(current_bounding_boxes.bounding_boxes.at(i).Class == class_name) {
      selected_bounding_boxes.bounding_boxes.push_back(current_bounding_boxes.bounding_boxes.at(i));
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
 * main syncronized callback when new messages are recieved
 * **/
void ObjectLocalizer::check_and_publish(){ 
  if(currentBoundingBoxes_.bounding_boxes.size()==0){
    if (!detectionClass_.empty()){ ROS_INFO("Object localizer: no detection of class type %s", detectionClass_.c_str()); }
    else { ROS_INFO("Object localizer: no detection"); } 
  }
  else {
    // select instance with largest probability
    darknet_ros_msgs::BoundingBox goal_bounding_box = select_largest_probability(currentBoundingBoxes_);
    
    // convert detected goal to desired frame. You can choose "/world" frame depending on expected goal position frame used by /move_base_simple
    std::string target_frame = "/local_origin";
    geometry_msgs::PoseStamped goal_camera = goal_camera_frame(goal_bounding_box);
    geometry_msgs::PoseStamped goal_local = goal_transform_frame(target_frame, goal_camera);

    ROS_INFO("Object localizer: %s (%0.2f%%) detected at [%0.2f m, %0.2f m, %0.2f m] in %s", goal_bounding_box.Class.c_str(), goal_bounding_box.probability*100, 
        goal_local.pose.position.x,goal_local.pose.position.y,goal_local.pose.position.z,target_frame.c_str() 
    );   
    pubGoal_.publish(goal_local);  
  }
}


/** 
 * main syncronized callback when new messages with pose info instead of depth
 * **/
void ObjectLocalizer::callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& current_bounding_box, const geometry_msgs::PoseStamped::ConstPtr& curent_pose, const sensor_msgs::CameraInfoConstPtr& cam_info){ 

  //select a calss if specifid
  if (!detectionClass_.empty()){
    currentBoundingBoxes_ = select_detection_class(*current_bounding_box, detectionClass_);
  }
  else{ currentBoundingBoxes_ = *current_bounding_box; }
  currentPose_ = *curent_pose;
  cameraInfo_ = *cam_info;
  tfListener_.waitForTransform("/local_origin",currentBoundingBoxes_.image_header.frame_id, currentBoundingBoxes_.image_header.stamp, ros::Duration(3.0));
  check_and_publish();
}


/** 
 * main syncronized callback when new messages with depth info are recieved
 * **/
void ObjectLocalizer::callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& current_bounding_box, const sensor_msgs::PointCloud2ConstPtr& current_depth){ 

  //select a calss if specifid
  if (!detectionClass_.empty()){
    currentBoundingBoxes_ = select_detection_class(*current_bounding_box, detectionClass_);
  }
  else{ currentBoundingBoxes_ = *current_bounding_box; }
  pcl::fromROSMsg(*current_depth, currentDepth_);
  tfListener_.waitForTransform("/local_origin",currentBoundingBoxes_.image_header.frame_id, currentBoundingBoxes_.image_header.stamp, ros::Duration(3.0));

  check_and_publish();
}