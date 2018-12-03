#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <ros/ros.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PoseStamped.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>

#include <boost/bind.hpp>

#define CV_PI   3.1415926535897932384626433832795

class ObjectLocalizer{
    public:
        ObjectLocalizer(ros::NodeHandle* nodehandle, int& queue, std::string& darknet_bounding_boxes, std::string& pose, std::string& camera_info, std::string& detection_class, std::string& published_pose);
        ~ObjectLocalizer();
        
        tf::StampedTransform get_goal_camera_transform(const darknet_ros_msgs::BoundingBox& goal_bounding_box);
        geometry_msgs::PoseStamped goal_camera_frame(const darknet_ros_msgs::BoundingBox& goal_bounding_box);
        geometry_msgs::PoseStamped goal_transform_frame(const std::string& target_frame, const geometry_msgs::PoseStamped& goal_camera);
        darknet_ros_msgs::BoundingBox select_largest_probability(const darknet_ros_msgs::BoundingBoxes& current_bounding_boxes);
        darknet_ros_msgs::BoundingBoxes select_detection_class(const darknet_ros_msgs::BoundingBoxes& current_bounding_boxes, std::string& class_name );

        void setParam(const darknet_ros_msgs::BoundingBoxes& current_bounding_box, const geometry_msgs::PoseStamped& curent_pose, const sensor_msgs::CameraInfo& cam_info);
        void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& current_bounding_box, const geometry_msgs::PoseStamped::ConstPtr& curent_pose, const sensor_msgs::CameraInfoConstPtr& cam_info);

        geometry_msgs::PoseStamped goal_local;

    private:
        ros::NodeHandle nh_;

        std::string detectionClass_;
        //nh_.getParam("detection_class", detectionClass_);
        


        ros::Publisher pubGoal_;
        //nh_.getParam("published_pose_topic", pubGoal_);


        message_filters::Subscriber<sensor_msgs::CameraInfo> subCameraInfo_;
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> subBoundingBoxes_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> subPose_;
        typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, geometry_msgs::PoseStamped, sensor_msgs::CameraInfo> MySyncPolicy; 
        message_filters::Synchronizer<MySyncPolicy> sync_;
        
        darknet_ros_msgs::BoundingBoxes currentBoundingBoxes_;
        geometry_msgs::PoseStamped currentPose_;
        
        sensor_msgs::CameraInfo cameraInfo_;

        tf::TransformListener tfListener_;
        
};

#endif  // LOCALIZER_H