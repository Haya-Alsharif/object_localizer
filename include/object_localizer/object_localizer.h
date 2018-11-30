#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

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
#include <boost/bind.hpp>

#define CV_PI   3.1415926535897932384626433832795

class ObjectLocalizer{
    public:
        ObjectLocalizer(ros::NodeHandle* nodehandle, int& q, std::string& darknet_bounding_boxes, std::string& camera_pointcloud);
        ~ObjectLocalizer();
        
        tf::StampedTransform get_goal_camera_transform(const darknet_ros_msgs::BoundingBox& goal_bounding_box);
        geometry_msgs::PoseStamped goal_camera_frame(const darknet_ros_msgs::BoundingBox& goal_bounding_box);
        geometry_msgs::PoseStamped goal_transform_frame(const std::string& target_frame, const geometry_msgs::PoseStamped& goal_camera);
        bool goal_within_range(const float& tolerance, const geometry_msgs::PoseStamped& goal_local);
        darknet_ros_msgs::BoundingBox select_largest_probability(const darknet_ros_msgs::BoundingBoxes& current_bounding_boxes);
        darknet_ros_msgs::BoundingBoxes select_detection_class(const darknet_ros_msgs::BoundingBoxes& current_bounding_boxes, std::string& class_name );

        void setParam(const darknet_ros_msgs::BoundingBoxes& current_bounding_box, const sensor_msgs::PointCloud2& current_depth, const geometry_msgs::PoseStamped& curent_pose);
        void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& current_bounding_box, const sensor_msgs::PointCloud2ConstPtr& current_depth, const geometry_msgs::PoseStamped::ConstPtr& curent_pose);

        bool mission_accomplished;

    private:
        ros::NodeHandle nh_;
        ros::Publisher pubGoal_;
        std::string detection_class;
        
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> subBoundingBoxes_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> subDepth_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> subPose_;
        typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy; 
        message_filters::Synchronizer<MySyncPolicy> sync_;

        unsigned int i;
        darknet_ros_msgs::BoundingBoxes currentBoundingBoxes_;
        pcl::PointCloud<pcl::PointXYZ> currentDepth_;
        geometry_msgs::PoseStamped currentPose_;

        tf::TransformListener tfListener_;
            
};

#endif  // LOCALIZER_H