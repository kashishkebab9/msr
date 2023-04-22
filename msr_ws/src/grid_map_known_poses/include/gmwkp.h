#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "GridMap.h"
#include "grid_map_ros.hpp"


class gm_known_pose {
  public:
    gm_known_pose() {
      pose_sub = nh.subscribe("/scarab41/pose", 1, &gm_known_pose::poseCallback, this);
      scan_sub = nh.subscribe("/scarab41/scan", 1, &gm_known_pose::scanCallback, this);
    }

  private:
    ros::NodeHandle nh, pnh;

    ros::Subscriber pose_sub;
    ros::Subscriber scan_sub;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    Eigen::Vector3d current_pos;





};

