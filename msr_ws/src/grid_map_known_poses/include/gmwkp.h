#include <iostream>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/OccupancyGrid.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

class gm_known_pose {
  public:
    gm_known_pose() : 
      map_frame("scarab41/map"),
      odom_frame("scarab41/odom"),
      base_link_frame("scarab41/laser"),
      p_occ(.7),
      prior(.5),
      p_free(.4)
  {
      pose_sub = nh.subscribe("/scarab41/pose", 1, &gm_known_pose::pose_callback, this);
      scan_sub = nh.subscribe("/pointcloud", 1, &gm_known_pose::pcl_callback, this);
      tf_pcl_viz = nh.advertise<visualization_msgs::Marker>("tf_pcl", 1);
      occ_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_by_kash", 1);

      grid_map::Length length;
      length << 100, 100;
      this->gm.setGeometry(length, .05);
      this->gm.add("occ_grid_map", 0.5);
      
      std::cout << "Grid Map size: " << this->gm.getSize()(0) << ", " << this->gm.getSize()(1) << std::endl;
    }

  private:
    ros::NodeHandle nh, pnh;

    ros::Subscriber pose_sub;
    ros::Subscriber scan_sub;
    ros::Publisher tf_pcl_viz;
    ros::Publisher occ_grid_pub;

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pcl_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
    std::vector<std::pair<grid_map::Index, float>> inv_sensor_model(grid_map::Index src, grid_map::Index target);
    double p_2_lo(double prob);
    double lo_2_p(double log_odds);

    grid_map::Index robot_index;
    grid_map::GridMap gm;

    tf2_ros::Buffer tfBuffer;
    std::string map_frame;
    std::string odom_frame;
    std::string base_link_frame;
    float p_occ;
    float prior;
    float p_free;
};

