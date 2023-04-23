#include <iostream>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

class gm_known_pose {
  public:
    gm_known_pose() : 
      map_frame("scarab41/map"),
      base_link_frame("scarab41/base_link")
  {
      pose_sub = nh.subscribe("/scarab41/pose", 1, &gm_known_pose::pose_callback, this);
      scan_sub = nh.subscribe("/pointcloud", 1, &gm_known_pose::pcl_callback, this);
      tf_pcl_viz = nh.advertise<visualization_msgs::Marker>("tf_pcl", 1);

      this->gm.add("occ_grid_map", 0.5);

      //listener allows buffer to consume and store all transforms living in /tf
      tf2_ros::TransformListener listener(this->tfBuffer);

    }

  private:
    ros::NodeHandle nh, pnh;

    ros::Subscriber pose_sub;
    ros::Subscriber scan_sub;
    ros::Publisher tf_pcl_viz;


    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pcl_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void inv_sensor_model(grid_map::Index src, grid_map::Index target);
    std::vector<grid_map::Index> bresenham(grid_map::Index src, grid_map::Index target);

    grid_map::Index robot_index;
    grid_map::GridMap gm;

    tf2_ros::Buffer tfBuffer;
    std::string map_frame;
    std::string base_link_frame;






};

