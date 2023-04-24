#include "gmwkp.h"
#include <exception>

void gm_known_pose::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  grid_map::Position robot_position;
  robot_position.x() = msg->pose.position.x;
  robot_position.y() = msg->pose.position.y;

  this->gm.getIndex(robot_position, this->robot_index);
}

void gm_known_pose::pcl_callback(const sensor_msgs::PointCloud::ConstPtr& msg) {
  // Need to convert the pointcloud data to grid_map data
  // Might be easier to use our scan_to_pcl node 
  // And then convert to grid_map since we know the robot pose and scan data
  // These are points relative to the laser frame, so we need to transform them into the world frame first
  tf2_ros::TransformListener listener(this->tfBuffer);
  
  grid_map::Index robot_index_ = this->robot_index;
  std::vector<grid_map::Index> pcl_indices;

  ros::Time time = ros::Time::now();
  std::cout << "Time: " << time << std::endl;

  ros::Duration quarter_sec(0.20);

  std::string * map_base_check_err = new std::string();
  bool map_base_check = this->tfBuffer.canTransform(this->base_link_frame, this->map_frame, time, quarter_sec, map_base_check_err);

  if (map_base_check) {

    tf2::Stamped<tf2::Transform> t_base_map;
    tf2::fromMsg(this->tfBuffer.lookupTransform(this->base_link_frame, this->map_frame, time), t_base_map);

    std::vector<tf2::Vector3> tf_points;
    for (const auto& point: msg->points) {
      tf2::Vector3 pt(point.x, point.y, 1);
      tf2::Vector3 tf_point = t_base_map.inverse() * pt;
      tf_points.push_back(tf_point);
    }
    
    visualization_msgs::Marker transformed_points;
    transformed_points.action = visualization_msgs::Marker::DELETEALL;
    transformed_points.header.frame_id = "scarab41/map";
    transformed_points.ns = "transformed_pcl";
    transformed_points.action = visualization_msgs::Marker::ADD;
    transformed_points.pose.orientation.w = 1.0;
    transformed_points.id = 0;
    transformed_points.type = visualization_msgs::Marker::POINTS;

    transformed_points.scale.x = 0.05;
    transformed_points.scale.y = 0.05;
    transformed_points.color.g = 1.0f;
    transformed_points.color.b = 1.0f;
    transformed_points.color.a = 1.0;

    for (auto x: tf_points)  {
      geometry_msgs::Point point;
      point.x = x.x();
      point.y = x.y();
      point.z = 0;
      transformed_points.points.push_back(point);
    }

    this->tf_pcl_viz.publish(transformed_points);

  }else {
    ROS_WARN("Can't find tf!");
    std::cout << *map_base_check_err << std::endl;
  }


}

void gm_known_pose::inv_sensor_model(grid_map::Index src, grid_map::Index target){

}

std::vector<grid_map::Index> gm_known_pose::bresenham(grid_map::Index src, grid_map::Index target){


}

//-------------------------------------------------------//
int main(int argc, char **argv) {
  ros::init(argc, argv, "grid_mapping_with_known_poses");
  gm_known_pose solver;

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;

}
