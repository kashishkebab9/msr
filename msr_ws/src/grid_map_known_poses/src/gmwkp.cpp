#include "gmwkp.h"
#include <exception>
#include <iterator>

//ASSUMPTION: Delay in tf of pointcloud due to rosbag delay that's implemented

void gm_known_pose::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  grid_map::Position robot_position;
  robot_position.x() = msg->pose.position.x;
  robot_position.y() = msg->pose.position.y;

  bool check = this->gm.getIndex(robot_position, this->robot_index);
  if (!check) {
    ROS_WARN("Robot is out of map!");
  }
}

void gm_known_pose::pcl_callback(const sensor_msgs::PointCloud::ConstPtr& msg) {
  // Need to convert the pointcloud data to grid_map data -[x]
  // Might be easier to use our scan_to_pcl node -[x]
  // These are points relative to the laser frame, so we need to transform them into the world frame first-[x]
  tf2_ros::TransformListener listener(this->tfBuffer);
  
  grid_map::Index robot_index_ = this->robot_index;
  std::vector<grid_map::Index> pcl_indices;

  ros::Time time = ros::Time::now();
  std::cout << "Time: " << time << std::endl;

  ros::Duration quarter_sec(0.25);

  std::string * map_odom_check_err = new std::string();
  std::string * odom_base_check_err = new std::string();
  bool map_odom_check = this->tfBuffer.canTransform(this->odom_frame, this->map_frame, time, quarter_sec, map_odom_check_err);
  bool odom_base_check = this->tfBuffer.canTransform(this->base_link_frame, this->map_frame, time, quarter_sec, odom_base_check_err);

  if (map_odom_check && odom_base_check) {

    tf2::Stamped<tf2::Transform> t_odom_map;
    tf2::fromMsg(this->tfBuffer.lookupTransform(this->odom_frame, this->base_link_frame, time), t_odom_map);

    std::vector<tf2::Vector3> tf_points;
    for (const auto& point: msg->points) {
      tf2::Vector3 pt(point.x, point.y, 1);
      tf2::Vector3 tf_point = t_odom_map * pt;
      tf_points.push_back(tf_point);
    }
    
    visualization_msgs::Marker transformed_points;
    transformed_points.action = visualization_msgs::Marker::DELETEALL;
    transformed_points.header.frame_id = "scarab41/odom";
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

    //We want to convert each of these transformed positions into indices for the bresenham algorithm
    std::vector<grid_map::Index> pts_as_indices;
    for (auto x: tf_points){
      grid_map::Position pt_position;
      grid_map::Index pt_index;
      pt_position.x() = x.x();
      pt_position.y() = x.y();

      this->gm.getIndex(pt_position, pt_index);
      pts_as_indices.push_back(pt_index);
    }

    std::vector<std::pair<grid_map::Index, float>> pts_to_process;
    std::cout << "Size before: " << pts_to_process.size() << std::endl;
    for (grid_map::Index pt : pts_as_indices){
      std::vector<std::pair<grid_map::Index, float>> line = this->inv_sensor_model(robot_index_, pt);
      std::copy(line.begin(), line.end(), std::back_inserter(pts_to_process));
    }
    std::cout << "Size After: " << pts_to_process.size() << std::endl;

  }else {
    ROS_WARN("Can't find tf!");
    std::cout << *map_odom_check_err << std::endl;
    std::cout << *odom_base_check_err << std::endl;
  }

}

std::vector<std::pair<grid_map::Index, float>> gm_known_pose::inv_sensor_model(grid_map::Index src, grid_map::Index target){

  std::vector<std::pair<grid_map::Index, float>> output;
  for (grid_map::LineIterator iterator(this->gm, src, target); !iterator.isPastEnd(); ++iterator) {
    std::pair<grid_map::Index, float> point_prob_pair;
    if(target.isApprox(*iterator)) {
      (*iterator, this->p_free);
      output.push_back(point_prob_pair);
    } else {
      std::pair<grid_map::Index, float> point_prob_pair(*iterator, this->p_occ);
      output.push_back(point_prob_pair);
    }

  }
  return output;


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
