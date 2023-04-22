#include "gmwkp.h"
#include <exception>

void gm_known_pose::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  this->current_pos <<  msg->pose.position.x, msg->pose.position.y, 1;
}

void gm_known_pose::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "grid_mapping_with_known_poses");
  gm_known_pose solver;

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;

}
