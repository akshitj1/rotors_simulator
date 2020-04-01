#include <ros/ros.h>

#include "tailsitter_controller_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_node");

  ros::NodeHandle nh;
  rotors_control::TailsitterControllerNode tailsitter_controller_node(nh);

  ros::spin();

  return 0;
}
