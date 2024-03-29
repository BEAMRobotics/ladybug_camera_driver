#include "ros/ros.h"
#include <nodelet/loader.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ladybug_camera_node");

  // This is code based nodelet loading, the preferred nodelet launching is done
  // through roslaunch
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "ladybug_camera_driver/LadybugCameraNodelet",
               remap, nargv);

  ros::spin();

  return 0;
}
