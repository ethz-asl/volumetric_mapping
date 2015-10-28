#include "octomap_world/octomap_manager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "octomap_manager");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  volumetric_mapping::OctomapManager manager(nh, nh_private);

  ros::spin();
  return 0;
}
