#include "octomap_world/octomap_manager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "octomap_manager");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  volumetric_mapping::OctomapManager manager(nh, nh_private);

  // After creating the manager, if the octomap_file parameter is set,
  // load the octomap at that path and publish it.
  std::string octomap_file;
  if (nh_private.getParam("octomap_file", octomap_file)) {
    if (manager.loadOctomapFromFile(octomap_file)) {
      ROS_INFO_STREAM("Successfully loaded octomap from path: " << octomap_file);
      manager.publishAll();
    } else {
      ROS_ERROR_STREAM("Could not load octomap from path: " << octomap_file);
    }
  }

  ros::spin();
  return 0;
}
