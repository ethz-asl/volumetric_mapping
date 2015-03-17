#ifndef OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
#define OCTOMAP_WORLD_OCTOMAP_MANAGER_H_

#include "octomap_world/octomap_world.h"

namespace volumetric_mapping {

// An inherited class from OctomapWorld, which also handles the connection to
// ROS via publishers, subscribers, service calls, etc.
class OctomapManager : public OctomapWorld {
 public:
  typedef std::shared_ptr<OctomapManager> Ptr;

  // By default, loads octomap parameters from the ROS parameter server.
  OctomapManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) {
  }

  // Sets up subscriptions based on ROS node parameters.
  void subscribe();
  void advertiseServices();
  void advertisePublishers();

  void publishAll();
  void publishOccupied();
  void publishFree();
  void publishUnknown();

  // TODO(helenol): figure out which services we need again.
  void someServiceCallbacks();
};

}  // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
