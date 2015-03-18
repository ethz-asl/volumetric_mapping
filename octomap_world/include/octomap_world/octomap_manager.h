#ifndef OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
#define OCTOMAP_WORLD_OCTOMAP_MANAGER_H_

#include "octomap_world/octomap_world.h"

#include <tf/transform_listener.h>

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

  // Data insertion callbacks with TF frame resolution through the listener.
  void insertDisparityImageWithTf(
      const stereo_msgs::DisparityImageConstPtr& disparity);
  void insertPointcloudWithTf(
      const sensor_msgs::PointCloud2::ConstPtr& disparity);

  // TODO(helenol): figure out which services we need again.
  void someServiceCallbacks();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener tf_listener_;
};

}  // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
