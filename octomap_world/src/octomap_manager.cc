#include "octomap_world/octomap_manager.h"

#include <glog/logging.h>

namespace volumetric_mapping {

void OctomapManager::subscribe() {}
void OctomapManager::advertiseServices() {}
void OctomapManager::advertisePublishers() {}
void OctomapManager::publishAll() {}
void OctomapManager::publishOccupied() {}
void OctomapManager::publishFree() {}
void OctomapManager::publishUnknown() {}
void OctomapManager::someServiceCallbacks() {}

void OctomapManager::insertDisparityImageWithTf(
    const stereo_msgs::DisparityImageConstPtr& disparity) {}
void OctomapManager::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& disparity) {}

}  // namespace volumetric_mapping
