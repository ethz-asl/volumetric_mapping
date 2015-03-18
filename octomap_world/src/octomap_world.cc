#include "octomap_world/octomap_world.h"

#include <glog/logging.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

namespace volumetric_mapping {

void OctomapWorld::resetMap() {
  CHECK(octree_) << "Octree uninitialized!";
  octree_->clear();
}

void OctomapWorld::setOctomapParameters(const OctomapParameters& params) {
  LOG(FATAL) << "TODO!";
}

void OctomapWorld::insertDisparityImage(
    const Transformation& sensor_to_world,
    const stereo_msgs::DisparityImageConstPtr& disparity) {
  LOG(FATAL) << "TODO!";
}

void OctomapWorld::insertPointcloud(
    const Transformation& sensor_to_world,
    const sensor_msgs::PointCloud2::ConstPtr& cloud) {
  LOG(FATAL) << "TODO!";
}

OctomapWorld::CellStatus OctomapWorld::getCellStatusBoundingBox(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& bounding_box_size) const {
  return CellStatus::kUnknown;
}

OctomapWorld::CellStatus OctomapWorld::getCellStatusPoint(
    const Eigen::Vector3d& point) const {
  return CellStatus::kUnknown;
}

double OctomapWorld::getResolution() const {
  CHECK(octree_) << "Octree uninitialized!";
  return octree_->getResolution();
}

bool OctomapWorld::getOctomapBinaryMsg(octomap_msgs::Octomap* msg) const {
  return octomap_msgs::binaryMapToMsg(*octree_, *msg);
}

bool OctomapWorld::getOctomapFullMsg(octomap_msgs::Octomap* msg) const {
  return octomap_msgs::fullMapToMsg(*octree_, *msg);
}

void OctomapWorld::setOctomapFromMsg(const octomap_msgs::Octomap& msg) {
  if (msg.binary) {
    setOctomapFromBinaryMsg(msg);
  } else {
    setOctomapFromFullMsg(msg);
  }
}

void OctomapWorld::setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg) {
  octree_.reset(octomap_msgs::binaryMsgToMap(msg));
}

void OctomapWorld::setOctomapFromFullMsg(const octomap_msgs::Octomap& msg) {
  octree_.reset(
      dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(msg)));
}

bool OctomapWorld::loadOctomapFromFile(const std::string& filename) {
  LOG(FATAL) << "TODO!";
}

bool OctomapWorld::writeOctomapToFile(const std::string& filename) const {
  LOG(FATAL) << "TODO!";
}

}  // namespace volumetric_mapping
