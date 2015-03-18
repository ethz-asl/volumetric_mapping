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
  if (octree_) {
    if (octree_->getResolution() != params.resolution) {
      LOG(WARNING) << "Octomap resolution has changed! Deleting tree!";
      octree_.reset(new octomap::OcTree(params.resolution));
    }
  } else {
    octree_.reset(new octomap::OcTree(params.resolution));
  }

  octree_->setProbHit(params.probability_hit);
  octree_->setProbMiss(params.probability_miss);
  octree_->setClampingThresMin(params.threshold_min);
  octree_->setClampingThresMax(params.threshold_max);
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

void OctomapWorld::setLogOddsBoundingBox(const Eigen::Vector3d& position,
                             const Eigen::Vector3d& bounding_box,
                             double log_odds_value) {
  CHECK(octree_) << "Octree uninitialized!";

  const bool lazy_eval = true;
  const double resolution = octree_->getResolution();
  const double epsilon = 0.001; // Small offset to not hit boundary of nodes.

  for (double x_position = position.x() - bounding_box.x()/2 - epsilon;
      x_position <= position.x() + bounding_box.x()/2 + epsilon;
      x_position += resolution) {
    for (double y_position = position.y() - bounding_box.y()/2 - epsilon;
         y_position <= position.y() + bounding_box.y()/2 + epsilon;
         y_position += resolution) {
      for (double z_position = position.z() - bounding_box.z()/2 - epsilon;
           z_position <= position.z() + bounding_box.z()/2 + epsilon;
           z_position += resolution) {
        octomap::point3d point = octomap::point3d(x_position, y_position, z_position);
        octree_->setNodeValue(point, log_odds_value, lazy_eval);
      }
    }
  }
  octree_->updateInnerOccupancy(); // TODO(burrimi): check if necessary.
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


bool OctomapWorld::isSpeckleNode(const octomap::OcTreeKey& key) const {
  octomap::OcTreeKey current_key;
  // Search neighbors in a +/-1 key range cube. If there are neighbors, it's
  // not a speckle.
  bool neighbor_found = false;
  for (current_key[2] = key[2] - 1;
       current_key[2] <= key[2] + 1;
       ++current_key[2]) {
    for (current_key[1] = key[1] - 1;
        current_key[1] <= key[1] + 1;
         ++current_key[1]) {
      for (current_key[0] = key[0] - 1;
           current_key[0] <= key[0] + 1;
           ++current_key[0]) {
        if (current_key != key) {
          octomap::OcTreeNode* node = octree_->search(key);
          if (node && octree_->isNodeOccupied(node)) {
            // we have a neighbor => break!
            return false;
          }
        }
      }
    }
  }
  return true;
}

}  // namespace volumetric_mapping
