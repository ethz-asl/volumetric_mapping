#include "octomap_world/octomap_manager.h"

#include <glog/logging.h>
#include <minkindr_conversions/kindr_tf.h>

namespace volumetric_mapping {

OctomapManager::OctomapManager(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      Q_(Eigen::Matrix4d::Identity()) {
  setParametersFromROS();
  subscribe();
  advertiseServices();
  advertisePublishers();
}

void OctomapManager::setParametersFromROS() {
  OctomapParameters params;
  nh_private_.param("tf_frame", world_frame_, world_frame_);
  nh_private_.param("resolution", params.resolution, params.resolution);
  nh_private_.param("probability_hit", params.probability_hit,
                    params.probability_hit);
  nh_private_.param("probability_miss", params.probability_miss,
                    params.probability_miss);
  nh_private_.param("threshold_min", params.threshold_min,
                    params.threshold_min);
  nh_private_.param("threshold_max", params.threshold_max,
                    params.threshold_max);
  nh_private_.param("filter_speckles", params.filter_speckles,
                    params.filter_speckles);
  nh_private_.param("sensor_max_range", params.sensor_max_range,
                    params.sensor_max_range);

  // Set the parent class parameters.
  setOctomapParameters(params);
}

void OctomapManager::subscribe() {
  left_info_sub_ = nh_.subscribe("cam0/camera_info", 1,
                                 &OctomapManager::leftCameraInfoCallback, this);
  right_info_sub_ = nh_.subscribe(
      "cam1/camera_info", 1, &OctomapManager::rightCameraInfoCallback, this);
  disparity_sub_ = nh_.subscribe(
      "disparity", 40, &OctomapManager::insertDisparityImageWithTf, this);
  pointcloud_sub_ = nh_.subscribe(
      "pointcloud", 40, &OctomapManager::insertPointcloudWithTf, this);
}

void OctomapManager::advertiseServices() {
  reset_map_service_ = nh_private_.advertiseService(
      "reset_map", &OctomapManager::resetMapCallback, this);
  publish_all_service_ = nh_private_.advertiseService(
      "publish_all", &OctomapManager::publishAllCallback, this);
  get_map_service_ = nh_private_.advertiseService(
      "get_map", &OctomapManager::getOctomapCallback, this);
  save_octree_service_ = nh_private_.advertiseService(
      "save_map", &OctomapManager::saveOctomapCallback, this);
  load_octree_service_ = nh_private_.advertiseService(
      "load_map", &OctomapManager::loadOctomapCallback, this);
}

void OctomapManager::advertisePublishers() {
  occupied_nodes_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "octomap_occupied", 1, true);
  free_nodes_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "octomap_free", 1, true);

  binary_map_pub_ =
      nh_private_.advertise<octomap_msgs::Octomap>("octomap_binary", 1, true);
  full_map_pub_ =
      nh_private_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
}

void OctomapManager::publishAll() {
  visualization_msgs::MarkerArray occupied_nodes, free_nodes;
  generateMarkerArray(world_frame_, &occupied_nodes, &free_nodes);

  occupied_nodes_pub_.publish(occupied_nodes);
  free_nodes_pub_.publish(free_nodes);

  octomap_msgs::Octomap binary_map, full_map;
  getOctomapBinaryMsg(&binary_map);
  getOctomapFullMsg(&full_map);

  binary_map_pub_.publish(binary_map);
  full_map_pub_.publish(full_map);
}

bool OctomapManager::resetMapCallback(std_srvs::Empty::Request& request,
                                      std_srvs::Empty::Response& response) {
  resetMap();
  return true;
}

bool OctomapManager::publishAllCallback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response) {
  publishAll();
  return true;
}

bool OctomapManager::getOctomapCallback(
    octomap_msgs::GetOctomap::Request& request,
    octomap_msgs::GetOctomap::Response& response) {
  return getOctomapFullMsg(&response.map);
}

bool OctomapManager::loadOctomapCallback(
    volumetric_msgs::LoadMap::Request& request,
    volumetric_msgs::LoadMap::Response& response) {
  return loadOctomapFromFile(request.file_path);
}

bool OctomapManager::saveOctomapCallback(
    volumetric_msgs::SaveMap::Request& request,
    volumetric_msgs::SaveMap::Response& response) {
  return writeOctomapToFile(request.file_path);
}

void OctomapManager::leftCameraInfoCallback(
    const sensor_msgs::CameraInfoPtr& left_info) {
  left_info_ = left_info;
  if (left_info_ && right_info_) {
    calculateQ();
  }
}
void OctomapManager::rightCameraInfoCallback(
    const sensor_msgs::CameraInfoPtr& right_info) {
  right_info_ = right_info;
  if (left_info_ && right_info_) {
    calculateQ();
  }
}

void OctomapManager::calculateQ() {
  Q_ = getQForROSCameras(*left_info_, *right_info_);
  full_image_size_.x() = left_info_->width;
  full_image_size_.y() = left_info_->height;
}

void OctomapManager::insertDisparityImageWithTf(
    const stereo_msgs::DisparityImageConstPtr& disparity) {
  if (!(left_info_ && right_info_)) {
    ROS_WARN("No camera info available yet, skipping adding disparity.");
    return;
  }

  // Look up transform from sensor frame to world frame.
  Transformation sensor_to_world;
  if (lookupTransform(disparity->header.frame_id, world_frame_,
                      disparity->header.stamp, &sensor_to_world)) {
    insertDisparityImage(sensor_to_world, disparity, Q_, full_image_size_);
  }
}

void OctomapManager::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud) {
  // Look up transform from sensor frame to world frame.
  Transformation sensor_to_world;
  if (lookupTransform(pointcloud->header.frame_id, world_frame_,
                      pointcloud->header.stamp, &sensor_to_world)) {
    insertPointcloud(sensor_to_world, pointcloud);
  }
}

bool OctomapManager::lookupTransform(const std::string& from_frame,
                                     const std::string& to_frame,
                                     const ros::Time& timestamp,
                                     Transformation* transform) {
  tf::StampedTransform tf_transform;
  try {
    tf_listener_.lookupTransform(to_frame, from_frame, timestamp, tf_transform);
  }
  catch (tf::TransformException& ex) {
    ROS_ERROR_STREAM(
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }
  tf::transformTFToKindr(tf_transform, transform);
  return true;
}

}  // namespace volumetric_mapping
