#ifndef OCTOMAP_WORLD_OCTOMAP_WORLD_H_
#define OCTOMAP_WORLD_OCTOMAP_WORLD_H_

#include <string>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <volumetric_map_base/world_base.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

namespace volumetric_mapping {

struct OctomapParameters {
  OctomapParameters()
      : resolution(0.15),
        probability_hit(0.65),
        probability_miss(0.4),
        threshold_min(0.12),
        threshold_max(0.97),
        filter_speckles(true),
        sensor_max_range(5.0),
        visualize_min_z(-std::numeric_limits<double>::max()),
        visualize_max_z(std::numeric_limits<double>::max()) {
    // Set reasonable defaults here...
    // TODO(helenol): use params from OctomapProvider defaults or Sammy configs?
  }

  // Resolution for the Octree. It is not possible to change this without
  // creating a new Octree.
  double resolution;
  // Hit probabilities for pointcloud data.
  double probability_hit;
  double probability_miss;
  // Clamping thresholds for pruning: above and below these thresholds, all
  // values are treated the same.
  double threshold_min;
  double threshold_max;

  // Filter neighbor-less nodes as 'speckles'.
  bool filter_speckles;

  // Maximum range to allow a sensor measurement. Negative values to not
  // filter.
  double sensor_max_range;

  // Minimum and maximum z to visualize. Only used for marker, not full
  // octomap, visualization.
  double visualize_min_z;
  double visualize_max_z;
};

// A wrapper around octomap that allows insertion from various ROS message
// data sources, given their transforms from sensor frame to world frame.
// Does not need to run within a ROS node, does not do any TF look-ups, and
// does not publish/subscribe to anything (though provides serialization
// and deserialization functions to and from ROS messages).
class OctomapWorld : public WorldBase {
  typedef std::shared_ptr<OctomapWorld> Ptr;

 public:
  // Default constructor - creates a valid octree using parameter defaults.
  OctomapWorld();

  // Creates an octomap with the correct parameters.
  OctomapWorld(const OctomapParameters& params);
  virtual ~OctomapWorld() {}

  // General map management.
  void resetMap();
  void prune();
  // Creates an octomap if one is not yet created or if the resolution of the
  // current varies from the parameters requested.
  void setOctomapParameters(const OctomapParameters& params);

  // Virtual functions for manually manipulating map probabilities.
  virtual void setFree(const Eigen::Vector3d& position,
                       const Eigen::Vector3d& bounding_box_size);
  virtual void setOccupied(const Eigen::Vector3d& position,
                           const Eigen::Vector3d& bounding_box_size);

  // Virtual functions for outputting map status.
  virtual CellStatus getCellStatusBoundingBox(
      const Eigen::Vector3d& point,
      const Eigen::Vector3d& bounding_box_size) const;
  virtual CellStatus getCellStatusPoint(const Eigen::Vector3d& point) const;
  virtual double getCellStatusPointProbability(const Eigen::Vector3d& point) const;
  virtual CellStatus getLineStatus(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& end) const;
  virtual CellStatus getVisibility(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test,
                                  bool stop_at_unknown_cell) const;
  virtual CellStatus getLineStatusBoundingBox(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& bounding_box_size) const;

  virtual double getResolution() const;
  virtual Eigen::Vector3d getMapCenter() const;
  virtual Eigen::Vector3d getMapSize() const;

  // Serialization and deserialization from ROS messages.
  bool getOctomapBinaryMsg(octomap_msgs::Octomap* msg) const;
  bool getOctomapFullMsg(octomap_msgs::Octomap* msg) const;
  // Clears the current octomap and replaces it with one from the message.
  void setOctomapFromMsg(const octomap_msgs::Octomap& msg);

  // Loading and writing to disk.
  bool loadOctomapFromFile(const std::string& filename);
  bool writeOctomapToFile(const std::string& filename);

  // Helpers for publishing.
  void generateMarkerArray(const std::string& tf_frame,
                           visualization_msgs::MarkerArray* occupied_nodes,
                           visualization_msgs::MarkerArray* free_nodes);

 protected:
  // Actual implementation for inserting disparity data.
  virtual void insertProjectedDisparityIntoMapImpl(
      const Transformation& sensor_to_world, const cv::Mat& projected_points);

  // Actual implementation for inserting pointclouds.
  virtual void insertPointcloudIntoMapImpl(
      const Transformation& sensor_to_world,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

 private:
  // Check if the node at the specified key has neighbors or not.
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  // Manually affect the probabilities of areas within a bounding box.
  void setLogOddsBoundingBox(const Eigen::Vector3d& position,
                             const Eigen::Vector3d& bounding_box_size,
                             double log_odds_value);

  // Helper functions for building up a map from sensor data.
  void castRay(const octomap::point3d& sensor_origin,
               const octomap::point3d& point, octomap::KeySet* free_cells,
               octomap::KeySet* occupied_cells) const;
  void updateOccupancy(octomap::KeySet* free_cells,
                       octomap::KeySet* occupied_cells);
  bool isValidPoint(const cv::Vec3f& point) const;

  void setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg);
  void setOctomapFromFullMsg(const octomap_msgs::Octomap& msg);

  double colorizeMapByHeight(double z, double min_z, double max_z) const;
  std_msgs::ColorRGBA percentToColor(double h) const;

  std::shared_ptr<octomap::OcTree> octree_;

  OctomapParameters params_;
};

}  // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_WORLD_H_
