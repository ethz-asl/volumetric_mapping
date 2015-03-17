#ifndef OCTOMAP_WORLD_OCTOMAP_WORLD_H_
#define OCTOMAP_WORLD_OCTOMAP_WORLD_H_

#include <string>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <volumetric_map_base/world_base.h>

namespace volumetric_mapping {

struct OctomapParameters {
  OctomapParameters() {
    // Set reasonable defaults here...
  }

  double resolution;
  // TODO(helenol): fill rest in.
};

// A wrapper around octomap that allows insertion from various ROS message
// data sources, given their transforms from sensor frame to world frame.
// Does not need to run within a ROS node, does not do any TF look-ups, and
// does not publish/subscribe to anything (though provides serialization
// and deserialization functions to and from ROS messages).
class OctomapWorld : public WorldBase {
  typedef std::shared_ptr<OctomapWorld> Ptr;

 public:
  // Default constructor - if this one is called, you MUST call
  // setOctomapParameters() before calling any other functions.
  OctomapWorld() {}

  // Creates an octomap with the correct parameters.
  OctomapWorld(const OctomapParameters& params) {}
  virtual ~OctomapWorld() {}

  // General map management.
  void resetMap();
  // Creates an octomap if one is not yet created or if the resolution of the
  // current varies from the parameters requested.
  void setOctomapParameters(const OctomapParameters& params);

  // Virtual functions for inserting data.
  virtual void insertDisparityMap(
      const Transformation& sensor_to_world,
      const stereo_msgs::DisparityImageConstPtr& disparity);

  virtual void insertPointcloud(
      const Transformation& sensor_to_world,
      const sensor_msgs::PointCloud2::ConstPtr& cloud);

  // Virtual functions for outputting map status.
  virtual CellStatus getCellStatus(const Eigen::Vector3d& point,
                                   const Eigen::Vector3d& bounding_box) const;

  virtual double getResolution() const;

  // Serialization and deserialization from ROS messages.
  bool getOctomapBinaryMsg(octomap_msgs::Octomap* msg) const;
  bool getOctomapFullMsg(octomap_msgs::Octomap* msg) const;
  // Clears the current octomap and replaces it with one from the message.
  void setOctomapFromMsg(const octomap_msgs::Octomap& msg);
  void setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg);
  void setOctomapFromFullMsg(const octomap_msgs::Octomap& msg);

  // Loading and writing to disk.
  bool loadOctomapFromFile(const std::string& filename);
  bool writeOctomapToFile(const std::string& filename) const;

 private:
  std::shared_ptr<octomap::OcTree> octree_;
};

}  // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_WORLD_H_
