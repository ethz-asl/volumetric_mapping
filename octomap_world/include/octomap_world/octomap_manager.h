#ifndef OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
#define OCTOMAP_WORLD_OCTOMAP_MANAGER_H_

#include "octomap_world/octomap_world.h"

#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <octomap_msgs/GetOctomap.h>
#include <volumetric_msgs/LoadMap.h>
#include <volumetric_msgs/SaveMap.h>

namespace volumetric_mapping {

// An inherited class from OctomapWorld, which also handles the connection to
// ROS via publishers, subscribers, service calls, etc.
class OctomapManager : public OctomapWorld {
 public:
  typedef std::shared_ptr<OctomapManager> Ptr;

  // By default, loads octomap parameters from the ROS parameter server.
  OctomapManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void publishAll();
  void publishAllEvent(const ros::TimerEvent& e);

  // Data insertion callbacks with TF frame resolution through the listener.
  void insertDisparityImageWithTf(
      const stereo_msgs::DisparityImageConstPtr& disparity);
  void insertPointcloudWithTf(
      const sensor_msgs::PointCloud2::ConstPtr& pointcloud);

  // Camera info callbacks.
  void leftCameraInfoCallback(const sensor_msgs::CameraInfoPtr& left_info);
  void rightCameraInfoCallback(const sensor_msgs::CameraInfoPtr& right_info);

  // Service callbacks.
  bool resetMapCallback(std_srvs::Empty::Request& request,
                        std_srvs::Empty::Response& response);
  bool publishAllCallback(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& response);
  bool getOctomapCallback(octomap_msgs::GetOctomap::Request& request,
                          octomap_msgs::GetOctomap::Response& response);

  bool loadOctomapCallback(volumetric_msgs::LoadMap::Request& request,
                           volumetric_msgs::LoadMap::Response& response);

  bool saveOctomapCallback(volumetric_msgs::SaveMap::Request& request,
                           volumetric_msgs::SaveMap::Response& response);

 private:
  // Sets up subscriptions based on ROS node parameters.
  void setParametersFromROS();
  void subscribe();
  void advertiseServices();
  void advertisePublishers();

  bool setQFromParams(std::vector<double>* Q_vec);
  void calculateQ();
  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener tf_listener_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  // frame.
  std::string world_frame_;

  // Subscriptions for input sensor data.
  ros::Subscriber disparity_sub_;
  ros::Subscriber left_info_sub_;
  ros::Subscriber right_info_sub_;
  ros::Subscriber pointcloud_sub_;

  // Publish full state of octomap.
  ros::Publisher binary_map_pub_;
  ros::Publisher full_map_pub_;

  // Publish markers for visualization.
  ros::Publisher occupied_nodes_pub_;
  ros::Publisher free_nodes_pub_;

  // Services!
  ros::ServiceServer reset_map_service_;
  ros::ServiceServer publish_all_service_;
  ros::ServiceServer get_map_service_;
  ros::ServiceServer save_octree_service_;
  ros::ServiceServer load_octree_service_;

  // Keep state of the cameras.
  sensor_msgs::CameraInfoPtr left_info_;
  sensor_msgs::CameraInfoPtr right_info_;

  // Only calculate Q matrix for disparity once.
  bool Q_initialized_;
  Eigen::Matrix4d Q_;
  Eigen::Vector2d full_image_size_;
  double map_publish_frequency_;
  ros::Timer map_publish_timer_;
};

}  // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
