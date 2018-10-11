#ifndef grid_map_handler_H_
#define grid_map_handler_H_

#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mutex>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <laser_geometry/laser_geometry.h>
#include <costmap_2d/observation_buffer.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>

#include "grid_map_handler/gmh_definitions.h"

namespace grid_map_handler{ //TODO write doc

class GridMapHandler{

public:

  GridMapHandler(const std::string &fname, double res, double local_map_length, double inflation_radius,
            bool use_polygon, tf::TransformListener &tf, grid_map::GridMap &global_grid_map,
            grid_map::GridMap &local_gridmap);

  /**
   * @brief Destructor
   */
  virtual ~GridMapHandler();

  bool start();

  bool stop();

  /**
   * @brief  A callback to handle buffering LaserScan messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message);

private:
  std::mutex local_map_lock_;

  double lastCall_;

  bool first_pos_call_;

  bool first_laser_call_;

  bool use_polygon_;

  bool move_, broadcast_, merge_;

  double inflation_radius_;

  double robot_angle_;

  grid_map::Position robot_pos_;

  cv::Mat kernel_;

  grid_map::GridMap& global_map_;

  grid_map::GridMap& local_map_;

  ros::Time lastScan_;

  ros::Time last_moved_;

  ros::Publisher static_map_pub_;

  ros::Publisher static_inf_map_pub_;

  ros::Publisher inflated_map_pub_;

  ros::Publisher local_map_pub_;

  ros::Publisher local_map_pub_infl_;

  laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds

  tf::TransformListener& tf_;

  tf::TransformBroadcaster tf_bc_;

  void cropImage(const std::string& mapfname, cv::Mat& cropped);

  void loadImage(std::string mapfname, double pic_res, double res);

  void mergeInflationLayers();

  void updateLocalInflation();

  void publishLocal();

  void publishGlobal();

  void tfRotationBroadcast();

  void move();

  ros::Subscriber sub_scan_, sub_pos_, sub_rot_;

  std::shared_ptr<std::thread> merge_thread_ptr_, move_thread_ptr_, rotation_broadcast_thread_ptr_;
};

} /* namespace grid_map_handler */
#endif