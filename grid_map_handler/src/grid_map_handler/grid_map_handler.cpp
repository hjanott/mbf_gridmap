#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <ros/console.h>
#include <yaml-cpp/yaml.h>
#include <LinearMath/btQuaternion.h>

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "grid_map_handler/grid_map_handler.h"

namespace grid_map_handler{

  #ifdef HAVE_YAMLCPP_GT_0_5_0
    // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i)
  {
    i = node.as<T>();
  }
  #endif

  GridMapHandler::GridMapHandler(const std::string &fname, double res, double local_map_length, double inflation_radius,
                     bool use_polygon, tf::TransformListener &tf, grid_map::GridMap &global_grid_map,
                     grid_map::GridMap &local_gridmap)
      : global_map_(global_grid_map), local_map_(local_gridmap), tf_(tf), first_pos_call_(true), first_laser_call_(true), inflation_radius_(inflation_radius), use_polygon_(use_polygon) {

  ros::Time::waitForValid();
  double begin = ros::Time::now().toSec();
  lastCall_ = ros::Time::now().toSec();

  tf::TransformBroadcaster tf_bc_;
  std::string frame_id;
  int negate;
  double occ_th, free_th, pic_res;
  double alpha = 0.5;
  double origin[3];
  std::string mapfname = "";

  ros::NodeHandle nh;
  static_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  static_inf_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("static_inflated_map", 1, true);
  inflated_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1, true);
  local_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);
  local_map_pub_infl_ = nh.advertise<nav_msgs::OccupancyGrid>("local_map_inflated", 1, true);

  ros::NodeHandle private_nh("~");
  private_nh.param("frame_id", frame_id, std::string("map"));

  std::ifstream fin(fname.c_str());
  if (fin.fail()) {
    ROS_ERROR("grid_map_handler could not open %s.", fname.c_str());
    exit(-1);
  }

  #ifdef HAVE_YAMLCPP_GT_0_5_0
    // The document loading process changed in yaml-cpp 0.5.
          YAML::Node doc = YAML::Load(fin);
  #else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
  #endif

  try {
    doc["resolution"] >> pic_res;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
    exit(-1);
  }
  try {
    doc["occupied_thresh"] >> occ_th;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
    exit(-1);
  }
  try {
    doc["free_thresh"] >> free_th;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
    exit(-1);
  }
  try {
    doc["origin"][0] >> origin[0];
    doc["origin"][1] >> origin[1];
    doc["origin"][2] >> origin[2];
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The map does not contain an origin tag or it is invalid.");
    exit(-1);
  }
  try {
    doc["image"] >> mapfname;
    // TODO: make this path-handling more robust
    if(mapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      exit(-1);
    }
    if(mapfname[0] != '/')
    {
      // dirname can modify what you pass it
      char* fname_copy = strdup(fname.c_str());
      mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
      free(fname_copy);
    }
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    exit(-1);
  }
    ROS_INFO_STREAM("Origin resolution: " << pic_res << " desired resolution: " << res);

    //load image and initialize global grid map
  loadImage(mapfname, pic_res, res);

  // override grey fields with NO_INFORMATION
  for (grid_map::GridMapIterator iterator(global_map_); !iterator.isPastEnd(); ++iterator) {
    if (global_map_.at(STATIC, *iterator) >= 19.0 and global_map_.at(STATIC, *iterator) <= 20.0) //TODO define gray value as no info
      global_map_.at(STATIC, *iterator) = NO_INFORMATION;
    if (global_map_.at(STATIC_INF, *iterator) >= 19.0 and global_map_.at(STATIC_INF, *iterator) <= 20.0) //TODO define gray value as no info
      global_map_.at(STATIC_INF, *iterator) = NO_INFORMATION;
  }

  //initialize layers
  local_map_.setFrameId("base_rotation");
  local_map_.setGeometry(grid_map::Length(local_map_length, local_map_length), res);
  local_map_.add(DYNAMIC, NO_INFORMATION);
  local_map_.add(INFLATION, NO_INFORMATION);
  global_map_.add(MERGE);

  //convert & publish
  nav_msgs::OccupancyGrid static_map;
  nav_msgs::OccupancyGrid static_inflated_map;
  grid_map::GridMapRosConverter::toOccupancyGrid(global_map_, STATIC, NO_INFORMATION, LETHAL_OBJECT, static_map);
  grid_map::GridMapRosConverter::toOccupancyGrid(global_map_, STATIC_INF, NO_INFORMATION, LETHAL_OBJECT, static_inflated_map);
  static_map_pub_.publish(static_map);
  static_inf_map_pub_.publish(static_inflated_map);

  ROS_INFO_STREAM("\n Published static map");

  ROS_INFO("Subscribe to laser scan");
  sub_scan_ = nh.subscribe("scan", 1, &GridMapHandler::laserScanCallback, this); //TODO eventually clean up this space
//  sub_rot_ = nh.subscribe("amcl_pose", 10, &GridMapHandler::tfRotationBroadcast, this); // tf seems to have more recent updates
//  sub_pos_ = nh.subscribe("amcl_pose", 1, &GridMapHandler::positionCallback, this); //not really needed
    start();
}

GridMapHandler::~GridMapHandler() {
  stop();
}

void GridMapHandler::loadImage(std::string mapfname, double pic_res, double res)
{
  //load, crop and inflate image
  cv::Mat cropped;
  cv::Mat cropped_inflated;
  cropImage(mapfname, cropped);
  kernel_ = cv::getStructuringElement(cv::MORPH_ERODE, cv::Size(inflation_radius_ / pic_res, inflation_radius_ / pic_res));
  cv::erode(cropped, cropped_inflated, kernel_);
  kernel_ = cv::getStructuringElement(cv::MORPH_ERODE, cv::Size(inflation_radius_ / res, inflation_radius_ / res)); //used for laserscanCallbacks

  //load image into gridmap
  grid_map::GridMap temp_map;
  grid_map::GridMapCvConverter::initializeFromImage(cropped, pic_res, temp_map, grid_map::Position(0,0));
  temp_map.setFrameId("map");
  const int cvEncoding = CV_8UC3; //TODO check pics encoding
  switch (cvEncoding) {
    case CV_8UC1:
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(cropped, STATIC, temp_map, LETHAL_OBJECT, FREE);
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(cropped_inflated, STATIC_INF, temp_map, LETHAL_OBJECT, FREE);
      break;
    case CV_8UC3:
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(cropped, STATIC, temp_map, LETHAL_OBJECT, FREE);
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(cropped_inflated, STATIC_INF, temp_map, LETHAL_OBJECT, FREE);
      break;
    case CV_8UC4:
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 4>(cropped, STATIC, temp_map, LETHAL_OBJECT, FREE);
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 4>(cropped_inflated, STATIC_INF, temp_map, LETHAL_OBJECT, FREE);
      break;
    case CV_16UC1:
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(cropped, STATIC, temp_map, LETHAL_OBJECT, FREE);
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(cropped_inflated, STATIC_INF, temp_map, LETHAL_OBJECT, FREE);
      break;
    case CV_16UC3:
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 3>(cropped, STATIC, temp_map, LETHAL_OBJECT, FREE);
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 3>(cropped_inflated, STATIC_INF, temp_map, LETHAL_OBJECT, FREE);
      break;
    case CV_16UC4:
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 4>(cropped, STATIC, temp_map, LETHAL_OBJECT, FREE);
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 4>(cropped_inflated, STATIC_INF, temp_map, LETHAL_OBJECT, FREE);
      break;
    default:
      ROS_ERROR("Expected MONO8, MONO16, RGB(A)8, RGB(A)16, BGR(A)8, or BGR(A)16 image encoding.");
      break;
  }
  grid_map::GridMapCvProcessing::changeResolution(temp_map, global_map_, res, cv::INTER_NEAREST);
}

void GridMapHandler::cropImage(const std::string& mapfname, cv::Mat& cropped)
{
  // pgm to cv
  cv::Mat image = cv::imread(mapfname, CV_LOAD_IMAGE_COLOR);

  cv::Mat image_gray;
  cv::cvtColor(image,image_gray,cv::COLOR_BGR2GRAY);

  std::cout << "Color image: " << (int)image.at<unsigned char>(0,0) << std::endl;
  std::cout << "Gray Image: " << (int)image_gray.at<unsigned char>(0,0) << std::endl;

  cv::Mat thresholds;
  cv::Mat locations;   // indices
  cv::threshold(image_gray, thresholds, 203, 255, cv::THRESH_BINARY_INV);

  cv::findNonZero(thresholds, locations);
  cv::Rect bounding_box = cv::boundingRect(locations);
  cropped = image(bounding_box);
}

bool GridMapHandler::start() {
  broadcast_ = true;
  move_ = true;
  merge_ = true;
  rotation_broadcast_thread_ptr_ = std::make_shared<std::thread>(&GridMapHandler::tfRotationBroadcast, this);
  move_thread_ptr_ = std::make_shared<std::thread>(&GridMapHandler::move, this);
  merge_thread_ptr_ = std::make_shared<std::thread>(&GridMapHandler::mergeInflationLayers, this);
}

bool GridMapHandler::stop() {
  move_ = false;
  broadcast_ = false;
  merge_ = false;
  rotation_broadcast_thread_ptr_->join();
  move_thread_ptr_->join();
  merge_thread_ptr_->join();
}

void GridMapHandler::mergeInflationLayers() { //TODO only rewrite the affected regions (faster?)
  cv::Mat res;
  cv::Mat local_pic;
  cv::Mat cropped;
  cv_bridge::CvImage in_image, global_image;
  tf::Stamped<tf::Point> origin(tf::Point(0, 0, 0), ros::Time(0), "base_footprint");
  tf::Stamped<tf::Point> robot_pos;
  grid_map::Index robot_index;
  int x, y, x_shift, y_shift, x_crop, y_crop;
  std::chrono::milliseconds sleep_duration(980);

  while(merge_ && ros::ok()) {
    //merge local_inflated and global_inflated into merge

    local_map_lock_.lock();
    grid_map::GridMapRosConverter::toCvImage(local_map_, INFLATION, "8UC3", in_image);
    local_map_lock_.unlock();
    grid_map::GridMapRosConverter::toCvImage(global_map_, STATIC_INF, "8UC3", global_image);

    cv::resize(global_image.image, local_pic, cv::Size(global_image.image.cols, global_image.image.rows), 0, 0,
               cv::INTER_NEAREST);

    //get robot position
    origin.stamp_ = ros::Time(0);
    tf_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));
    try {
      tf_.transformPoint("map", ros::Time(0), origin, "base_footprint", robot_pos);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame map when merging inflation layers: %s", ex.what());
    }

//TODO unreadable code starts
    global_map_.getIndex(grid_map::Position(robot_pos.x(), robot_pos.y()),
                         robot_index);
    x = robot_index.x() - in_image.image.rows / 2; // x, y: coordinates in "map" of the top left roi corner
    y = robot_index.y() - in_image.image.rows / 2;
    x_shift = 0;
    y_shift = 0;
    x_crop = 0;
    y_crop = 0;

    //crop regions exceeding the global borders
    if (x > local_pic.rows - in_image.image.rows) {         //  if (ROI's upper left point near enough to the global maps border so that local map exceeds global maps borders in positive y direction) {
      x_crop = in_image.image.rows - (local_pic.rows - x);  //    crop by the amount exceeded in y direction
    }
    if (y > local_pic.cols - in_image.image.rows) {         //  if (ROI's upper left point near enough to the global maps border so that local map exceeds global maps borders in positive x direction) {
      y_crop = in_image.image.rows - (local_pic.cols - y);  //    crop by the amount exceeded in x direction
    }
    if (x < 0) {                                            //  if (ROI's upper left point is in the negative and thus exceeds the global maps borders in x direction) {
      x_shift = -x;                                         //    shift by the absolute of ROI, i.e. the local map, exceeding the global map in negative x direction
      x_crop = x_crop + x_shift;                            //    crop by the same amount + the crop amount if local map is oversized and exceeds in both directions
      x = 0;                                                //    place ROI's upper left point onto the global maps border
    }
    if (y < 0) {                                            //  if (ROI's upper left point is in the negative and thus exceeds the global maps borders in y direction) {
      y_shift = -y;                                         //    shift by the absolute of ROI, i.e. the local map, exceeding the global map in negative y direction
      y_crop = y_crop + y_shift;                            //    crop by the same amount + the crop amount if local map is oversized and exceeds in both directions
      y = 0;                                                //    place ROI's upper left point onto the global maps border
    }
    cropped = in_image.image(cv::Rect(y_shift, x_shift, in_image.image.rows - y_crop, in_image.image.rows - x_crop));

//copy to large image
    cv::Rect roi(y, x, cropped.cols, cropped.rows);
    cropped.copyTo(local_pic(roi));{}

    cv::bitwise_or(global_image.image, local_pic, res); // merge

    local_map_lock_.lock();
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(res, MERGE, global_map_, FREE, LETHAL_OBJECT);
    local_map_lock_.unlock();
    publishGlobal();
    std::this_thread::sleep_for(sleep_duration);
  }
}


void GridMapHandler::updateLocalInflation() {
  //inflate local
  cv_bridge::CvImage in_image;
  cv::Mat out_image;

  local_map_lock_.lock();
  grid_map::GridMapRosConverter::toCvImage(local_map_, DYNAMIC, "8UC3", in_image);
  local_map_lock_.unlock();

  cv::dilate(in_image.image, out_image, kernel_);

  local_map_lock_.lock();
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(out_image, INFLATION, local_map_, FREE, LETHAL_OBJECT);
  local_map_lock_.unlock();
}

void GridMapHandler::publishLocal() {
  nav_msgs::OccupancyGrid dynamic_layer;
  nav_msgs::OccupancyGrid inflation_layer;
  local_map_lock_.lock();
  grid_map::GridMapRosConverter::toOccupancyGrid(local_map_, DYNAMIC, FREE, LETHAL_OBJECT, dynamic_layer);
  grid_map::GridMapRosConverter::toOccupancyGrid(local_map_, INFLATION, NO_INFORMATION, LETHAL_OBJECT, inflation_layer);
  local_map_lock_.unlock();
  local_map_pub_.publish(dynamic_layer);
  local_map_pub_infl_.publish(inflation_layer);
}

void GridMapHandler::publishGlobal() {
  nav_msgs::OccupancyGrid inflated_map;
  grid_map::GridMapRosConverter::toOccupancyGrid(global_map_, MERGE, NO_INFORMATION, LETHAL_OBJECT, inflated_map);
  inflated_map_pub_.publish(inflated_map);
}

void GridMapHandler::tfRotationBroadcast() {
  while(broadcast_ && ros::ok()){
    tf::StampedTransform transform;
    try {
      tf_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame base_footprint when broadcasting the rotation frame of the local map: %s", ex.what());
    }
    tf::StampedTransform send_tf;
    send_tf.setIdentity();
    send_tf.setRotation(transform.getRotation().inverse());
    send_tf.frame_id_ = "base_footprint";
    send_tf.child_frame_id_ = "base_rotation";
    send_tf.stamp_ = transform.stamp_;
    tf_bc_.sendTransform(send_tf);
  }
}

void GridMapHandler::laserScanCallback(const sensor_msgs::LaserScanConstPtr& laserMessage)
{
//    ros::Time begin = ros::Time::now();

    // project the laser into a point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header = laserMessage->header;

    // transform
    try {
      tf_.waitForTransform(local_map_.getFrameId(), laserMessage->header.frame_id, laserMessage->header.stamp,
                           ros::Duration(1.0));
        projector_.transformLaserScanToPointCloud(local_map_.getFrameId(), *laserMessage, cloud, tf_);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s when transforming the laser scan: %s",
               local_map_.getFrameId().c_str(),
               ex.what());
      projector_.projectLaser(*laserMessage, cloud);
    }

    //condense data
    sensor_msgs::PointCloud2Iterator<float> x_iter(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iter(cloud, "y");
    std::vector<grid_map::Index> set;

    for (; x_iter != x_iter.end(); ++x_iter, ++y_iter) {
      grid_map::Position obstaclePosition(*x_iter, *y_iter);
      grid_map::Index index;
      index.operator<(index);
      if (local_map_.getIndex(obstaclePosition, index)) { //true if Position in Gridmap
        set.push_back(index);
      }
    }
    std::vector<grid_map::Index>::iterator end = std::unique(set.begin(), set.end(),
                                                             [](grid_map::Index a, grid_map::Index b) {
                                                               return a[0] == b[0] && a[1] == b[1];
                                                             });
    set.erase(end, set.end());
    local_map_lock_.lock();

    // refresh
    local_map_.add(DYNAMIC, NO_INFORMATION);

    //write data
    grid_map::Position last_obstacle;
    grid_map::Position very_first_obstacle;
    bool first_obstacle = true;
    for (auto index : set) {
      grid_map::Position pos;
      local_map_.getPosition(index, pos);

      if (!use_polygon_) //walk over line iterator filling dynamic layer
        for (grid_map::LineIterator lIt(local_map_, grid_map::Position(0, 0), pos); !lIt.isPastEnd(); ++lIt) {
          local_map_.at(DYNAMIC, *lIt) = FREE;
        }
      else { //walk over polygon iterator filling dynamic layer
        if(first_obstacle)//should not create a polygon from 2 points
        {
          first_obstacle = false;
          very_first_obstacle = pos;
        }
        else {
          grid_map::Polygon polygon(std::vector<grid_map::Position> { last_obstacle, grid_map::Position(0, 0), pos });
          for (grid_map::PolygonIterator pIt(local_map_, polygon); !pIt.isPastEnd(); ++pIt) {
            local_map_.at(DYNAMIC, *pIt) = FREE;
          }
        }
        last_obstacle = pos;
      }

      local_map_.at(DYNAMIC, index) = LETHAL_OBJECT;
    }

    if(use_polygon_){ //close the circle of polygons
      grid_map::Polygon polygon(std::vector<grid_map::Position> { last_obstacle, grid_map::Position(0, 0), very_first_obstacle });
      for (grid_map::PolygonIterator pIt(local_map_, polygon); !pIt.isPastEnd(); ++pIt) {
        local_map_.at(DYNAMIC, *pIt) = FREE;
      }
    }

    local_map_lock_.unlock();

//    std::cout << ros::Time::now().toSec() - begin.toSec() << " seconds laserscancallback." << std::endl << std::endl;

//    ros::Time begin = ros::Time::now();
    updateLocalInflation();

    publishLocal();
//  std::cout << ros::Time::now().toSec() - begin.toSec() << " seconds laserscancallback." << std::endl << std::endl;
}

void GridMapHandler::move(){
  std::chrono::milliseconds sleep_duration(100);
  while(move_ && ros::ok()){
   ros::Time now = ros::Time(0);
   if(first_pos_call_){
     first_pos_call_ = false;
     last_moved_ = now;
   }else {
     tf::StampedTransform tf_moved;
     tf_.waitForTransform(local_map_.getFrameId(), last_moved_, local_map_.getFrameId(), now, "map", ros::Duration(1.0));
     try {
       tf_.lookupTransform(local_map_.getFrameId(), last_moved_, local_map_.getFrameId(), now, "map", tf_moved);
     }
     catch (tf::TransformException &ex) {
       ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s when moving the grid map: %s",
                local_map_.getFrameId().c_str(),
                ex.what());
     }
     tf::Vector3 origin = tf_moved.getOrigin();
     grid_map::Position moved(origin.x(), origin.y());
     local_map_lock_.lock();
     if (local_map_.move(moved)) {
       last_moved_ = now;
     }
     local_map_lock_.unlock();
     publishLocal();
     std::this_thread::sleep_for(sleep_duration);
   }
 }
}

} /* namespace grid_map_handler */

int main(int argc, char** argv){

  ros::init(argc, argv, "grid_map_handler_node");
  tf::TransformListener tf;
  grid_map::GridMap grid_map_1;
  grid_map::GridMap grid_map_2;
  grid_map_handler::GridMapHandler grid_map_handler_(argv[1], atof(argv[2]), 0, 0, false, tf, grid_map_1, grid_map_2);

  ros::NodeHandle n;
  ros::Subscriber sub0 = n.subscribe("scan", 1, &grid_map_handler::GridMapHandler::laserScanCallback, &grid_map_handler_);
//  ros::Subscriber sub1 = n.subscribe("amcl_pose", 1, &grid_map_handler::GridMapHandler::positionCallback, &grid_map_handler_);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return EXIT_SUCCESS;
}