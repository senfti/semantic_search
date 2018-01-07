//
// Created by thomas on 02.01.18.
//

#ifndef EXECUTION_SEARCHER_H
#define EXECUTION_SEARCHER_H

#include <execution/ObjectMapper.h>
#include <execution/OctoMapper.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int8.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vision/VisionMsg.h>

template <class T>
class Map{
  public:
    static constexpr int BASE_SIZE = 100;

    float resolution_ = 0.05;
    double default_value_ = 0.0;
    cv::Point origin_;
    cv::Mat_<T> map_;

    Map(float resolution, double initial_value = 0.0)
      : resolution_(resolution), default_value_(initial_value), origin_(BASE_SIZE/2, BASE_SIZE/2), map_(BASE_SIZE, BASE_SIZE, T(initial_value))
    { }
    Map(const cv::Mat_<T>& map, float resolution, const cv::Point& origin, double default_value = 0.0);
    Map(const Map& rhs);

    void resize(int x1, int x2, int y1, int y2);
    void resample(float new_resolution);
    void makeNiceSize();
    void makeSame(Map& other_map, float resolution);

    Map operator*(const Map& rhs);
    Map operator*(const cv::Mat_<T>& rhs);

    T& operator()(const cv::Point& p) { return map_(p); }
    T& operator()(int y, int x) { return map_(y,x); }
};


class Searcher{
  public:
    const float SEARCH_MAX_ROT_VEL = 1.0;
    const float SEARCH_MAX_TRANS_VEL = 0.3;
    const int VIEW_DIST = 1.0/0.05;
    const int ROBOT_SIZE = 0.3/0.05;

    float OBJ_PRIOR_PROB = 0.0005f;
    float OBJ_MIN_PROB = 0.00001f;
    float OBJ_MAX_PROB = 0.7f;

    float RESOLUTION = 10.f;
    float OBJ_DEFUALT_MAX_HEIGHT = 1.6f;
    float ROOM_EXPECTED_SIZE = 32.f;

    float V_H = 0.3;
    float V_M = 0.0002;

    float POINTCLOUD_MIN_Z = 0.0f;
    float POINTCLOUD_MAX_Z = 1.8f;

    float OBJECT_FOUND_THRESH = 0.7;

    int VIEW_ANGLE_STEPS = 12;
    int SAMPLE_COUNT_THRESH = 10000;

    float VIEW_MIN_DIST = 0.8f;
    float VIEW_MAX_DIST = 3.5f;
    float VIEW_ANGLE = 50.f;

    float TURN_SPEED = 0.5;
    float MOVE_SPEED = 0.1;
    float VIEW_TIME = 0.2;

    int SEEN_MAP_STEPS = 24;
    int BORDER_SEEN_THRESH = 3;
    int SEEN_MAP_MAX_DIST = 2.5;

  private:
    ros::Subscriber map_sub_;
    ros::Subscriber vision_sub_;
    tf::TransformListener* tf_listener_;

    ros::Publisher octomap_pub_;

    int searched_obj_ = 0;
    ObjectMap* obj_map_ = nullptr;
    ObjectMap* prior_prob_map_ = nullptr;
    OctoMapper* octo_mapper_ = nullptr;
    std::vector<cv::Mat_<float>> seen_maps_;
    cv::Mat_<float> accessible_map_;
    cv::Mat_<uchar> border_map_;
    cv::Mat_<float> border_dir_map_;

    ros::Publisher map_pub_;

    geometry_msgs::Pose curr_view_pose_;
    bool curr_view_changed_;
    bool finished_ = false;

    cv::Point getNearestFree(const cv::Mat_<uchar>& valid_cells, int x, int y) const;
    cv::Mat_<float> getProbMap(cv::Point& origin);
    cv::Mat_<float> getViewKernel(float angle, float max_dist, float resolution) const;
    cv::Mat_<float> calcMoveTime(int width, int height, int angle_step, const cv::Point& curr_pos, float curr_angle);
    bool insertIntoSeenMaps(const tf::Transform& curr_pose);

    void resize(float x1, float x2, float y1, float y2);

  public:
    Searcher(int searched_obj, int curr_room, tf::TransformListener* tf_listener);
    ~Searcher();

    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
    void visionCb(const vision::VisionMsgConstPtr& msg);
    void insertObject(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg);
    void insertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Point& sensorOriginTf);

    bool objFound();

    void calcNextViewpoint(const tf::Transform& curr_pose);
};


#endif //EXECUTION_SEARCHER_H
