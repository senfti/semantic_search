//
// Created by thomas on 02.01.18.
//

#ifndef EXECUTION_SEARCHER_H
#define EXECUTION_SEARCHER_H

#include <execution_test/ObjectMapper.h>
#include <execution_test/OctoMapper.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int8.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Float64MultiArray.h>

#include <vision/VisionMsg.h>

class Searcher{
  public:
    float SEARCH_MAX_ROT_VEL = 1.0;
    float SEARCH_MAX_TRANS_VEL = 0.3;
    int ROBOT_SIZE = 0.35/0.05;

    float OBJ_PRIOR_PROB = 0.003f;
    float OBJ_MIN_PROB = 0.00001f;
    float OBJ_MAX_PROB = 0.9f;

    float RESOLUTION = 10.f;
    float OBJ_DEFUALT_MAX_HEIGHT = 1.6f;
    float ROOM_EXPECTED_SIZE = 24.f;

    float V_H = 0.08;
    float V_M = 0.0003;

    float POINTCLOUD_MIN_Z = 0.0f;
    float POINTCLOUD_MAX_Z = 1.8f;

    float OBJECT_FOUND_THRESH = 0.7;

    int VIEW_ANGLE_STEPS = 12;
    int SEEN_MAP_STEPS = 36;
    int SAMPLE_COUNT_THRESH = 5;

    float VIEW_MIN_DIST = 0.55f;
    float VIEW_MAX_DIST = 3.5f;
    float VIEW_ANGLE = 50.f*M_PI/180.0;

    float TURN_SPEED = 0.5;
    float MOVE_SPEED = 0.01;
    float VIEW_TIME = 0.2;

    int BORDER_SEEN_THRESH = 1;
    float BORDER_SEEN_MAX_ANGLE = 25.f*M_PI/180.0;
    int SEEN_MAP_MAX_DIST = 2.5;

    float INTERESTING_BORDER_SEEN_REWARD = 0.01f;

    float QUICK_SEARCH_MIN_DIST = 0.8;
    float QUICK_SEARCH_MAX_DIST = 2.0;
    float QUICK_SEARCH_ANGLE_AREA = M_PI/45.f;
    int QUICK_SEARCH_VIEWS = 5;

  private:
    ros::Subscriber map_sub_;
    ros::Subscriber vision_sub_;
    ros::Subscriber search_query_sub_;
    tf::TransformListener* tf_listener_;

    ros::Publisher octomap_pub_;
    std::vector<ros::Publisher> obj_pub_;
    std::vector<ros::Publisher> full_pub_;
    ros::Publisher count_pub_;
    ros::Publisher next_pose_pub_;
    ros::Publisher obj_found_pub_;
    ros::Publisher prior_pub_;

    ros::ServiceClient obj_map_service_client_;

    int searched_obj_ = 0;
    std::vector<ObjectMap*> obj_map_;
    std::vector<ObjectMap*> prior_prob_map_;
    OctoMapper* octo_mapper_ = nullptr;
    std::vector<cv::Mat_<uchar>> seen_maps_;
    std::vector<cv::Mat_<uchar>> previous_pose_maps_;
    cv::Mat_<uchar> accessible_map_;
    cv::Mat_<uchar> good_accessible_map_;
    cv::Mat_<uchar> border_map_;
    cv::Mat_<float> border_dir_map_;
    cv::Mat_<uchar> not_fully_viewed_border_;

    geometry_msgs::Pose curr_view_pose_;
    geometry_msgs::Pose old_view_pose_;
    bool curr_view_changed_ = true;
    bool have_curr_view_ = false;
    bool got_map_ = false;
    bool got_vision_ = false;
    bool finished_ = false;
    bool running_ = false;
    bool obj_found_ = false;
    bool is_quick_search_ = false;
    int quick_search_step_ = 0;
    bool quick_search_step_viewed_ = true;
    geometry_msgs::Pose quick_search_target_;
    geometry_msgs::PoseStamped found_pose_;

    std::vector<std::vector<cv::Point>> seen_kernel_points_;
    std::vector<std::vector<float>> seen_kernel_points_value_;

    cv::Point getNearestFree(const cv::Mat_<uchar>& valid_cells, int x, int y) const;
    cv::Mat_<float> getProbMap(cv::Point& origin);
    cv::Mat_<uchar> getViewKernel(float angle, float max_dist, float resolution) const;
    float calcMoveTime(const cv::Point& pos, float angle, const cv::Point& curr_pos, float curr_angle);
    bool insertIntoSeenMaps(const tf::Transform& curr_pose);
    float calcViewpointGain(const cv::Point& pos, int angle_step, const cv::Mat_<float>& prob_map, const cv::Point& curr_pos, float curr_angle);
    void calcSeenKernels();

    void resize(float x1, float x2, float y1, float y2);

  public:
    Searcher(tf::TransformListener* tf_listener);
    ~Searcher();

    geometry_msgs::Pose getNextViewPose();
    bool hasViewPoseChanged() const { return curr_view_changed_; }
    bool haveViewPose() const { return have_curr_view_; }
    bool finished() const { return finished_; }
    bool running() const { return running_; }
    bool objectFound() const { return obj_found_; }

    void start(int searched_obj, bool quick_search, geometry_msgs::Pose& target_pose);
    void stop();

    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
    void visionCb(const vision::VisionMsgConstPtr& msg);
    void insertObject(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg);
    void insertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Point& sensorOriginTf);

    bool objFound(int obj = -1);

    bool calcNextViewpoint(const tf::Transform& curr_pose);
    bool calcNextQuickSearchViewpoint(const geometry_msgs::Pose& target_pose);

    void doCalculations();

    bool did_abort_ = false;

    void searchQueryCb(const geometry_msgs::QuaternionConstPtr& msg);
};


#endif //EXECUTION_SEARCHER_H
