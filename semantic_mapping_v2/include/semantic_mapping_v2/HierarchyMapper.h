//
// Created by thomas on 19.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H
#define SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H

#include "semantic_mapping_v2/RoomMapper.h"

#include <semantic_mapping_v2/MapSrv.h>
#include <semantic_mapping_v2/OctomapSrv.h>
#include <semantic_mapping_v2/HierarchySrv.h>
#include <semantic_mapping_v2/ObjectMapSrv.h>
#include <semantic_mapping_v2/ObjectProbSrv.h>
#include <semantic_mapping_v2/RoomTypeMapSrv.h>
#include <semantic_mapping_v2/RoomTypeProbSrv.h>
#include <semantic_mapping_v2/ObjFoundSrv.h>

#include <std_msgs/Int8.h>

class HierarchyMapper{
  protected:
    std::vector<RoomMapper*> room_mapper_;
    boost::mutex mapper_mutex_;
    int current_mapper_ = -1;
    ros::Time last_map_switch_time_;
    int curr_action_ = -1;

    ros::Subscriber laser_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber door_pose_sub_;
    ros::Subscriber vision_sub_;
    ros::Subscriber curr_action_sub_;

    ros::Publisher map_pub_;
    ros::Publisher gmap_pub_;
    ros::Publisher map_door_blocked_pub_;
    ros::Publisher map_switch_pub_;
    ros::Publisher map_info_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher door_pose_pub_;
    ros::Publisher door_found_pub_;
    std::vector<ros::Publisher> obj_prob_pub_;
    std::vector<ros::Publisher> room_prob_pub_;
    ros::Publisher particle_pose_pub_;
    std::vector<ros::Publisher> base_obj_prob_pub_;
    std::vector<ros::Publisher> base_room_prob_pub_;

    ros::CallbackQueue service_queue_;
    ros::AsyncSpinner service_spinner_;
    ros::ServiceServer gmap_srv_;
    ros::ServiceServer map_srv_;
    ros::ServiceServer map_door_blocked_srv_;
    ros::ServiceServer octomap_srv_;
    ros::ServiceServer obj_map_srv_;
    ros::ServiceServer room_type_map_srv_;
    ros::ServiceServer obj_prob_srv_;
    ros::ServiceServer room_type_prob_srv_;
    ros::ServiceServer hierarchy_srv_;
    ros::ServiceServer obj_found_srv_;
    std::vector<semantic_mapping_v2::RoomMsg> last_room_msgs_;
    std::vector<bool> room_changed_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster* tfB_;
    ros::NodeHandle nh_;
    boost::thread* transform_thread_;

    void transformPublishLoop(double transform_publish_period);

    double transform_publish_period_;
    double publish_period_;
    int debug_publish_interval_ = 10;//std::numeric_limits<int>::max();
    double MIN_MAP_SWITCH_TIME = 2.0;

    float ROOM_CELL_OBJ_KERNEL_SIZE = 4.f;
    float OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE = 2.f;
    float OBJ_FILL_FRACTION = 1.f/16.f;
    float ROOM_ESTIMATED_VOLUME = 32.f;
    float CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA = 2.f;
    float SINGLE_VIEW_OBJ_KERNEL_SIZE = 2.f;
    float TRAVEL_DIST_LIN_FACTOR = 4.f;
    float TRAVEL_DIST_QUAD_FACTOR = 0.5f;
    float TRAVEL_TURN_FACTOR = 10.f;
    float SEARCH_TIME_PER_GRID_CELL = 0.05f;

  public:
    HierarchyMapper();
    ~HierarchyMapper();

    void addMapper(const Door& door = Door());
    void switchMapper(int mapper_idx, const Door& door = Door());

    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void doorPoseCb(const geometry_msgs::PoseArray::ConstPtr& msg);
    void visionCb(const vision::VisionMsgConstPtr& msg);
    void currActionCb(const std_msgs::Int8ConstPtr& msg);

    bool gmapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res);
    bool mapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res);
    bool mapDoorBlockedSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res);
    bool octomapSrvCb(semantic_mapping_v2::OctomapSrv::Request& req, semantic_mapping_v2::OctomapSrv::Response& res);
    bool roomTypeMapSrvCb(semantic_mapping_v2::RoomTypeMapSrv::Request& req, semantic_mapping_v2::RoomTypeMapSrv::Response& res);
    bool roomTypeProbSrvCb(semantic_mapping_v2::RoomTypeProbSrv::Request& req, semantic_mapping_v2::RoomTypeProbSrv::Response& res);
    bool objMapSrvCb(semantic_mapping_v2::ObjectMapSrv::Request& req, semantic_mapping_v2::ObjectMapSrv::Response& res);
    bool objProbSrvCb(semantic_mapping_v2::ObjectProbSrv::Request& req, semantic_mapping_v2::ObjectProbSrv::Response& res);
    bool hierarchySrvCb(semantic_mapping_v2::HierarchySrv::Request& req, semantic_mapping_v2::HierarchySrv::Response& res);
    bool objFoundSrvCb(semantic_mapping_v2::ObjFoundSrv::Request& req, semantic_mapping_v2::ObjFoundSrv::Response& res);

    //void publish();
    void downprojecAndPublish();
    void publishRoomTypeProbMap(const RoomTypeMap& map, int idx);
    void publishObjProbMap(const ObjectMap& map, int idx);

    void run();


    cv::Mat_<float> getBehindDoorMask(const std::vector<Door>& doors, int width, int height);
    std::vector<cv::Mat_<float>> get2dAreaObjProbMaps(const std::vector<ObjectMap>& obj_maps, const cv::Mat_<float> behind_door_mask, const ObjectMap& occ_map,
                                                      const cv::Point& room_origin, int room_width, int room_height, float kernel_size);
    std::vector<cv::Mat_<float>> getObjBasedRoomTypeMap(const std::vector<cv::Mat_<float>>& obj_prob_2d_area, int num_room_types);
    std::vector<cv::Mat_<float>> getCompleteRoomTypeMap(const std::vector<RoomTypeMap>& room_type_map, const std::vector<cv::Mat_<float>>& obj_based_room_type_map);
    std::vector<float> getRoomTypeProbs(const std::vector<cv::Mat_<float>>& complete_room_type_map, const cv::Mat_<uchar>& seen_map, const cv::Point& origin,
                                        const nav_msgs::OccupancyGrid& grid_map, const std::vector<Door>& doors, float resolution, int base_size);
    std::vector<ObjectMap> getCompleteObjMap(const std::vector<cv::Mat_<float>>& complete_room_type_map, const std::vector<ObjectMap>& obj_map,
                                             const ObjectMap& occ_map, const cv::Point& new_orig);
    std::vector<float> getCompleteObjProbs(const std::vector<ObjectMap>& complete_obj_map, std::vector<float> room_type_probs, const ObjectMap& occ_map);
    float getTravelTime(const geometry_msgs::Pose& door1, const geometry_msgs::Pose& door2);
    std::vector<float> getToLinkTravelTime(int room, const std::vector<Door>& doors, const nav_msgs::OccupancyGrid& grid_map);
    float getSearchTime(const nav_msgs::OccupancyGrid& grid_map);
};

#endif //SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H
