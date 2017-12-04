//
// Created by thomas on 19.11.17.
// base on ROS gmapping node https://github.com/ros-perception/slam_gmapping
//

#ifndef SEMANTIC_MAPPING_V2_SLAMGMAPPING_H
#define SEMANTIC_MAPPING_V2_SLAMGMAPPING_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#include <boost/thread.hpp>

class MyGridSlamProcessor : public GMapping::GridSlamProcessor{
  protected:

  public:
    MyGridSlamProcessor() : GMapping::GridSlamProcessor(){
      m_indexes = std::vector<unsigned int>(m_particles.size());
      std::iota(m_indexes.begin(), m_indexes.end(), 0);
    }
    MyGridSlamProcessor(std::ostream& infoStr) : GridSlamProcessor(infoStr) {
      m_indexes = std::vector<unsigned int>(m_particles.size());
      std::iota(m_indexes.begin(), m_indexes.end(), 0);
    }

    bool resume_ = false;
    int resume_particle_ = 0;
    GMapping::OrientedPoint new_odom_, new_pose_;

    inline void resetIndexes() { m_indexes = std::vector<unsigned int>(); }
    void init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, GMapping::OrientedPoint initialPose, const GMapping::ScanMatcherMap& initial_map);
    void resume(const GMapping::OrientedPoint& new_pose, const GMapping::OrientedPoint& new_odom);

    virtual void onOdometryUpdate();
    virtual void onResampleUpdate();
    virtual void onScanmatchUpdate();
};


class SlamGMapping
{
  public:
    SlamGMapping(tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom);
    SlamGMapping(tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom, ros::NodeHandle& nh, ros::NodeHandle& pnh);
    SlamGMapping(tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom, unsigned long int seed, unsigned long int max_duration_buffer);
    virtual ~SlamGMapping();

    void init();

    tf::StampedTransform getTransform();
    nav_msgs::OccupancyGrid getGMap();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool isInitialized() const { return got_first_scan_; }

    void resume(const GMapping::OrientedPoint& new_pose);

  protected:
    ros::NodeHandle node_;
    tf::TransformListener* tf_;

    MyGridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    // The angles in the laser, going from -x to x (adjustment is made to get the laser between
    // symmetrical bounds as that's what gmapping expects)
    std::vector<double> laser_angles_;
    // The pose, in the original laser frame, of the corresponding centered laser with z facing up
    tf::Stamped<tf::Pose> centered_laser_pose_;
    // Depending on the order of the elements in the scan and the orientation of the scan frame,
    // We might need to change the order of the scan
    bool do_reverse_range_;
    unsigned int gsp_laser_beam_count_;
    GMapping::OdometrySensor* gsp_odom_;

    bool got_first_scan_;
    bool processed_scan_ = false;
    ros::Time activate_time_ = ros::TIME_MAX;
    double settle_time_ = 1.0;
    GMapping::OrientedPoint initial_pose_;

    bool got_map_;
    nav_msgs::GetMap::Response map_;

    ros::Duration map_update_interval_;
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(const sensor_msgs::LaserScan& scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

    ros::NodeHandle private_nh_;

    unsigned long int seed_;

    double transform_publish_period_;
    double tf_delay_;
};


#endif //SEMANTIC_MAPPING_V2_SLAMGMAPPING_H
