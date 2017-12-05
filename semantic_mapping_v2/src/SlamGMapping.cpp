//
// Created by thomas on 19.11.17.
// base on ROS gmapping node https://github.com/ros-perception/slam_gmapping
//

#include "semantic_mapping_v2/SlamGMapping.h"


// Modified init for resume
void MyGridSlamProcessor::init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta,
                               GMapping::OrientedPoint initialPose, const GMapping::ScanMatcherMap& initial_map)
{
  m_xmin=xmin;
  m_ymin=ymin;
  m_xmax=xmax;
  m_ymax=ymax;
  m_delta=delta;
  if (m_infoStream)
    m_infoStream
          << " -xmin "<< m_xmin
          << " -xmax "<< m_xmax
          << " -ymin "<< m_ymin
          << " -ymax "<< m_ymax
          << " -delta "<< m_delta
          << " -particles "<< size << std::endl;

  m_particles.clear();
  TNode* node=new TNode(initialPose, 0, 0, 0);
  for (unsigned int i=0; i<size; i++){
    m_particles.push_back(Particle(initial_map));
    m_particles.back().pose=initialPose;
    m_particles.back().previousPose=initialPose;
    m_particles.back().setWeight(0);
    m_particles.back().previousIndex=0;
    m_particles.back().node= node;
  }
  m_neff=(double)size;
  m_count=0;
  m_readingCount=0;
  m_linearDistance=m_angularDistance=0;
}


void MyGridSlamProcessor::onOdometryUpdate(){
  if(resume_){
    for(auto& p : m_particles){
      p.pose = new_pose_;
    }
    m_odoPose = new_odom_;
    m_lastPartPose = new_odom_;
  }
  resume_ = false;
}


void MyGridSlamProcessor::onResampleUpdate(){
  //ROS_ERROR("RES");
}


void MyGridSlamProcessor::onScanmatchUpdate(){
//  if(resume_){
//    ROS_ERROR("SM");
//    for(int i=0; i<m_particles.size(); i++){
//      if(i != resume_particle_){
//        m_particles[i].weight = m_particles[resume_particle_].weight - 100000.0;
//      }
//    }
//  }
}


void MyGridSlamProcessor::resume(const GMapping::OrientedPoint& new_pose, const GMapping::OrientedPoint& new_odom){
  resume_particle_ = getBestParticleIndex();
  new_pose_ = new_pose;
  new_odom_ = new_odom;
  m_linearDistance = m_linearThresholdDistance;
  m_angularDistance = m_angularThresholdDistance;
  resume_ = true;
}



// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping(tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom):
      laser_count_(0), private_nh_("~"), tf_(tf), initial_pose_(initial_pose), map_to_odom_(initial_map_to_odom)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom, ros::NodeHandle& nh, ros::NodeHandle& pnh):
      laser_count_(0),node_(nh), private_nh_(pnh), tf_(tf), initial_pose_(initial_pose), map_to_odom_(initial_map_to_odom)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom, long unsigned int seed, long unsigned int max_duration_buffer):
      laser_count_(0), private_nh_("~"), seed_(seed), tf_(tf), initial_pose_(initial_pose), map_to_odom_(initial_map_to_odom)
{
  init();
}


void SlamGMapping::init(){
  gsp_ = new MyGridSlamProcessor(std::cerr);
  ROS_ASSERT(gsp_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;

  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("gmapping/throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("gmapping/base_frame", base_frame_))
    base_frame_ = "base_footprint";
  if(!private_nh_.getParam("gmapping/map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("gmapping/odom_frame", odom_frame_))
    odom_frame_ = "odom";

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;
  if(!private_nh_.getParam("gmapping/map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);

  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if(!private_nh_.getParam("gmapping/minimumScore", minimum_score_))
    minimum_score_ = 0;
  if(!private_nh_.getParam("gmapping/sigma", sigma_))
    sigma_ = 0.05;
  if(!private_nh_.getParam("gmapping/kernelSize", kernelSize_))
    kernelSize_ = 1;
  if(!private_nh_.getParam("gmapping/lstep", lstep_))
    lstep_ = 0.05;
  if(!private_nh_.getParam("gmapping/astep", astep_))
    astep_ = 0.05;
  if(!private_nh_.getParam("gmapping/iterations", iterations_))
    iterations_ = 5;
  if(!private_nh_.getParam("gmapping/lsigma", lsigma_))
    lsigma_ = 0.01;
  if(!private_nh_.getParam("gmapping/ogain", ogain_))
    ogain_ = 3.0;
  if(!private_nh_.getParam("gmapping/lskip", lskip_))
    lskip_ = 0;
  if(!private_nh_.getParam("gmapping/srr", srr_))
    srr_ = 0.01;
  if(!private_nh_.getParam("gmapping/srt", srt_))
    srt_ = 0.02;
  if(!private_nh_.getParam("gmapping/str", str_))
    str_ = 0.01;
  if(!private_nh_.getParam("gmapping/stt", stt_))
    stt_ = 0.02;
  if(!private_nh_.getParam("gmapping/linearUpdate", linearUpdate_))
    linearUpdate_ = 0.5;
  if(!private_nh_.getParam("gmapping/angularUpdate", angularUpdate_))
    angularUpdate_ = 0.436;
  if(!private_nh_.getParam("gmapping/temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if(!private_nh_.getParam("gmapping/resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if(!private_nh_.getParam("gmapping/particles", particles_))
    particles_ = 10;
  if(!private_nh_.getParam("gmapping/xmin", xmin_))
    xmin_ = -1.0;
  if(!private_nh_.getParam("gmapping/ymin", ymin_))
    ymin_ = -1.0;
  if(!private_nh_.getParam("gmapping/xmax", xmax_))
    xmax_ = 1.0;
  if(!private_nh_.getParam("gmapping/ymax", ymax_))
    ymax_ = 1.0;
  if(!private_nh_.getParam("gmapping/delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("gmapping/occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if(!private_nh_.getParam("gmapping/llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if(!private_nh_.getParam("gmapping/llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if(!private_nh_.getParam("gmapping/lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if(!private_nh_.getParam("gmapping/lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;

  if(!private_nh_.getParam("gmapping/tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;

  private_nh_.param("gmapping/settle_time", settle_time_, settle_time_);
}

SlamGMapping::~SlamGMapping(){
  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
}

bool SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t){
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  tf::Stamped<tf::Transform> odom_pose;
  try {
    tf_->transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());
  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw);

  return true;
}

bool SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan){
  laser_frame_ = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;
  try {
    tf_->transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e) {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp, base_frame_);
  try {
    tf_->transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e) {
    ROS_WARN("Unable to determine orientation of laser: %s", e.what());
    return false;
  }

  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001) {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.z());
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();
  double angle_center = (scan.angle_min + scan.angle_max)/2;
  if (up.z() > 0) {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center), tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center), tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i) {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max, scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(), laser_angles_.back(), std::fabs(scan.angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("gmapping/maxRange", maxRange_))
    maxRange_ = 4.5;//scan.range_max - 0.01;
  if(!private_nh_.getParam("gmapping/maxUrange", maxUrange_))
    maxUrange_ = 3.5;//maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_, fabs(scan.angle_increment), gmap_pose, 0.0, maxRange_);
  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_, kernelSize_, lstep_, astep_, iterations_, lsigma_, ogain_, lskip_);
  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initial_pose_);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,seed_);

  ROS_INFO("Initialization complete");

  return true;
}

bool SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose){
  if(!getOdomPose(gmap_pose, scan.header.stamp))
    return false;

  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_) {
    ROS_INFO("Inverting scan");
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++) {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  }
  else {
    for(unsigned int i=0; i < scan.ranges.size(); i++) {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(), ranges_double, gsp_laser_, scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);
  ROS_DEBUG("processing scan");

  return gsp_->processScan(reading);
}

void SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_) {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
    activate_time_ = ros::Time::now() + ros::Duration(settle_time_);
  }

  GMapping::OrientedPoint odom_pose;

  if(addScan(*scan, odom_pose)) {
    processed_scan_ = true;
    ROS_DEBUG("scan processed");

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_) {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  }
  else
    ROS_DEBUG("cannot process scan");
}

void SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan) {
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]), gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best = gsp_->getParticles()[gsp_->getBestParticleIndex()];

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  GMapping::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent){
    ROS_DEBUG("  %.3f %.3f %.3f", n->pose.x, n->pose.y, n->pose.theta);
    if(!n->reading){
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;

    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(), xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smap.getMapSizeX(); x++) {
    for(int y=0; y < smap.getMapSizeY(); y++) {
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_) {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_->resolve( map_frame_ );
}


nav_msgs::OccupancyGrid SlamGMapping::getGMap(){
  boost::mutex::scoped_lock map_lock(map_mutex_);
  return map_.map;
}

tf::StampedTransform SlamGMapping::getTransform(){
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  return tf::StampedTransform(map_to_odom_, tf_expiration, map_frame_, odom_frame_);
}



void SlamGMapping::resume(const GMapping::OrientedPoint& new_pose){
  GMapping::OrientedPoint odom_pose;
  getOdomPose(odom_pose, ros::Time(0));
  tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, new_pose.theta), tf::Vector3(new_pose.x, new_pose.y, 0.0)).inverse();
  tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
  gsp_->resume(new_pose, odom_pose);
}

