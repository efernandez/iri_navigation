#include "sensors_2_link_alg_node.h"

Sensors2LinkAlgNode::Sensors2LinkAlgNode(void) :
algorithm_base::IriBaseAlgorithm<Sensors2LinkAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  
  // [init publishers]
  
  // [init subscribers]
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 100, &Sensors2LinkAlgNode::scan_callback, this);
  this->odom_relative_subscriber_ = this->public_node_handle_.subscribe("odom_relative", 100, &Sensors2LinkAlgNode::odom_relative_callback, this);
  this->cmd_vel_subscriber_ = this->public_node_handle_.subscribe("cmd_vel", 100, &Sensors2LinkAlgNode::cmd_vel_callback, this);
  
  // [init services]
  this->get_link_server_ = this->public_node_handle_.advertiseService("get_link", &Sensors2LinkAlgNode::get_linkCallback, this);
  
  // [init clients]
  get_relative_pose_client_ = this->public_node_handle_.serviceClient<iri_laser_icp::GetRelativePose>("get_relative_pose");
  
  // [init action servers]
  
  // [init action clients]
  
  // init variables
  odom_rel_       = MatrixXd::Zero(3,1);                              
  odom_rel_cov_   = MatrixXd::Zero(3,3);                              
  Jp              = MatrixXd::Identity(3,3);                             
  Jd              = MatrixXd::Identity(3,3);                             
  fusion_ready_   = false;                              
  new_laser_scan_ = false;                              
  prev_seq        = 0;                             
}

Sensors2LinkAlgNode::~Sensors2LinkAlgNode(void)
{
  // [free dynamic memory]
}

void Sensors2LinkAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void Sensors2LinkAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("SENSORS 2 LINK: New laser scan received! time = %f", msg->header.stamp.toSec());

  // LOAD THE LASER SCAN
  last_laser_scan_ = (*msg);
  new_laser_scan_ = true;

  // ADD FIRST SCAN DIRECTLY TO laser_scan_buffer_
  if (laser_scan_buffer_.empty())
  {
    laser_scan_buffer_.push_back(*msg);
    new_laser_scan_ = false; // avoid self pose odometry
  }
}

void Sensors2LinkAlgNode::odom_relative_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (prev_seq + 1 < msg->header.seq)
    ROS_WARN("SENSORS 2 LINK: Any odom relative lost! prev_seq = %i current_seq = %i", prev_seq, msg->header.seq);

  prev_seq = msg->header.seq;

  Q = MatrixXd::Zero(3,3);
  d_local = MatrixXd::Zero(3,1);

  // stopped?
  if (!stopped_since_last_odom)
  {
    // Odometry
    d_local = pose_2_vector(msg->pose.pose);

    // Odometry noise
    Q = covariance_2_matrix(msg->pose);
  }
  else
    ROS_DEBUG("SENSORS 2 LINK: Zero motion relative odometry");

  // if now is stopped, update the boolean
  if (currently_stopped)
    stopped_since_last_odom = true;
  
  // accumulate odometry
  // TIME
  odom_rel_time_ = msg->header.stamp;

  // COVARIANCE PROPAGATION
  // Reference Point Jacobian (previous orientation)
  Jp(0,2) =-sin(odom_rel_(2)) * d_local(0) - cos(odom_rel_(2)) * d_local(1);
  Jp(1,2) = cos(odom_rel_(2)) * d_local(0) - sin(odom_rel_(2)) * d_local(1);

  // Displacement Jacobian
  Jd = rotation_matrix(odom_rel_(2));

  // Covariance propagation
  odom_rel_cov_ = Jp * odom_rel_cov_ * Jp.transpose() + Jd * Q * Jd.transpose();

  // POSE
  odom_rel_ += rotation_matrix(odom_rel_(2)) * d_local;

  // ADD IN THE BUFFER
  // reserve space if needed
  if (odom_rel_buffer_.size() == odom_rel_buffer_.capacity())
  {
    odom_rel_time_buffer_.reserve(int(odom_rel_time_buffer_.size()) + 1000);
    odom_rel_cov_buffer_.reserve(int(odom_rel_cov_buffer_.size()) + 1000);
    odom_rel_buffer_.reserve(int(odom_rel_buffer_.size()) + 1000);
  }
  odom_rel_time_buffer_.push_back(odom_rel_time_);
  odom_rel_cov_buffer_.push_back(odom_rel_cov_);
  odom_rel_buffer_.push_back(odom_rel_);

  
  // ODOMETRY FUSION ISSUES

  // OFFLINE MODE
  if (!online_mode && odom_rel_buffer_.size() > 1 && new_laser_scan_ && last_laser_scan_.header.stamp < msg->header.stamp)
  {
    // reserve space if needed
    if (odom_buffer_.size() == odom_buffer_.capacity())
      odom_buffer_.reserve(int(odom_buffer_.size()) + 1000);

    // ODOMETRY FUSION (LASER & ODOMETRY_RELATIVE)
    geometry_msgs::PoseWithCovarianceStamped new_odometry = odometry_fusion(last_laser_scan_, odom_rel_buffer_.size() - 1);
    odom_buffer_.push_back(new_odometry);
    new_laser_scan_ = false;

    //ROS_INFO("SENSORS 2 LINK: New precomputed odometry stored!");
  }

  // ONLINE MODE
  else
  {
    if (odom_rel_buffer_.size() > 1 && new_laser_scan_ && last_laser_scan_.header.stamp < msg->header.stamp)
    {
      fusion_ready_ = true;
      new_laser_scan_ = false;
      ready_laser_scan_ = last_laser_scan_;
      ready_odom_id_ = odom_rel_buffer_.size() - 1;
    }
  }
}

void Sensors2LinkAlgNode::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  currently_stopped = (msg->linear.x == 0 && msg->linear.y == 0 && msg->linear.z == 0 && msg->angular.x == 0 && msg->angular.y == 0 && msg->angular.z == 0);

  if (stopped_since_last_odom)
    stopped_since_last_odom = currently_stopped;
}

/*  [service callbacks] */
bool Sensors2LinkAlgNode::get_linkCallback(iri_poseslam::GetLink::Request &req, iri_poseslam::GetLink::Response &res)
{
  //ROS_INFO("SENSORS 2 LINK: New GetLink Request Received!");

  res.success = false;
  res.end = false;

  // FIRST CALLBACK (ask for first laser scan header)
  if (req.current_step == 0 && req.with_step == 0)
  {
    if (!laser_scan_buffer_.empty())
    {
      // give the first laser scan
      res.odom.header = laser_scan_buffer_.front().header;
      res.success = true;

      ROS_INFO("SENSORS 2 LINK: first laser scan header given");
    }
    //else
      //ROS_WARN("SENSORS 2 LINK: couldn't give the first laser scan header, not received yet...");
  }

  // NORMAL CALLBACK
  else
  {
    // ODOMETRY LINK
    if (req.current_step < req.with_step)
    {
      // OFFLINE MODE
      if (!online_mode)
        res = offline_odometry();
      
      // ONLINE MODE
      else
        res = online_odometry();
    }

    // LOOP CLOSURE LINK
    else
    { 
      if (req.current_step != laser_scan_buffer_.size() - 1)
        ROS_WARN("SENSORS 2 LINK: current_scan is not last_scan in LOOP CLOSURE");
        
      get_relative_pose_srv_.request.scan_ref = laser_scan_buffer_.at(req.with_step);
      get_relative_pose_srv_.request.scan_sens = laser_scan_buffer_.at(req.current_step);
      
      Vector3d prior_d = pose_2_vector(get_relative_pose_srv_.response.pose_rel.pose.pose);
      base_2_laser_frame(prior_d);
      get_relative_pose_srv_.request.prior_d = vector_2_pose(prior_d);

      // call ICP
      if (get_relative_pose_client_.call(get_relative_pose_srv_))
      {
        if (get_relative_pose_srv_.response.success)
        {
          Vector3d link_ICP = pose_2_vector(get_relative_pose_srv_.response.pose_rel.pose.pose);
          Matrix3d link_ICP_cov = covariance_2_matrix(get_relative_pose_srv_.response.pose_rel.pose);
          
          // CHANGE FRAME TO BASE FOOTPRINT
          laser_2_base_frame(link_ICP, link_ICP_cov);

          // COVARIANCE CORRECTION
          link_ICP_cov = link_ICP_cov * ICP_covariance_correction_factor;

          res.odom.header = get_relative_pose_srv_.request.scan_sens.header;
          res.odom.pose = eigen_2_posewithcovariance(link_ICP, link_ICP_cov);
          
          res.success = true;
          //ROS_INFO("SENSORS 2 LINK: ICP in LC prior = %f, %f, %f", req.prior_d.position.x, req.prior_d.position.y, tf::getYaw(req.prior_d.orientation));
          //ROS_INFO("SENSORS 2 LINK: ICP in LC result = %f, %f, %f - seq: %i - %i", res.odom.pose.pose.position.x, res.odom.pose.pose.position.y, tf::getYaw(res.odom.pose.pose.orientation), laser_scan_buffer_.at(req.with_step).header.seq, laser_scan_buffer_.at(req.current_step).header.seq);
          //ROS_INFO("SENSORS 2 LINK: ICP covariance:");
          //ROS_INFO_STREAM(link_ICP_cov);
        }
        else
        {
          ROS_WARN("SENSORS 2 LINK: ICP didn't found a match in loop closure");
          res.success = false;
        }
      }
      else
      {
        ROS_ERROR("SENSORS 2 LINK: ICP communication failed in loop closure");
        res.success = false;
      }
    }
  }

  return true;
}

/*  [action callbacks] */

/*  [action requests] */

void Sensors2LinkAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  stopped_since_last_odom = config.cmd_vel_available;
  currently_stopped = config.cmd_vel_available;
  online_mode = config.online_mode;
  d_base_2_laser(0) = config.dx_base_2_laser;
  d_base_2_laser(1) = config.dy_base_2_laser;
  d_base_2_laser(2) = config.dth_base_2_laser;
  ICP_covariance_correction_factor = config.ICP_covariance_correction_factor;
  
  ROS_WARN("SENSORS 2 LINK: Config updated");

  this->alg_.unlock();
}

void Sensors2LinkAlgNode::addNodeDiagnostics(void)
{
}

geometry_msgs::PoseWithCovarianceStamped Sensors2LinkAlgNode::odometry_fusion(const sensor_msgs::LaserScan &laser_scan, const int &odom_rel_idx)
{
  Vector3d odom, odom_fused, odom_ICP;
  Matrix3d odom_cov, odom_fused_cov, odom_ICP_cov;
  geometry_msgs::PoseWithCovarianceStamped fused_odom;
  
  // RELATIVE ODOMETRY INTERPOLATION
  std::pair<Vector3d, Matrix3d> int_odom = interpolate_odom_rel(odom_rel_idx, laser_scan.header.stamp);

  Vector3d int_odom_rel = int_odom.first;
  Matrix3d int_odom_rel_cov = int_odom.second;

  // LASER SCANS ICP
  get_relative_pose_srv_.request.scan_sens = laser_scan;  // next laser scan
  get_relative_pose_srv_.request.scan_ref = laser_scan_buffer_.back(); // current laser scan
  
  base_2_laser_frame(int_odom_rel);
  get_relative_pose_srv_.request.prior_d = vector_2_pose(int_odom_rel);

  // call ICP
  if (get_relative_pose_client_.call(get_relative_pose_srv_))
  {
    if (get_relative_pose_srv_.response.success)
    {
      ROS_DEBUG("SENSORS 2 LINK: ICP in odometry success!");
      odom_ICP = pose_2_vector(get_relative_pose_srv_.response.pose_rel.pose.pose);
      odom_ICP_cov = covariance_2_matrix(get_relative_pose_srv_.response.pose_rel.pose);

      // CHANGE FRAME TO BASE FOOTPRINT
      laser_2_base_frame(odom_ICP, odom_ICP_cov);
      
      // COVARIANCE CORRECTION
      odom_ICP_cov = odom_ICP_cov * ICP_covariance_correction_factor;
      
      //ROS_INFO("SENSORS 2 LINK: ICP in odometry prior = %f, %f, %f", get_relative_pose_srv_.request.prior_d.position.x, get_relative_pose_srv_.request.prior_d.position.y, tf::getYaw(get_relative_pose_srv_.request.prior_d.orientation));
      //ROS_INFO("SENSORS 2 LINK: ICP in odometry result = %f, %f, %f", get_relative_pose_srv_.response.pose_rel.pose.pose.position.x, get_relative_pose_srv_.response.pose_rel.pose.pose.position.y, tf::getYaw(get_relative_pose_srv_.response.pose_rel.pose.pose.orientation));

      // BEST ODOMETRY ELECTION
      // Motion zero
      if (int_odom_rel_cov.isZero())
      {
        //ROS_INFO("SENSORS 2 LINK: Zero motion odometry");
        odom_cov = int_odom_rel_cov;
        odom = int_odom_rel;
      }
      else
      {
        // // FUSION
        // // covariance
        // odom_fused_cov = (int_odom_rel_cov.inverse() + odom_ICP_cov.inverse()).inverse();
        // // mean 
        // odom_fused = odom_fused_cov * (int_odom_rel_cov.inverse() * int_odom_rel + odom_ICP_cov.inverse() * odom_ICP);
        
        // Best is interpolated odom
        if (int_odom_rel_cov.trace() < odom_ICP_cov.trace())
        {
          odom = int_odom_rel;
          odom_cov = int_odom_rel_cov;
        }
        // Best is ICP odom
        else
        {
          odom = odom_ICP;
          odom_cov = odom_ICP_cov;
        }
        
        //ROS_INFO("odom_ICP_cov:");
        //ROS_INFO_STREAM(odom_ICP_cov);
        //ROS_INFO("int_odom_rel_cov:");
        //ROS_INFO_STREAM(int_odom_rel_cov);
        //ROS_INFO("odom_cov:");
        //ROS_INFO_STREAM(odom_cov);
        //ROS_INFO("SENSORS 2 LINK: Fused odometry %f %f %f", odom(0), odom(1), odom(2));
      }
    }
    else
    {
      ROS_WARN("SENSORS 2 LINK: ICP didn't found a match in odometry fusion");
      odom_cov = int_odom_rel_cov;
      odom = int_odom_rel;
    }
  }
  else
  {
    ROS_WARN("SENSORS 2 LINK: ICP communication failed in odometry fusion");
    odom_cov = int_odom_rel_cov;
    odom = int_odom_rel;
  }

  // reserve space if needed
  if (laser_scan_buffer_.size() == laser_scan_buffer_.capacity())
    laser_scan_buffer_.reserve(int(laser_scan_buffer_.size()) + 1000);
  
  // Add laser scan in the laser_scan_buffer_
  laser_scan_buffer_.push_back(laser_scan);
  
  // TRANSLATING to a PoseWithCovariancestamped
  fused_odom.header = laser_scan.header;
  fused_odom.pose = eigen_2_posewithcovariance(odom, odom_cov);
  
  return fused_odom;
}

Matrix3d Sensors2LinkAlgNode::rotation_matrix(const double &alpha) const
{
  Matrix3d rot = MatrixXd::Identity(3,3);
  
  rot(0,0) = cos(alpha);
  rot(0,1) = -sin(alpha);
  rot(1,0) = sin(alpha);
  rot(1,1) = cos(alpha);

  return rot;
}

Matrix3d Sensors2LinkAlgNode::covariance_2_matrix(const geometry_msgs::PoseWithCovariance &pose) const
{
  Matrix3d cov;
  
  cov(0,0) = pose.covariance[0];
  cov(0,1) = pose.covariance[1];
  cov(0,2) = pose.covariance[5];
  cov(1,0) = pose.covariance[6];
  cov(1,1) = pose.covariance[7];
  cov(1,2) = pose.covariance[11];
  cov(2,0) = pose.covariance[30];
  cov(2,1) = pose.covariance[31];
  cov(2,2) = pose.covariance[35];

  return cov;
}

Vector3d Sensors2LinkAlgNode::pose_2_vector(const geometry_msgs::Pose &pose) const
{
  Vector3d p;
  
  p(0) = pose.position.x;
  p(1) = pose.position.y;
  p(2) = tf::getYaw(pose.orientation);
  
  return p;
}

geometry_msgs::PoseWithCovariance Sensors2LinkAlgNode::eigen_2_posewithcovariance(const Vector3d &p, const Matrix3d &cov) const
{
  geometry_msgs::PoseWithCovariance pose;

  pose.pose = vector_2_pose(p);
  
  pose.covariance[0]  = cov(0,0);
  pose.covariance[1]  = cov(0,1);
  pose.covariance[5]  = cov(0,2);
  pose.covariance[6]  = cov(1,0);
  pose.covariance[7]  = cov(1,1);
  pose.covariance[11] = cov(1,2);
  pose.covariance[30] = cov(2,0);
  pose.covariance[31] = cov(2,1);
  pose.covariance[35] = cov(2,2);
  
  return pose;
}

geometry_msgs::Pose Sensors2LinkAlgNode::vector_2_pose(const Vector3d &p) const
{
  geometry_msgs::Pose pose;

  pose.position.x = p(0);
  pose.position.y = p(1);
  pose.position.z = 0;
  pose.orientation = tf::createQuaternionMsgFromYaw(p(2));
  
  return pose;
}

iri_poseslam::GetLink::Response Sensors2LinkAlgNode::offline_odometry()
{
  iri_poseslam::GetLink::Response result;
 
  // Return the pre-computed odometry
  if (!odom_buffer_.empty())
  {
    result.odom = odom_buffer_.front();
    result.success = true;
    result.end = false;

    // Erase from the buffer
    odom_buffer_.erase(odom_buffer_.begin());
  }
  else
  {
    ROS_DEBUG("SENSORS 2 LINK: There is no more precomputed odometries");
    result.success = false;
    result.end = true;
  }
  
  return result;
}

iri_poseslam::GetLink::Response Sensors2LinkAlgNode::online_odometry()
{
  iri_poseslam::GetLink::Response result;
  
  if (fusion_ready_)
  {
    // Call sensor fusion
    result.odom = odometry_fusion(ready_laser_scan_, ready_odom_id_);
    result.success = true;
    result.end = false;
    fusion_ready_ = false;
  }
  else
  {
    ROS_DEBUG("SENSORS 2 LINK: There is not enough sensor data");
    result.success = false;
    result.end = true;
  }
  
  return result;
}

std::pair<Vector3d, Matrix3d> Sensors2LinkAlgNode::interpolate_odom_rel(const int &odom_rel_idx, const ros::Time scan_stamp)
{
  Vector3d pre_odom_rel  = odom_rel_buffer_.at(odom_rel_idx - 1);
  Vector3d post_odom_rel = odom_rel_buffer_.at(odom_rel_idx);
  Matrix3d pre_odom_rel_cov  = odom_rel_cov_buffer_.at(odom_rel_idx - 1);
  Matrix3d post_odom_rel_cov = odom_rel_cov_buffer_.at(odom_rel_idx);
  ros::Time pre_odom_rel_time  = odom_rel_time_buffer_.at(odom_rel_idx - 1);
  ros::Time post_odom_rel_time = odom_rel_time_buffer_.at(odom_rel_idx);

  // Interpolation
  double alpha = (scan_stamp - pre_odom_rel_time).toSec() / (post_odom_rel_time - pre_odom_rel_time).toSec();
  alpha = 1; // NOT INTERPOLE: posterior odometry message
  //alpha = 0; // NOT INTERPOLE: previous odometry message
  VectorXd int_odom_rel = (1 - alpha) * pre_odom_rel + alpha * post_odom_rel;
  MatrixXd int_odom_rel_cov = (1 - alpha) * pre_odom_rel_cov + alpha * post_odom_rel_cov;

  // Update relative odometries buffers
  MatrixXd rot = rotation_matrix(int_odom_rel(2));
  VectorXd rest_odom_rel = rot * (post_odom_rel - int_odom_rel);

  rot = rotation_matrix(-int_odom_rel(2));
  MatrixXd rest_odom_rel_cov = rot * (post_odom_rel_cov - int_odom_rel_cov) * rot.transpose();

  odom_rel_buffer_.clear();
  odom_rel_cov_buffer_.clear();
  odom_rel_time_buffer_.clear();

  odom_rel_buffer_.push_back(rest_odom_rel);
  odom_rel_cov_buffer_.push_back(rest_odom_rel_cov);
  odom_rel_time_buffer_.push_back(pre_odom_rel_time + (post_odom_rel_time - pre_odom_rel_time) * alpha);

  // initialize odom relative
  odom_rel_ = rest_odom_rel; //MatrixXd::Zero(3,1);
  odom_rel_cov_ = rest_odom_rel_cov; //MatrixXd::Zero(3,3);

  std::pair<Vector3d, Matrix3d> result;
  result.first = int_odom_rel;
  result.second = int_odom_rel_cov;

  return result;
}

void Sensors2LinkAlgNode::base_2_laser_frame(Vector3d &prior_d)
{
  Matrix3d rot_frame = rotation_matrix(-d_base_2_laser(2));
  Matrix3d rot_d = rotation_matrix(prior_d(2));
  
  prior_d = rot_d * rot_frame * d_base_2_laser + rot_frame * prior_d - rot_frame * d_base_2_laser;

}

void Sensors2LinkAlgNode::laser_2_base_frame(Vector3d &odom_ICP, Matrix3d &odom_ICP_cov)
{
  Matrix3d rot_frame = rotation_matrix(d_base_2_laser(2));
  Matrix3d rot_d = rotation_matrix(odom_ICP(2));
  odom_ICP = d_base_2_laser + rot_frame * odom_ICP - rot_d * d_base_2_laser;

  Matrix3d J = rotation_matrix(d_base_2_laser(2));
  odom_ICP_cov = J * odom_ICP_cov * J.transpose();
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Sensors2LinkAlgNode>(argc, argv, "sensors_2_link_node");
}
