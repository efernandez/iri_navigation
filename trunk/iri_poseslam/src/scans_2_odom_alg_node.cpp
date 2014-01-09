#include "scans_2_odom_alg_node.h"

Scans2OdomAlgNode::Scans2OdomAlgNode(void) :
algorithm_base::IriBaseAlgorithm<Scans2OdomAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  
  // [init publishers]
  
  // [init subscribers]
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 10, &Scans2OdomAlgNode::scan_callback, this);
  
  // [init services]
  this->get_link_server_ = this->public_node_handle_.advertiseService("get_link", &Scans2OdomAlgNode::get_linkCallback, this);
  
  // [init clients]
  get_relative_pose_client_ = this->public_node_handle_.serviceClient<iri_laser_icp::GetRelativePose>("get_relative_pose");
  
  // [init action servers]
  
  // [init action clients]
}

Scans2OdomAlgNode::~Scans2OdomAlgNode(void)
{
  // [free dynamic memory]
}

void Scans2OdomAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]
  
  // [publish messages]
}

/*  [subscriber callbacks] */
void Scans2OdomAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_DEBUG("SCANS 2 ODOM New LaserScan Message Received");
  //ROS_INFO_STREAM(msg->header.covariance);
  
  // LASER SCAN BUFFER
  // reserve space if needed
  if (discarded_laser_scan_buffer_.size() == discarded_laser_scan_buffer_.capacity())
    discarded_laser_scan_buffer_.reserve(int(discarded_laser_scan_buffer_.size()) + 1000);
  
  discarded_laser_scan_buffer_.push_back(*msg);
  
  ROS_DEBUG("SCANS 2 ODOM New Laser Scan buffered");
  
}

/*  [service callbacks] */
bool Scans2OdomAlgNode::get_linkCallback(iri_poseslam::GetLink::Request &req, iri_poseslam::GetLink::Response &res)
{
  // reserve space if needed
  if (laser_scan_buffer_.size() == laser_scan_buffer_.capacity())
    laser_scan_buffer_.reserve(int(laser_scan_buffer_.size()) + 1000);
  
  res.success = false;
  res.end = false;
  
  // First callback (ask for first laser scan header)
  if (req.current_step == 0 && req.with_step == 0)
  {
    if (discarded_laser_scan_buffer_.size() > 0)
    {
      // give the first laser scan
      res.odom.header = discarded_laser_scan_buffer_.front().header;
      res.success = true;
      
      // Add into the laser_scan_buffer_
      laser_scan_buffer_.push_back(discarded_laser_scan_buffer_.front());
      
      // erase the first laser scan
      discarded_laser_scan_buffer_.erase(discarded_laser_scan_buffer_.begin(), discarded_laser_scan_buffer_.begin() + 1);
      
      ROS_INFO("SCANS 2 ODOM first laser scan header given");
    }
  }
  
  // ICP betwen 2 states
  else
  {
    //ROS_INFO("SCANS 2 ODOM compute link: %i with %i | laser buffer size = %i", int(req.current_step), int(req.with_step), int(laser_scan_buffer_.size()));
    
    if (discarded_laser_scan_buffer_.size() == 0)
    {
      if (laser_scan_buffer_.size() > 0)
        ROS_DEBUG("SCANS 2 ODOM there is no more laser scans for now...");
      
      res.end = true;
    }
    else
    {
      // ODOMETRY LINK
      if (req.current_step < req.with_step)
      {
        if (online_mode)
          get_relative_pose_srv_.request.scan_sens = discarded_laser_scan_buffer_.back();  // last laser scan
        else
          get_relative_pose_srv_.request.scan_sens = discarded_laser_scan_buffer_.front();  // next laser scan
            
        get_relative_pose_srv_.request.scan_ref = laser_scan_buffer_.at(req.current_step); // actual laser scan
        get_relative_pose_srv_.request.prior_d.position.x = 0;
        get_relative_pose_srv_.request.prior_d.position.y = 0;
        get_relative_pose_srv_.request.prior_d.position.z = 0;
        get_relative_pose_srv_.request.prior_d.orientation.x = 0;
        get_relative_pose_srv_.request.prior_d.orientation.y = 0;
        get_relative_pose_srv_.request.prior_d.orientation.z = 0;
        
        // call ICP
        if (get_relative_pose_client_.call(get_relative_pose_srv_))
        {
          ROS_DEBUG("SCANS 2 ODOM ICP in odometry success!");
          geometry_msgs::PoseWithCovarianceStamped d = get_relative_pose_srv_.response.pose_rel;
          
          // OFFLINE MODE
          if (!online_mode)
          {
            // Erase the computed Laser Scan from discarded_laser_scan_buffer_
            discarded_laser_scan_buffer_.erase(discarded_laser_scan_buffer_.begin(), discarded_laser_scan_buffer_.begin() + 1);
          }
          
          // ONLINE MODE
          else
          {
            // Too much covariance
            if ((d.pose.covariance.at(0)  > bad_cov_thres ||
              d.pose.covariance.at(7)  > bad_cov_thres ||
              d.pose.covariance.at(17) > bad_cov_thres))
            {
              // Try to reconstruct with intermediate discarded laser scans
              uint idx_LS = uint(discarded_laser_scan_buffer_.size() - 1);
              
              ROS_WARN("SCANS 2 ODOM ICP given a too much covariance odometry, trying to reconstruct with the %i discarded poses", idx_LS + 1);
              bool good_cov = false;
              
              while (!good_cov && idx_LS != 0) // iteration half by half of discarded laser scans
              {
                // new ICP request in a half distance laser scan
                idx_LS = int(idx_LS / 2);
                get_relative_pose_srv_.request.scan_sens = discarded_laser_scan_buffer_.at(idx_LS);
                
                if (get_relative_pose_client_.call(get_relative_pose_srv_))
                {
                  d = get_relative_pose_srv_.response.pose_rel;
                  
                  if (idx_LS > 0  && (d.pose.covariance.at(0)  > bad_cov_thres
                    ||  d.pose.covariance.at(7)  > bad_cov_thres
                    ||  d.pose.covariance.at(17) > bad_cov_thres))
                  {
                    ROS_INFO("SCANS 2 ODOM LaserScan %i still too much covariance - try with number %i", idx_LS + 1, int(idx_LS / 2) + 1);
                  }
                  else
                  {
                    if (idx_LS == 0) // while consecutive laser scan and still too much covariance ICP
                      ROS_WARN("SCANS 2 ODOM ICP given too much covariance even in consecutive laser scans. Consecutive scans computed.");
                    else
                      ROS_INFO("SCANS 2 ODOM LaserScan %i works! adding it into the laser scan buffer", idx_LS + 1);
                    
                    good_cov = true;
                    
                    // Erase the idx_LS and the previous ones from the discarded_laser_scan_buffer_
                    discarded_laser_scan_buffer_.erase(discarded_laser_scan_buffer_.begin(), discarded_laser_scan_buffer_.begin() + idx_LS + 1);
                    
                    ROS_INFO("SCANS 2 ODOM Removed discarded laser scans until %i. New discarded bufer size: %u", idx_LS + 1, uint(discarded_laser_scan_buffer_.size()));
                  }
                }
              }
            }
            else
              // Erase all laser scans of discarded_laser_scan_buffer_
              discarded_laser_scan_buffer_.clear();
          }
          
          // Results
          Vector3d link_ICP = pose_2_vector(d.pose.pose);
          Matrix3d link_ICP_cov = covariance_2_matrix(d.pose);
          
          // CHANGE FRAME TO BASE FOOTPRINT
          change_2_base_footprint_frame(link_ICP, link_ICP_cov);

          // COVARIANCE CORRECTION
          link_ICP_cov = link_ICP_cov * ICP_covariance_correction_factor;
          
          res.odom.header = get_relative_pose_srv_.request.scan_sens.header;
          res.odom.pose = eigen_2_posewithcovariance(link_ICP, link_ICP_cov);
          res.success = true;
          
          // Add the laser scan in the buffer
          laser_scan_buffer_.push_back(get_relative_pose_srv_.request.scan_sens);
        }
        else
        {
          ROS_WARN("SCANS 2 ODOM ICP failed in a odometry");
          res.success = false;
        }
      }
      
      // LOOP CLOSURE LINK
      else
      {
        get_relative_pose_srv_.request.scan_ref = laser_scan_buffer_.at(req.with_step);
        get_relative_pose_srv_.request.scan_sens = laser_scan_buffer_.at(req.current_step);
        get_relative_pose_srv_.request.prior_d = req.prior_d;
        
        // call ICP
        if (get_relative_pose_client_.call(get_relative_pose_srv_))
        {
          Vector3d link_ICP = pose_2_vector(get_relative_pose_srv_.response.pose_rel.pose.pose);
          Matrix3d link_ICP_cov = covariance_2_matrix(get_relative_pose_srv_.response.pose_rel.pose);
          
          // CHANGE FRAME TO BASE FOOTPRINT
          change_2_base_footprint_frame(link_ICP, link_ICP_cov);
          
          // COVARIANCE CORRECTION
          link_ICP_cov = link_ICP_cov * ICP_covariance_correction_factor;
          
          res.odom.header = get_relative_pose_srv_.request.scan_sens.header;
          res.odom.pose = eigen_2_posewithcovariance(link_ICP, link_ICP_cov);
          res.success = true;
          ROS_DEBUG("SCANS 2 ODOM ICP in LC result = = %f, %f, %f", res.odom.pose.pose.position.x, res.odom.pose.pose.position.y, tf::getYaw(res.odom.pose.pose.orientation));
          
        }
        else
        {
          ROS_WARN("SCANS 2 ODOM ICP failed in loop closure");
          res.success = false;
        }
      }
    }
  }
  
  return true;
}

Matrix3d Scans2OdomAlgNode::rotation_matrix(const double &alpha) const
{
  Matrix3d rot = MatrixXd::Identity(3,3);
  
  rot(0,0) = cos(alpha);
  rot(0,1) = -sin(alpha);
  rot(1,0) = sin(alpha);
  rot(1,1) = cos(alpha);
  
  return rot;
}

Matrix3d Scans2OdomAlgNode::covariance_2_matrix(const geometry_msgs::PoseWithCovariance &pose) const
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

Vector3d Scans2OdomAlgNode::pose_2_vector(const geometry_msgs::Pose &pose) const
{
  Vector3d p;
  
  p(0) = pose.position.x;
  p(1) = pose.position.y;
  p(2) = tf::getYaw(pose.orientation);
  
  return p;
}

geometry_msgs::PoseWithCovariance Scans2OdomAlgNode::eigen_2_posewithcovariance(const Vector3d &p, const Matrix3d &cov) const
{
  geometry_msgs::PoseWithCovariance pose;
  
  pose.pose.position.x = p(0);
  pose.pose.position.y = p(1);
  pose.pose.position.z = 0;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(p(2));
  
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

void Scans2OdomAlgNode::change_2_base_footprint_frame(Vector3d &odom_ICP, Matrix3d &odom_ICP_cov)
{
  Matrix3d rot_frame = rotation_matrix(d_base_2_laser(2));
  Matrix3d rot_d = rotation_matrix(odom_ICP(2));
  odom_ICP = d_base_2_laser + rot_frame * odom_ICP - rot_d * d_base_2_laser;
  
  Matrix3d J = rotation_matrix(d_base_2_laser(2));
  odom_ICP_cov = J * odom_ICP_cov * J.transpose();
}

/*  [action callbacks] */

/*  [action requests] */

void Scans2OdomAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  
  online_mode = config.online_mode;
  bad_cov_thres = config.bad_cov_thres;
  d_base_2_laser(0) = config.dx_base_2_laser;
  d_base_2_laser(1) = config.dy_base_2_laser;
  d_base_2_laser(2) = config.dth_base_2_laser;
  ICP_covariance_correction_factor = config.ICP_covariance_correction_factor;

  ROS_WARN("SCANS 2 ODOM: Config updated");

  this->alg_.unlock();
}

void Scans2OdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Scans2OdomAlgNode>(argc, argv, "scans_2_odom_alg_node");
}