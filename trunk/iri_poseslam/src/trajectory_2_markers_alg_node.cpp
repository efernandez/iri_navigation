#include "trajectory_2_markers_alg_node.h"

Trajectory2MarkersAlgNode::Trajectory2MarkersAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<Trajectory2MarkersAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->CovarianceMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("CovarianceMarkers", 1);
  this->TrajectoryMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("TrajectoryMarkers", 1);
  this->ActualPoseMarker_publisher_ = this->public_node_handle_.advertise<visualization_msgs::Marker>("ActualPoseMarker", 1);
  
  // [init subscribers]
  this->trajectory_subscriber_ = this->public_node_handle_.subscribe("trajectory", 100, &Trajectory2MarkersAlgNode::trajectory_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]

  nLoops_ = 0;
}

Trajectory2MarkersAlgNode::~Trajectory2MarkersAlgNode(void)
{
  // [free dynamic memory]
}

void Trajectory2MarkersAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  // [fill srv structure and make request to the server]
  // [fill action structure and make request to the action server]
  // [publish messages]
}

/*  [subscriber callbacks] */
void Trajectory2MarkersAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  //ROS_INFO("Trajectory2MarkersAlgNode::trajectory_callback: New Message Received");
  //ROS_INFO_STREAM(*msg);
  
  // draw covariances and if new trajectory, 
  drawTrajectory(*msg);
  
  // publish trajectory, covariance and actual markers
  this->CovarianceMarkers_publisher_.publish(get_covariance_markers());
  this->TrajectoryMarkers_publisher_.publish(get_trajectory_marker());
  this->ActualPoseMarker_publisher_.publish(get_actual_marker());
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void Trajectory2MarkersAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  config_ = config;

  this->alg_.unlock();

  initialize_markers();
}

void Trajectory2MarkersAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Trajectory2MarkersAlgNode>(argc, argv, "trajectory_2_markers_alg_node");
}

void Trajectory2MarkersAlgNode::initialize_markers()
{
  // trajectory line initialization
  trajectory_marker_.header.stamp = ros::Time::now();
  trajectory_marker_.header.frame_id = "/world";
  trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker_.action = visualization_msgs::Marker::ADD;
  
  trajectory_marker_.color.a = config_.line_color_a;
  trajectory_marker_.color.r = config_.line_color_r;
  trajectory_marker_.color.g = config_.line_color_g;
  trajectory_marker_.color.b = config_.line_color_b;
  
  trajectory_marker_.ns = "/trajectory";
  trajectory_marker_.id = 1;
  
  trajectory_marker_.scale.x = config_.line_width;
  
  // loop line initialization
  loops_marker_.header.stamp = ros::Time::now();
  loops_marker_.header.frame_id = "/world";
  loops_marker_.type = visualization_msgs::Marker::LINE_LIST;
  loops_marker_.action = visualization_msgs::Marker::ADD;
  
  loops_marker_.color.a = config_.line_loop_color_a;
  loops_marker_.color.r = config_.line_loop_color_r;
  loops_marker_.color.g = config_.line_loop_color_g;
  loops_marker_.color.b = config_.line_loop_color_b;
  
  loops_marker_.ns = "/loops";
  loops_marker_.id = 1;
  
  loops_marker_.scale.x = config_.line_loop_width;
  
  // actual marker initialization
  actual_marker_.header.stamp = ros::Time::now();
  actual_marker_.header.frame_id = "/world";
  actual_marker_.type = visualization_msgs::Marker::SPHERE;
  actual_marker_.action = visualization_msgs::Marker::ADD;
  
  actual_marker_.color.a = config_.actual_marker_color_a;
  actual_marker_.color.r = config_.actual_marker_color_r;
  actual_marker_.color.g = config_.actual_marker_color_g;
  actual_marker_.color.b = config_.actual_marker_color_b;
  
  actual_marker_.ns = "/actual";
  actual_marker_.id = 1;

  ROS_WARN("TR 2 MARKERS: Config updated");
}

// Trajectory2MarkersAlgNode Public API
void Trajectory2MarkersAlgNode::drawTrajectory(const iri_poseslam::Trajectory& msg)
{
  //ROS_INFO("TR 2 MARKERS: draw Covariances");
  
  // Reserve space
  if (loop_step_.capacity() == loop_step_.size())
    loop_step_.reserve(loop_step_.size() + 100);

  if (covariance_markers_.markers.capacity() == covariance_markers_.markers.size()) 
    covariance_markers_.markers.reserve(covariance_markers_.markers.size() + 100);

  if (trajectory_marker_.points.capacity() == trajectory_marker_.points.size()) 
    trajectory_marker_.points.reserve(trajectory_marker_.points.size() + 100);

  // Boolean vector of loopclosure
  bool LoopClosure = (msg.loops.size() > nLoops_ * 2);
  loop_step_.push_back(LoopClosure);

  uint step = msg.poses.size() - 1;
  
  // LOOP CLOSURE --> Recompute all markers
  if (LoopClosure)
  {
    //ROS_INFO("TR 2 MARKERS: Loop closed: nLoops_ = %i | msg.loops.size() / 2 = %i", nLoops_, msg.loops.size() / 2);
    
    // loops_marker_ (position of old loops and adding new(s) loop(s))
    nLoops_ = uint(msg.loops.size()) / 2;
    loops_marker_.points.clear();
    loops_marker_.points.reserve(nLoops_);

    for (uint i = 0; i < nLoops_; i++)
    {
      uint from = msg.loops.at(2 * i);
      uint with = msg.loops.at(2 * i + 1);
      
      loops_marker_.points.push_back(msg.poses.at(from).pose.pose.position);
      loops_marker_.points.push_back(msg.poses.at(with).pose.pose.position);
      loop_step_.at(from) = true;
      loop_step_.at(with) = true;
    }
    
    // covariance_markers_ & trajectory_marker_ (covariances and poses)
    covariance_markers_.markers.clear();
    covariance_markers_.markers.reserve(msg.states_2_steps.size());
    trajectory_marker_.points.clear();
    trajectory_marker_.points.reserve(msg.poses.size());

    for (uint i = 0; i < msg.poses.size(); i++)
    {
      // COVARIANCE for NON-REDUNDANT POSES
      if (msg.steps_2_states.at(i) != -1)
      {
        Eigen::Matrix2d covs = get_ith_cov(msg, i);
        double theta_cov = get_ith_theta_cov(msg, i);
        
        visualization_msgs::Marker aux_marker = create_marker(i, msg.header, covs, theta_cov, msg.poses.at(i).pose.pose.position, loop_step_.at(i));
        covariance_markers_.markers.push_back(aux_marker);
      }
      // TRAJECTORY for ALL POSES
      trajectory_marker_.points.push_back(msg.poses.at(i).pose.pose.position);
    }
  }
  // NOT LOOP CLOSURE
  else
  {
    // NOT REDUNDANT POSE --> Covariance marker
    if (msg.steps_2_states.back() != -1)
    {
      //ROS_INFO("TR 2 MARKERS: Non-redundant pose, step = %i msg.poses.size() = %i", step, msg.poses.size());

      //COVARIANCE
      Eigen::Matrix2d covs = get_ith_cov(msg, step);
      double theta_cov = get_ith_theta_cov(msg, step);
      
      visualization_msgs::Marker aux_marker = create_marker(step, msg.header, covs, theta_cov, msg.poses.back().pose.pose.position, false);
      
      covariance_markers_.markers.push_back(aux_marker);
    }
    // TRAJECTORY
    trajectory_marker_.points.push_back(msg.poses.back().pose.pose.position);
  } 
  
  // CHECK IF ANY MESSANGE HAVE BEEN LOST
  if (trajectory_marker_.points.size() - 1 != step)
    ROS_ERROR("TR 2 MARKERS: Step lost!");

  if (covariance_markers_.markers.size() != msg.states_2_steps.size())
    ROS_ERROR("TR 2 MARKERS: State lost!");
  
  // UPDATE ACTUAL MARKER
  Eigen::Matrix2d covs = get_ith_cov(msg, step);
  double theta_cov = get_ith_theta_cov(msg, step);
  change_actual_marker(covs, theta_cov, msg.poses.back().pose.pose.position, LoopClosure);
}

visualization_msgs::MarkerArray Trajectory2MarkersAlgNode::get_trajectory_marker() const
{
  visualization_msgs::MarkerArray trajectory_markers_array_;
  trajectory_markers_array_.markers.push_back(loops_marker_);
  trajectory_markers_array_.markers.push_back(trajectory_marker_);
  
  return trajectory_markers_array_;
}

visualization_msgs::Marker Trajectory2MarkersAlgNode::get_actual_marker() const
{
  return actual_marker_;
}

visualization_msgs::MarkerArray Trajectory2MarkersAlgNode::get_covariance_markers() const
{
  return covariance_markers_;
}

visualization_msgs::Marker Trajectory2MarkersAlgNode::create_marker(const uint& id, const std_msgs::Header& header, const Eigen::Matrix2d& covs, const double& theta_cov, const geometry_msgs::Point& position, const bool& loopClosure) const
{
  visualization_msgs::Marker current_marker_;
  current_marker_.header = header;
  current_marker_.type = visualization_msgs::Marker::SPHERE;
  current_marker_.action = visualization_msgs::Marker::ADD;
  
  if (loopClosure)
  {
    current_marker_.color.a = config_.marker_loop_color_a;
    current_marker_.color.r = config_.marker_loop_color_r;
    current_marker_.color.g = config_.marker_loop_color_g;
    current_marker_.color.b = config_.marker_loop_color_b;
  }
  else
  {
    current_marker_.color.a = config_.marker_color_a;
    current_marker_.color.r = config_.marker_color_r;
    current_marker_.color.g = config_.marker_color_g;
    current_marker_.color.b = config_.marker_color_b;
  }
  current_marker_.ns = "/positions";
  current_marker_.id = id;
  //ROS_INFO("step %u", id);
  
  //ROS_INFO("cov:\n%f\t%f \n%f\t%f",covs(0,0),covs(0,1),covs(1,0),covs(1,1));
  
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covs);

  const Eigen::Vector2d& eigValues (eig.eigenvalues());
  const Eigen::Matrix2d& eigVectors (eig.eigenvectors());

  double angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

  double lengthMajor = sqrt(eigValues[0]);
  double lengthMinor = sqrt(eigValues[1]);

  //ROS_INFO("eigValues: %f %f",eigValues[0],eigValues[1]);
  //ROS_INFO("lengthMajor/Minor: %f %f",lengthMajor,lengthMinor);

  current_marker_.scale.x = lengthMajor + 0.001;
  current_marker_.scale.y = lengthMinor + 0.001;
  current_marker_.scale.z = 2 * sqrt(theta_cov) + 0.001;
  
  current_marker_.pose.position.x = position.x;
  current_marker_.pose.position.y = position.y;
  current_marker_.pose.position.z = position.z;
  current_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  //current_marker_.pose.orientation.w = cos(angle*0.5);
  //current_marker_.pose.orientation.z = sin(angle*0.5);
  
  return current_marker_;
}

void Trajectory2MarkersAlgNode::change_actual_marker(const Eigen::Matrix2d& covs, const double& theta_cov, const geometry_msgs::Point& position, const bool& loopClosure)
{  
  if (loopClosure)
  {
    actual_marker_.color.a = config_.marker_loop_color_a;
    actual_marker_.color.r = config_.marker_loop_color_r;
    actual_marker_.color.g = config_.marker_loop_color_g;
    actual_marker_.color.b = config_.marker_loop_color_b;
  }
  else
  {
    actual_marker_.color.a = config_.actual_marker_color_a;
    actual_marker_.color.r = config_.actual_marker_color_r;
    actual_marker_.color.g = config_.actual_marker_color_g;
    actual_marker_.color.b = config_.actual_marker_color_b;
  }
    
  //ROS_INFO("cov:\n%f\t%f \n%f\t%f",covs(0,0),covs(0,1),covs(1,0),covs(1,1));
  
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covs);

  const Eigen::Vector2d& eigValues (eig.eigenvalues());
  const Eigen::Matrix2d& eigVectors (eig.eigenvectors());

  double angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

  double lengthMajor = sqrt(eigValues[0]);
  double lengthMinor = sqrt(eigValues[1]);

  //ROS_INFO("eigValues: %f %f",eigValues[0],eigValues[1]);
  //ROS_INFO("lengthMajor/Minor: %f %f",lengthMajor,lengthMinor);

  actual_marker_.scale.x = lengthMajor + 0.001;
  actual_marker_.scale.y = lengthMinor + 0.001;
  actual_marker_.scale.z = 2 * sqrt(theta_cov) + 0.001;
  
  actual_marker_.pose.position.x = position.x;
  actual_marker_.pose.position.y = position.y;
  actual_marker_.pose.position.z = position.z;
  actual_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  //current_marker_.pose.orientation.w = cos(angle*0.5);
  //current_marker_.pose.orientation.z = sin(angle*0.5);
}

Eigen::Matrix2d Trajectory2MarkersAlgNode::get_ith_cov(const iri_poseslam::Trajectory& msg, const uint i) const
{
  //ROS_INFO("get cov: msg.poses.size() = %i - idx = %i", msg.poses.size(), i);
  Eigen::Matrix2d covs;
  covs(0, 0) = msg.poses.at(i).pose.covariance.at(0);
  covs(0, 1) = msg.poses.at(i).pose.covariance.at(1);
  covs(1, 0) = msg.poses.at(i).pose.covariance.at(3);
  covs(1, 1) = msg.poses.at(i).pose.covariance.at(4);

  return covs;
}

double Trajectory2MarkersAlgNode::get_ith_theta_cov(const iri_poseslam::Trajectory& msg, const uint i) const
{
  //ROS_INFO("get theta cov: msg.poses.size() = %i - idx = %i", msg.poses.size(), i);
  
  return msg.poses.at(i).pose.covariance.at(8);
}