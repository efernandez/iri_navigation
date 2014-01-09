#include "trajectory_3Dscans_2_pointcloud_alg_node.h"

Trajectory3DScans2PointcloudAlgNode::Trajectory3DScans2PointcloudAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<Trajectory3DScans2PointcloudAlgorithm>()
{
  //init class attributes if necessary
  emptyPointCloud_ = true;
  started_ = false;

  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->slices3D_pointcloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("slices3D_pointcloud", 10);

  // [init subscribers]
  this->slices3D_subscriber_ = this->public_node_handle_.subscribe("slices3D", 1000, &Trajectory3DScans2PointcloudAlgNode::slices3D_callback, this);
  this->trajectory_subscriber_ = this->public_node_handle_.subscribe("trajectory", 1000, &Trajectory3DScans2PointcloudAlgNode::trajectory_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

Trajectory3DScans2PointcloudAlgNode::~Trajectory3DScans2PointcloudAlgNode(void)
{
  // [free dynamic memory]
}

void Trajectory3DScans2PointcloudAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void Trajectory3DScans2PointcloudAlgNode::slices3D_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{  
  //ROS_INFO("SLICES 2 PC New slice Message Received"); 
  //ROS_INFO_STREAM(msg->header.covariance);
  
  // reserve space if needed
  if (slice_buffer_.size() == slice_buffer_.capacity())
    slice_buffer_.reserve(slice_buffer_.size() + 1000);

  if (started_)
    slice_buffer_.push_back(*msg);
}

void Trajectory3DScans2PointcloudAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  //ROS_INFO("SLICES 2 PC: New Trajectory Message Received");
  //ROS_INFO_STREAM(*msg);
  
  // RECOMPUTE PointCloud if have been 2 trajectory messages
  if (started_)
    recompute_PointCloud_msg(*msg);
  else
    started_ = true;

  slices3D_pointcloud_publisher_.publish(this->PointCloud_msg_);
  ROS_DEBUG("SLICES 2 PC: PointCloud_msg_ published! size = %u", PointCloud_msg_.height * PointCloud_msg_.width);
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void Trajectory3DScans2PointcloudAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  
  T_laser_frame = transformation_matrix(config.dx_base_2_h3d, config.dy_base_2_h3d, config.dz_base_2_h3d, config.dth_base_2_h3d);

  ROS_WARN("SLICES 2 PC: Config updated");

  this->alg_.unlock();
}

void Trajectory3DScans2PointcloudAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<Trajectory3DScans2PointcloudAlgNode>(argc, argv, "trajectory_3Dscans_2_pointcloud_alg_node");
}

void Trajectory3DScans2PointcloudAlgNode::recompute_PointCloud_msg(const iri_poseslam::Trajectory& trajectory)
{
  // AUX VARIABLES
  geometry_msgs::Pose slice_pose;
  bool LoopClosed = false;
  if (trajectory.loops.size() > 0)
    LoopClosed = (trajectory.loops.back() == trajectory.states_2_steps.back() && trajectory.steps_2_states.back() != -1);
  
  // LOOP CLOSED: Recompute all previous slices
  if (LoopClosed) 
  {
    clear_PointCloud_msg();
    
    //ROS_INFO("SLICES 2 PC: Recompute all %u slices of the trajectory with %u steps", uint(trajectory_slice_buffer_.size()), uint(trajectory.poses.size()));
    
    for (uint i = 0; i < trajectory_slice_buffer_.size(); i++)
    {
      slice_pose = interpole_slice_pose(i, trajectory);
      add_to_PointCloud_msg(transform_point_cloud(trajectory_slice_buffer_.at(i), slice_pose));
    }
    ROS_INFO("SLICES 2 PC: Pointcloud recomputed after a Loop Closure");
  }
  
  // ADD NEW SLICES
  while (slice_buffer_.size() > 0 && slice_buffer_.front().header.stamp < trajectory.poses.back().header.stamp)
  {
    //ROS_INFO("SLICES 2 PC: Add new slices: slice_buffer_.size = %u", uint(slice_buffer_.size()));
    
    // ADD to trajectory_slice_buffer_ and interpolation index and factor
    trajectory_slice_buffer_.push_back(slice_buffer_.front());
    interpolation_buffer_.push_back(search_interpolation(slice_buffer_.front(), trajectory));

    // ADD to the PointCloud
    slice_pose = interpole_slice_pose(trajectory_slice_buffer_.size() - 1, trajectory);
    add_to_PointCloud_msg(transform_point_cloud(slice_buffer_.front(), slice_pose));
    
    // ERASE from the slice_buffer_
    slice_buffer_.erase(slice_buffer_.begin());
  }
  //ROS_INFO("SLICES 2 PC: New slices added: slice_buffer_.size = %u", uint(slice_buffer_.size()));
}

sensor_msgs::PointCloud2 Trajectory3DScans2PointcloudAlgNode::transform_point_cloud(const sensor_msgs::PointCloud2& pcloud, const geometry_msgs::Pose& pose) 
{
  sensor_msgs::PointCloud2 transformed_pcloud;
  
  Matrix4f T_pose = transformation_matrix(pose.position.x, pose.position.y, pose.position.z, tf::getYaw(pose.orientation));
  
  pcl_ros::transformPointCloud(T_pose * T_laser_frame, pcloud, transformed_pcloud);

  return transformed_pcloud;
}

void Trajectory3DScans2PointcloudAlgNode::add_to_PointCloud_msg(const sensor_msgs::PointCloud2& newPointCloud)
{
  //ROS_INFO("ADD to pointcloud");
  pcl::PointCloud<pcl::PointXYZ> newcloud;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(newPointCloud, *cloud2);
  
  if (emptyPointCloud_)
  {
    newcloud = *cloud2;
    emptyPointCloud_ = false;
    //ROS_INFO("added empty");
  
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(PointCloud_msg_, *cloud1);
    
    newcloud = *cloud1 + *cloud2;
  }
  
  pcl::toROSMsg(newcloud, PointCloud_msg_);
  PointCloud_msg_.header.frame_id = "/map";
}

void Trajectory3DScans2PointcloudAlgNode::clear_PointCloud_msg()
{
  pcl::PointCloud<pcl::PointXYZ> cloud; //::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(PointCloud_msg_, cloud);
  
  cloud.clear();
  
  pcl::toROSMsg(cloud, PointCloud_msg_);
}

geometry_msgs::Pose Trajectory3DScans2PointcloudAlgNode::interpole_slice_pose(const uint& id, const iri_poseslam::Trajectory& trajectory) const
{
  // GET THE INTERPOLATION INDEX AND FACTOR
  uint id_pre  = interpolation_buffer_.at(id).first;
  double alpha = interpolation_buffer_.at(id).second;

  // GET THE PRE AND POST POSES
  geometry_msgs::Point pre_position  = trajectory.poses.at(id_pre).pose.pose.position;
  geometry_msgs::Point post_position = trajectory.poses.at(id_pre + 1).pose.pose.position;
  double pre_yaw = tf::getYaw(trajectory.poses.at(id_pre).pose.pose.orientation);
  double post_yaw = tf::getYaw(trajectory.poses.at(id_pre + 1).pose.pose.orientation);
  double d_yaw = pi_2_pi(post_yaw- pre_yaw);

  // INTERPOLE
  geometry_msgs::Pose slice_pose;
  slice_pose.position.x = (1 - alpha) * pre_position.x + alpha * post_position.x;
  slice_pose.position.y = (1 - alpha) * pre_position.y + alpha * post_position.y;
  slice_pose.position.z = (1 - alpha) * pre_position.z + alpha * post_position.z;
  slice_pose.orientation = tf::createQuaternionMsgFromYaw(pre_yaw + alpha * d_yaw);
  
  return slice_pose;
}

std::pair<uint, double> Trajectory3DScans2PointcloudAlgNode::search_interpolation(const sensor_msgs::PointCloud2& slice, const iri_poseslam::Trajectory& trajectory) const
{
  // SEARCH THE INTERPOLATION INDEX
  bool found = false;
  uint i = 1;
  uint id = 0;
  while (!found && i < trajectory.poses.size())
  {
    if (slice.header.stamp < trajectory.poses.at(i).header.stamp)
    {
      id  = i - 1;
      found = true;
    }
    else
      i++;
  }

  if (!found)
    ROS_ERROR("SLICES 2 PC: Interpolation index not found");

  // COMPUTE THE INTERPOLATION FACTOR
  ros::Time pre_time  = trajectory.poses.at(id).header.stamp;
  ros::Time post_time = trajectory.poses.at(id + 1).header.stamp;
  double alpha = (slice.header.stamp - pre_time).toSec() / (post_time - pre_time).toSec();

  std::pair<uint, double> result(id, alpha);

  return result;
}

Matrix4f Trajectory3DScans2PointcloudAlgNode::transformation_matrix(const float x, const float y, const float z, const float alpha) const
{
  Matrix4f T = MatrixXf::Identity(4,4);

  // Rotation
  T(0,0) =  cos(alpha);
  T(0,1) = -sin(alpha);
  T(1,0) =  sin(alpha);
  T(1,1) =  cos(alpha);

  // Translation
  T(0,3) = x;
  T(1,3) = y;
  T(2,3) = z;

  return T;
}

double Trajectory3DScans2PointcloudAlgNode::pi_2_pi(const double& angle) const
{
  return angle - 2 * M_PI * floor((angle + M_PI)/(2 * M_PI));
}