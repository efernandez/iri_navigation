#include "trajectory_scans_2_pointcloud_alg_node.h"

TrajectoryScans2PointcloudAlgNode::TrajectoryScans2PointcloudAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TrajectoryScans2PointcloudAlgorithm>()
{
  //init class attributes if necessary
  emptyPointCloud_ = true;
  
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->laser_pointcloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("laser_pointcloud", 100);
  
  // [init subscribers]
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 1000, &TrajectoryScans2PointcloudAlgNode::scan_callback, this);
  this->trajectory_subscriber_ = this->public_node_handle_.subscribe("trajectory", 100, &TrajectoryScans2PointcloudAlgNode::trajectory_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TrajectoryScans2PointcloudAlgNode::~TrajectoryScans2PointcloudAlgNode(void)
{
  // [free dynamic memory]
}

void TrajectoryScans2PointcloudAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void TrajectoryScans2PointcloudAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{  
  //ROS_INFO("TR 2 PC New LaserScan Message Received"); 
  //ROS_INFO_STREAM(msg->header.covariance);
  
  // reserve space if needed
  if (laser_scan_buffer_.size() == laser_scan_buffer_.capacity())
    laser_scan_buffer_.reserve(laser_scan_buffer_.size() + 1000);

  laser_scan_buffer_.push_back(*msg);
}

void TrajectoryScans2PointcloudAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  //ROS_INFO("TR 2 PC: New Trajectory Message Received");
  //ROS_INFO_STREAM(*msg);
  
  iri_poseslam::Trajectory new_trajectory = *msg;
  
  // RECOMPUTE trajectory_laser_scan_buffer_
  if (new_trajectory.steps_2_states.back() != -1 || publish_redundant_)
  {
    //ROS_INFO("TR 2 PC: \n\tRecompute trajectory_laser_scan_buffer_ size = %u \n\t laser_scan_buffer_.size = %u", trajectory_laser_scan_buffer_.size(), laser_scan_buffer_.size());
    
    bool added = false;
    while (!added)
    {
      if (laser_scan_buffer_.front().header.seq == new_trajectory.poses.back().header.seq)
      {
      	trajectory_laser_scan_buffer_.push_back(laser_scan_buffer_.front());
      	laser_scan_buffer_.erase(laser_scan_buffer_.begin());
      	added = true;
      	//ROS_INFO("TR 2 PC: laser scan added! \n\tnSteps = %i \n\tnStates = %i \n\ttrajectory_laser_scan_buffer_.size = %i", new_trajectory.steps_2_states.size(), new_trajectory.states_2_steps.size(), trajectory_laser_scan_buffer_.size());
      }
      else if (laser_scan_buffer_.front().header.seq < new_trajectory.poses.back().header.seq)
        laser_scan_buffer_.erase(laser_scan_buffer_.begin());
      else
      {
      	ROS_ERROR("TR 2 PC: Last trajectory pose laser scan not found!");
      	sensor_msgs::LaserScan empty_LaserScan;
        empty_LaserScan.header = new_trajectory.poses.back().header;
      	trajectory_laser_scan_buffer_.push_back(empty_LaserScan);
      	added = true;
      }
      
    }
  }
  
  // RECOMPUTE PointCloud
  if (recompute_PointCloud_msg(new_trajectory))
  {
    // PUBLISH if pointCloud has changed
    this->laser_pointcloud_publisher_.publish(this->PointCloud_msg_);
    ROS_DEBUG("TR 2 PC: PointCloud_msg_ published! size = %u", PointCloud_msg_.height * PointCloud_msg_.width);
  }
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TrajectoryScans2PointcloudAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  T_laser_frame = transformation_matrix(config.dx_base_2_laser, config.dy_base_2_laser, config.dz_base_2_laser, config.dth_base_2_laser);
  publish_redundant_ = config.publish_redundant;

  ROS_WARN("TR 2 PC: Config updated");
  
  this->alg_.unlock();
}

void TrajectoryScans2PointcloudAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TrajectoryScans2PointcloudAlgNode>(argc, argv, "trajectory_scans_2_pointcloud_alg_node");
}

bool TrajectoryScans2PointcloudAlgNode::recompute_PointCloud_msg(const iri_poseslam::Trajectory& trajectory)
{
  bool cloud_changed;
  bool LoopClosed = false;
  if (trajectory.loops.size() > 0)
    LoopClosed = (trajectory.loops.back() == trajectory.states_2_steps.back() && trajectory.steps_2_states.back() != -1);
  
  // LOOP CLOSED: Recompute all scans
  if (LoopClosed)
  {
    clear_PointCloud_msg();
    
    ROS_DEBUG("TR 2 PC: Loop Closed! Recompute all %u poses of the trajectory with %u laser scans", uint(trajectory.poses.size()), uint(trajectory_laser_scan_buffer_.size()));
    
    for (uint i = 0; i < trajectory.poses.size(); i++)
    {
      sensor_msgs::LaserScan laser_scan;

      if (publish_redundant_)
        laser_scan = trajectory_laser_scan_buffer_.at(i);
      
      else if (trajectory.steps_2_states.at(i) != -1)
        laser_scan = trajectory_laser_scan_buffer_.at(trajectory.steps_2_states.at(i));
      
      if (publish_redundant_ || trajectory.steps_2_states.at(i) != -1)
      {
        if (laser_scan.header.seq != trajectory.poses.at(i).header.seq)
          ROS_ERROR("TR 2 PC: headers don't match! step %i", i);

        add_to_PointCloud_msg(laser_scan_to_point_cloud(laser_scan, trajectory.poses.at(i).pose.pose));
        cloud_changed = true;
      }  
    }
    //ROS_INFO("TR 2 PC: Loop Closed - nLoops = %i - nStates = %i", trajectory.loops.size(), trajectory.poses.size());
  }

  // NOT LOOP: ADD CURRENT SCAN
  else if (trajectory.steps_2_states.back() != -1 || publish_redundant_) //NO LOOP CLOSED AND NO REDUNDANT POSE --> Compute last laserscan pointcloud
  {
    //ROS_INFO("TR 2 PC: New state: %i step %i \ntrajectory.steps_2_states.size() = %i\ntrajectory.poses.size() = %i\nlaser_scan_buffer_.size() = %i", trajectory.steps_2_states.back(), trajectory.states_2_steps.back(), trajectory.steps_2_states.size(), trajectory.poses.size(),laser_scan_buffer_.size());
    add_to_PointCloud_msg(laser_scan_to_point_cloud(trajectory_laser_scan_buffer_.back(), trajectory.poses.back().pose.pose));
    cloud_changed = true;
  }

  return cloud_changed;
}

sensor_msgs::PointCloud2 TrajectoryScans2PointcloudAlgNode::laser_scan_to_point_cloud(const sensor_msgs::LaserScan& LScan, const geometry_msgs::Pose& pose) 
{
  sensor_msgs::PointCloud2 pcloud;
  sensor_msgs::PointCloud2 transformed_pcloud;
  
  laser_projector_.projectLaser(LScan, pcloud, LScan.range_max - 0.0001);
  
  Matrix4f T_pose = transformation_matrix(pose.position.x, pose.position.y, pose.position.z, tf::getYaw(pose.orientation));
  
  pcl_ros::transformPointCloud(T_pose * T_laser_frame, pcloud, transformed_pcloud);

  return transformed_pcloud;
}

void TrajectoryScans2PointcloudAlgNode::add_to_PointCloud_msg(const sensor_msgs::PointCloud2& newPointCloud)
{
  //ROS_INFO("ADD to pointcloud");
  pcl::PointCloud<pcl::PointXYZ> newcloud;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(newPointCloud, *cloud2);
  
  if (emptyPointCloud_)
  {
    newcloud = *cloud2;
    emptyPointCloud_ = false;
  
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

void TrajectoryScans2PointcloudAlgNode::clear_PointCloud_msg()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(PointCloud_msg_, cloud);
  
  cloud.clear();
  
  pcl::toROSMsg(cloud, PointCloud_msg_);
}

Matrix4f TrajectoryScans2PointcloudAlgNode::transformation_matrix(const float x, const float y, const float z, const float alpha) const
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