#include "trajectory_scans_2_occgrid_alg_node.h"

TrajectoryScans2OccGridAlgNode::TrajectoryScans2OccGridAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TrajectoryScans2OccGridAlgorithm>()
{
  //init class attributes if necessary
  new_trajectory_ = false;
  last_step_ = 0;
  
  public_node_handle_.param<int>("max_n_cells", max_n_cells_, 10000);
  public_node_handle_.param<bool>("publish_redundant", publish_redundant_, true);
  public_node_handle_.param<double>("grid_size", grid_size_, 0.5);
  public_node_handle_.param<double>("laser_ray_incr", laser_ray_incr_, 0.05);
  public_node_handle_.param<int>("Lfree", Lfree_, -10);
  public_node_handle_.param<int>("Lobs", Lobs_, 100);
  public_node_handle_.param<int>("n_cells_x", n_cells_(0), 100);
  public_node_handle_.param<int>("n_cells_y", n_cells_(1), 100);  

  map_origin_(0) = -n_cells_(0) * grid_size_ / 2;
  map_origin_(1) = -n_cells_(1) * grid_size_ / 2;
  resize_OccupancyGrid();
  std::vector<int8_t> init_occ_grid(n_cells_(0) * n_cells_(1), 50);
  occupancy_grid_.data = init_occ_grid;
  logodds_grid_ = Array<double, Dynamic, Dynamic, ColMajor>::Zero(n_cells_(0),n_cells_(1));
  occupancy_grid_.header.frame_id = "/map";
  
  double d[4];
  public_node_handle_.param<double>("dx_base_2_laser", d[0], 0.57);
  public_node_handle_.param<double>("dy_base_2_laser", d[1], 0.0);
  public_node_handle_.param<double>("dz_base_2_laser", d[2], 0.145);
  public_node_handle_.param<double>("dth_base_2_laser", d[3], 0.0);
  T_laser_frame_ = transformation_matrix(d[0], d[1], d[2], d[3]);

  ROS_DEBUG("TR 2 OCCGRID: Config updated");

  this->loop_rate_ = 10;//in [Hz]

  // [init publishers]
  this->occgrid_publisher_ = this->public_node_handle_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
  
  // [init subscribers]
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 10, &TrajectoryScans2OccGridAlgNode::scan_callback, this);
  this->trajectory_subscriber_ = this->public_node_handle_.subscribe("trajectory", 1, &TrajectoryScans2OccGridAlgNode::trajectory_callback, this);
  
  pthread_mutex_init(&this->last_trajectory_mutex_,NULL);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TrajectoryScans2OccGridAlgNode::~TrajectoryScans2OccGridAlgNode(void)
{
  // [free Dynamic memory]
}

void TrajectoryScans2OccGridAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  if (new_trajectory_)
  {
    last_trajectory_mutex_enter();
    const iri_poseslam::Trajectory trajectory = last_trajectory_;
    new_trajectory_ = false;
    last_trajectory_mutex_exit();

    // Update the trajecory scans buffer
    update_trajectory_scans(trajectory);
  
    // Recompute occupancy grid
    recompute_occupancy_grid(trajectory);
  }
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->occgrid_publisher_.publish(this->occupancy_grid_);
  ROS_DEBUG("TR 2 OCCGRID: occupancy_grid_ published!");
}

/*  [subscriber callbacks] */
void TrajectoryScans2OccGridAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laser_scan_buffer_.push(*msg);
}

void TrajectoryScans2OccGridAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  last_trajectory_mutex_enter();
  if (publish_redundant_ || msg->steps_2_states.back() != -1)
  {
    last_trajectory_ = *msg;
    new_trajectory_ = true;
  }
  last_trajectory_mutex_exit();
}

void TrajectoryScans2OccGridAlgNode::last_trajectory_mutex_enter(void) 
{ 
  pthread_mutex_lock(&this->last_trajectory_mutex_); 
} 

void TrajectoryScans2OccGridAlgNode::last_trajectory_mutex_exit(void) 
{ 
  pthread_mutex_unlock(&this->last_trajectory_mutex_); 
}


/*  [service callbacks] */
/*  [action callbacks] */
/*  [action requests] */

void TrajectoryScans2OccGridAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();  
  this->alg_.unlock();
}

void TrajectoryScans2OccGridAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TrajectoryScans2OccGridAlgNode>(argc, argv, "trajectory_scans_2_occgrid_alg_node");
}

void TrajectoryScans2OccGridAlgNode::update_trajectory_scans(const iri_poseslam::Trajectory& trajectory)
{
  // Reserve space in buffer
  if (trajectory_scans_.size() == trajectory_scans_.capacity())
    trajectory_scans_.reserve(trajectory_scans_.size() + 100);

  // Add the lasers of the new trajectory poses
  uint N_poses = (publish_redundant_ ? trajectory.poses.size() : trajectory.states_2_steps.size());

  ROS_DEBUG_STREAM("TR 2 OCCGRID: update traj scans: N_poses = "<< N_poses <<" - trajectory_scans_.size() = " << trajectory_scans_.size());

  while (N_poses > trajectory_scans_.size())
  {
    uint step_idx = ( publish_redundant_ ? trajectory_scans_.size() :  trajectory.states_2_steps.at(trajectory_scans_.size()) );
    if (laser_scan_buffer_.size() > 0 && laser_scan_buffer_.front().header.stamp == trajectory.poses.at(step_idx).header.stamp)
    {
      trajectory_scans_.push_back(laser_scan_buffer_.front());
      laser_scan_buffer_.pop();
      //ROS_INFO("TR 2 OCCGRID: laser scan added! \n\tnSteps = %i \n\tnStates = %i \n\ttrajectory_scans_.size = %i", trajectory.steps_2_states.size(), trajectory.states_2_steps.size(), trajectory_scans_.size());
    }
    else if (laser_scan_buffer_.size() > 0 && laser_scan_buffer_.front().header.stamp < trajectory.poses.at(step_idx).header.stamp)
      laser_scan_buffer_.pop();
    else
    {
      ROS_WARN("TR 2 OCCGRID: Any trajectory pose laser scan lost");
      sensor_msgs::LaserScan empty_LaserScan;
      empty_LaserScan.header = trajectory.poses.at(step_idx).header;
      trajectory_scans_.push_back(empty_LaserScan);
    }
  }
}

void TrajectoryScans2OccGridAlgNode::recompute_occupancy_grid(const iri_poseslam::Trajectory& trajectory)
{
  bool LoopClosed = false;
  LoopClosed = (trajectory.loops.size() > last_loops_);
  last_loops_ = trajectory.loops.size();
  uint from_step;

  // LOOP CLOSED: Recompute logodds_grid_ from zero
  if (LoopClosed)
  {
    // Reset logodds_grid_
    logodds_grid_ = Array<double, Dynamic, Dynamic, ColMajor>::Zero(n_cells_(0),n_cells_(1));
    from_step = 0;
    ROS_DEBUG("TR 2 OCCGRID: Loop Closed! Recompute all %u poses of the trajectory with %u laser scans", uint(trajectory.poses.size()), uint(trajectory_scans_.size()));
  }

  // NOT LOOP CLOSED: Add the last scans to logodds_grid_
  else
    from_step = last_step_ + 1;

  // Update the logodds_grid_
  for (uint i = from_step; i < trajectory.poses.size(); i++)
  {
    //ROS_INFO("TR 2 OCCGRID: scan step: %i\ntrajectory.poses.size() = %u\ntrajectory.steps_2_states.size() = %u", i, trajectory.poses.size(), trajectory.steps_2_states.size());
    if (publish_redundant_ || trajectory.steps_2_states.at(i) != -1)
    {
      ROS_DEBUG("TR 2 OCCGRID: Adding scan from pose: %u - %f, %f, %f", i,
                           trajectory.poses.at(i).pose.pose.position.x, 
                           trajectory.poses.at(i).pose.pose.position.y, 
                           tf::getYaw(trajectory.poses.at(i).pose.pose.orientation));
      sensor_msgs::LaserScan laser_scan = ( publish_redundant_ ? trajectory_scans_.at(i) : trajectory_scans_.at(trajectory.steps_2_states.at(i)) );
      if (laser_scan.header.stamp == trajectory.poses.at(i).header.stamp)
        add_scan_to_logodds(laser_scan, trajectory.poses.at(i).pose.pose);
      else
        ROS_ERROR("TR 2 OCCGRID: headers don't match! step %i", i);
    }
  }
  
  update_occupancy_grid();
  last_step_ = trajectory.poses.size() - 1;
}

void TrajectoryScans2OccGridAlgNode::add_scan_to_logodds(const sensor_msgs::LaserScan& LScan, const geometry_msgs::Pose& pose) 
{
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z) || !std::isfinite(tf::getYaw(pose.orientation)))
  {
    ROS_WARN("TR 2 OCCGRID: Not finite pose! \n\tpose = [%f, %f, %f] \n\tYaw = %f", pose.position.x, pose.position.y, pose.position.z, tf::getYaw(pose.orientation));
  }
  else
  {
    // Compute the log odds update
    double theta = LScan.angle_min;

    for (uint i=0; i < LScan.ranges.size(); i++)
    {
      if (theta > LScan.angle_max + 1e-3)
        ROS_ERROR("TR 2 OCCGRID: Orientation of ray %f higher than angle_max %f", theta, LScan.angle_max);
      else if (LScan.ranges[i] < LScan.range_max)
        // Add the logodds ray
        add_ray_2_logodds(theta, LScan.ranges[i], pose);
      
      theta+=LScan.angle_increment;
    }
  }
}

void TrajectoryScans2OccGridAlgNode::add_ray_2_logodds(const double& theta, const double& range, const geometry_msgs::Pose& pose)
{
  double d = 0;
  double orientation = theta + tf::getYaw(pose.orientation);
  Vector2f origin, point, displ;

  Matrix4f T_origin_pose = transformation_matrix(pose.position.x, pose.position.y, pose.position.z, tf::getYaw(pose.orientation)) * T_laser_frame_;
  origin = T_origin_pose.block(0,3,2,1);
  
  displ(0) = cos(orientation) * laser_ray_incr_;
  displ(1) = sin(orientation) * laser_ray_incr_;
  point = origin;// + displ;

  if (!std::isfinite(point(0)) || !std::isfinite(point(1)))
  {
    //ROS_WARN("TR 2 OCCGRID: Not finite point! \n\tpoint = [%f, %f] \n\torigin = [%f, %f]  \n\tdispl = [%f, %f] ", point(0), point(1), origin(0), origin(1), displ(0), displ(1));
  }
  else
  {  
    // free cells log odds
    Vector2i cell_prev = MatrixXi::Constant(2,1,-1);

    while (d < range)
    {
      Vector2i cell = vector2cell(point);

      if (cell(0) < 0 || cell(1) < 0 || cell(0) >= n_cells_(0) || cell(1) >= n_cells_(1) )
        ROS_DEBUG("TR 2 OCCGRID: Cell out of the map! \n\tpoint = %f, %f\n\tlower = %f, %f\n\tupper = %f, %f\n\tcell = %i, %i\n\tn cells = %i, %i",
                           point(0), point(1), map_origin_(0), map_origin_(1), map_origin_(0) + grid_size_ * n_cells_(1), map_origin_(1) + grid_size_ * n_cells_(0),
                           cell(1), cell(0), n_cells_(1), n_cells_(0));
      else if (cell != cell_prev)
        logodds_grid_(cell(0), cell(1)) += Lfree_;
      
      cell_prev = cell;

      // next point
      point += displ;
      d += laser_ray_incr_;
    }

    // obstacle cell log odd
    point(0) = origin(0) + cos(orientation) * range;
    point(1) = origin(1) + sin(orientation) * range;

    Vector2i cell = vector2cell(point);
    
    if (cell(0) < 0 || cell(1) < 0 || cell(0) >= n_cells_(0) || cell(1) >= n_cells_(1) )
      ROS_ERROR("TR 2 OCCGRID: Cell out of the map! \n\tpoint = %f, %f\n\tlower = %f, %f\n\tupper = %f, %f\n\tcell = %i, %i\n\tn cells = %i, %i",
                         point(0), point(1), map_origin_(0), map_origin_(1), map_origin_(0) + grid_size_ * n_cells_(1), map_origin_(1) + grid_size_ * n_cells_(0),
                         cell(1), cell(0), n_cells_(1), n_cells_(0));
    else
      logodds_grid_(cell(0), cell(1))  += Lobs_;
  }
}

Vector2i TrajectoryScans2OccGridAlgNode::vector2cell(const Vector2f& p)
{
  Vector2i cell;
  cell(0) = int( floor((p(0) - map_origin_(0)) / grid_size_ ));
  cell(1) = int( floor((p(1) - map_origin_(1)) / grid_size_ ));
  
  for (uint i = 0; i<2; i++)
  {
    int oversize = std::max( 0 - cell(i), cell(i) - n_cells_(i) + 1 );
    while ( oversize > 0 && n_cells_(i) < max_n_cells_ )
    {
      bool back = !(cell(i) < 0);
      ROS_DEBUG("TR 2 OCCGRID: Point out of the map in dimension %i: \n\tpoint = %f, %f\n\tlower = %f, %f\n\tupper = %f, %f\n\tcell = %i, %i\n\tn cells = %i, %i", i,
                         p(0), p(1), map_origin_(0), map_origin_(1), map_origin_(0) + grid_size_ * n_cells_(1), map_origin_(1) + grid_size_ * n_cells_(0),
                         cell(1), cell(0), n_cells_(1), n_cells_(0));
      resize_map(i, oversize, back); 
      ROS_DEBUG("TR 2 OCCGRID: Map resized in dimension %i: new limits:\n\tlower = %f, %f\n\tupper = %f, %f\n\tn cells = %i, %i", i,
                         map_origin_(0), map_origin_(1), map_origin_(0) + grid_size_ * n_cells_(1), map_origin_(1) + grid_size_ * n_cells_(0), n_cells_(1), n_cells_(0));
      
      // Recompute current cell and oversize
      cell(i) = int( (p(i) - map_origin_(i)) / grid_size_ );
      oversize = std::max( 0 - cell(i), cell(i) - n_cells_(i) + 1 );
    }
  }
  return cell;
}

void TrajectoryScans2OccGridAlgNode::update_occupancy_grid()
{
  Array<double, Dynamic, Dynamic, ColMajor> occupancy_probability(n_cells_(0), n_cells_(1));
  occupancy_probability = 100 * (1 - (1 + logodds_grid_.exp()).cwiseInverse());

  std::copy(occupancy_probability.data(), occupancy_probability.data() + occupancy_probability.size(), occupancy_grid_.data.begin());
  //ROS_INFO("TR 2 OCCGRID: OccupancyGrid updated! occupancy_probability.size() = %i - occupancy_grid_.data.size() = %i",occupancy_probability.size(),occupancy_grid_.data.size());

  occupancy_grid_.header.stamp = ros::Time::now();
  occupancy_grid_.info.map_load_time = ros::Time::now();
}

void TrajectoryScans2OccGridAlgNode::resize_OccupancyGrid()
{
  // initial pose in the middle of the map
  geometry_msgs::Pose origin_pose;
  origin_pose.position.x = map_origin_(0);
  origin_pose.position.y = map_origin_(1);

  // Occupancy grid meta data Initializiation
  occupancy_grid_.info.resolution = grid_size_;
  occupancy_grid_.info.width = n_cells_(0);
  occupancy_grid_.info.height = n_cells_(1);
  occupancy_grid_.info.origin = origin_pose;

  // resize data
  occupancy_grid_.data.resize(logodds_grid_.size());
}

void TrajectoryScans2OccGridAlgNode::resize_map(const int& dim, const uint& oversize, const bool& back)
{
  Vector2i new_n_cells = n_cells_;  
  new_n_cells(dim) = n_cells_(dim) + oversize;

  // at the end of the array
  if (back)
  {
    // Origin doesn't change
    // Array resize
    Array<double, Dynamic, Dynamic, ColMajor> new_logodds_grid = Array<double, Dynamic, Dynamic, ColMajor>::Zero(new_n_cells(0), new_n_cells(1));
    new_logodds_grid.block(0, 0, n_cells_(0), n_cells_(1)) = logodds_grid_;
    logodds_grid_ = new_logodds_grid;
  }
  // at the beggining of the array
  else
  {
    // Origin
    map_origin_(dim) -= oversize * grid_size_;
    
    // Array resize
    Array<double, Dynamic, Dynamic, ColMajor> new_logodds_grid = Array<double, Dynamic, Dynamic, ColMajor>::Zero(new_n_cells(0), new_n_cells(1));
    Vector2i place = Vector2i::Zero(2);
    place(dim) = oversize;
    new_logodds_grid.block(place(0), place(1), n_cells_(0), n_cells_(1)) = logodds_grid_;
    logodds_grid_ = new_logodds_grid;
  }
  // Change in the occupancy grid msg
  n_cells_ = new_n_cells;
  resize_OccupancyGrid();
}

Matrix4f TrajectoryScans2OccGridAlgNode::transformation_matrix(const float x, const float y, const float z, const float alpha) const
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
