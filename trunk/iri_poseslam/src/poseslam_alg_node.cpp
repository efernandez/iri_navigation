#include "poseslam_alg_node.h"

PoseslamAlgNode::PoseslamAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PoseslamAlgorithm>(),
  step_(0),
  ended(0)
{
  //init class attributes if necessary
  this->loop_rate_ = alg_.config_.loop_rate; //in [Hz]

  // [init publishers]
  this->trajectory_publisher_ = this->public_node_handle_.advertise<iri_poseslam::Trajectory>("trajectory", 10);
  if (alg_.config_.also_no_loops)
    this->no_loops_trajectory_publisher_ = this->public_node_handle_.advertise<iri_poseslam::Trajectory>("no_loops_trajectory", 10);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  get_link_client_ = this->public_node_handle_.serviceClient<iri_poseslam::GetLink>("get_link");
  
  // [init action servers]
  
  // [init action clients]
}

PoseslamAlgNode::~PoseslamAlgNode(void)
{
  // [free dynamic memory]
}

void PoseslamAlgNode::mainNodeThread(void)
{
  //ROS_INFO("POSE SLAM: step_ = %u",step_);

  bool LoopClosure = false;
  while (step_ == 0)
  {
    // ROS_INFO("POSE SLAM: Asking for the first Laser Scan header... step_ = %u", step_);
    // First Laser Scan Header
    get_link_srv_.request.current_step = 0;
    get_link_srv_.request.with_step = 0;
    if (get_link_client_.call(get_link_srv_))
    {
      if (get_link_srv_.response.success)
      {
        augment_trajectories(get_link_srv_.response.odom.header);
      	update_trajectories(false);
      	
      	this->trajectory_publisher_.publish(this->Trajectory_msg_);
      	if (alg_.config_.also_no_loops)
      	  this->no_loops_trajectory_publisher_.publish(this->Trajectory_msg_no_loops_);
      	
      	step_++;
      	
      	ROS_INFO("POSE SLAM: First laser scan header obtained");
      }
      //else
        //ROS_WARN("POSE SLAM: Coudn't get first laser scan header");
    }
    //else
      //ROS_WARN("POSE SLAM: Coudn't comunicate with get_link for the first laser scan header");
  }
  
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  
  // ---------- POSE SLAM ROUTINE -----------
  // Odometry reception
  get_link_srv_.request.current_step = step_ - 1;
  get_link_srv_.request.with_step = step_;
  
  alg_.tSens = ros::Time::now().toSec();
  //ROS_INFO("POSE SLAM: New pose odometry requested! step_ = %i - previous step_ = %i", step_, step_ - 1);
  if (get_link_client_.call(get_link_srv_))
  {
    if (get_link_srv_.response.success && !get_link_srv_.response.end)// && step_ <= 800) 
    {
    	// TIME VARIABLES
    	if (alg_.timePub.capacity() == alg_.timePub.size())
    	{
    	  alg_.timePub.reserve(alg_.timePub.size() + size_t(100));
    	  alg_.timeSensOpen.reserve(alg_.timeSensOpen.size() + size_t(100));
    	  alg_.timeSensClose.reserve(alg_.timeSensClose.size() + size_t(100));
    	}
    	alg_.timePub.push_back(0.0);
    	alg_.timeSensOpen.push_back(0.0);
    	alg_.timeSensClose.push_back(0.0);
          
    	alg_.timeSensOpen.back() = alg_.timeSensOpen.back() + ros::Time::now().toSec() - alg_.tSens;
    	
    	// State Augmentation
    	alg_.augmentation(step_, get_link_srv_.response.odom);
    	
      // Augment trajectory
      augment_trajectories(get_link_srv_.response.odom.header);

    	// Create list of link candidates
    	alg_.create_candidates_list(true);
    	
    	// Process all link candidates for loop closure
    	while (alg_.any_candidate() && !alg_.is_redundant())
    	{
    	  // Select the most information gain link candidate
    	  alg_.select_best_candidate();
    	  //ROS_INFO("CANDIDATE: %i with step_ %i", step_, alg_.get_candidate_step());
    	  
    	  // Check if the present pose is redundant
    	  alg_.redundant_evaluation();

        // Check if we can try loop closure
    	  if (alg_.loop_closure_requeriments())
    	  {
    	    get_link_srv_.request.current_step = step_;
    	    get_link_srv_.request.with_step = alg_.get_candidate_step();
    	    get_link_srv_.request.prior_d = alg_.get_candidate_d();
    	    
          ROS_DEBUG("POSE SLAM: trying to close a loop: actual step_: %i with step_: %i", int(get_link_srv_.request.current_step), int(get_link_srv_.request.with_step));
    	    
    	    alg_.tSens = ros::Time::now().toSec();
    	    if (get_link_client_.call(get_link_srv_))
    	      LoopClosure = get_link_srv_.response.success;
    	    else
    	      ROS_ERROR("POSE SLAM: Communication with sensors_2_link failed");
    	    alg_.timeSensClose.back() = alg_.timeSensClose.back() + ros::Time::now().toSec() - alg_.tSens;
    	
    	    if (LoopClosure) //If the sensor(s) confirms the loop closure..
    	      LoopClosure = alg_.try_loop_closure(get_link_srv_.response.odom);
    	    
    	    if (LoopClosure)
    	    {
            ROS_INFO("POSE SLAM: LOOP CLOSED! current step_: %i with step_: %i", step_, alg_.get_candidate_step());
    	      loops_.push_back(alg_.get_candidate_step());
    	      loops_.push_back(step_);
    	    }
    	  }
    	  alg_.update_candidates_list(LoopClosure, true);
    	}
    	
    	if (!alg_.is_redundant()) 
    	  ROS_DEBUG("POSE SLAM: NEW STATE - step_: %i - IG: %f", step_, alg_.get_candidate_ig());
    	
    	// Trajectories publishing messages
    	alg_.tPub = ros::Time::now().toSec();
    	update_trajectories(LoopClosure);
    	//ROS_INFO("Trajectories updated");
    	
    	this->trajectory_publisher_.publish(this->Trajectory_msg_);
    	if (alg_.config_.also_no_loops)
    	  this->no_loops_trajectory_publisher_.publish(this->Trajectory_msg_no_loops_);
    	alg_.timePub.back() = alg_.timePub.back() + ros::Time::now().toSec() - alg_.tPub;
    		
    	step_++;
    	ended = 0;
    }
    else if (step_ != 1 && get_link_srv_.response.end) //no more odometries and not before starting simulation
    	ended++;
  }
  else 
    ROS_ERROR("POSE SLAM: Failed to Call Server (service get_link) in next odom");
  
  if (ended == 20) //20 consecutive steps without new odometries --> END SIMULATION 
  {
    ROS_INFO("POSE SLAM: SIMULATION ENDED");
    
    print_results();
    ended ++;
  }
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PoseslamAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
    loop_rate_ = config.loop_rate; //in [Hz]
    dz_footprint_2_base_ = config.dz_footprint_2_base;
  this->alg_.unlock();
}

void PoseslamAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PoseslamAlgNode>(argc, argv, "poseslam_alg_node");
}

void PoseslamAlgNode::augment_trajectories(const std_msgs::Header& header)
{  
  //ROS_INFO("POSE SLAM: Augment trajectories");
  
  // HEADER
  Trajectory_msg_.header.seq = step_;
  Trajectory_msg_.header.stamp = ros::Time::now();
  Trajectory_msg_.header.frame_id = "/world";
  
  if (alg_.config_.also_no_loops)
  {
    Trajectory_msg_no_loops_.header.seq = step_;
    Trajectory_msg_no_loops_.header.stamp = ros::Time::now();
    Trajectory_msg_no_loops_.header.frame_id = "/world";
  }
    
  // STEPS AND STATES suposed redundant
  Trajectory_msg_.steps_2_states.push_back(-1);
  if (alg_.config_.also_no_loops)
    Trajectory_msg_no_loops_.steps_2_states.push_back(-1);

  // LAST POSE
  Trajectory_msg_.poses.push_back(create_PoseWithCovarianceStamped(header, alg_.get_last_pose(), alg_.get_last_covariance()));
  Trajectory_poses_.push_back(alg_.get_last_pose());
  
  if (alg_.config_.also_no_loops)
    Trajectory_msg_no_loops_.poses.push_back(create_PoseWithCovarianceStamped(header, alg_.get_last_pose_no_loops(), alg_.get_last_covariance_no_loops()));
}

void PoseslamAlgNode::update_trajectories(const bool& LoopClosed)
{  
  //ROS_INFO("POSE SLAM: Update trajectories");
  
  // STEPS AND STATES CORRECTION
  if (!alg_.is_redundant())
  {
    Trajectory_msg_.states_2_steps.push_back(step_);
    Trajectory_msg_.steps_2_states.back() = Trajectory_msg_.states_2_steps.size() - 1;
    if (alg_.config_.also_no_loops)
    {
      Trajectory_msg_no_loops_.states_2_steps.push_back(step_);
      Trajectory_msg_no_loops_.steps_2_states.back() = Trajectory_msg_no_loops_.states_2_steps.size() - 1;
    }
    ROS_DEBUG("POSE SLAM: Update traj: STATE %u - step_ %u", uint(Trajectory_msg_.states_2_steps.size() - 1), step_);
  }

  // LOOP CLOSURE
  if (LoopClosed)
    recompute_trajectory();
}

void PoseslamAlgNode::print_results() const
{
  // Printing results in a Matlab File
  std::fstream file;
  file.open("/home/jvallve/poseSLAM_results.m",std::ios::out);
  if (file.is_open())
  {
    // Computational time
    this->alg_.print_time_file(file);
    
    file << "time_publishing";
    this->alg_.print_vector_in_file(this->alg_.timePub, file);
    file << "time_sensing_open";
    this->alg_.print_vector_in_file(this->alg_.timeSensOpen, file);
    file << "time_sensing_close";
    this->alg_.print_vector_in_file(this->alg_.timeSensClose, file);
    file << "states_2_steps";
    this->alg_.print_vector_in_file(alg_.get_trajectory_steps(), file);
    file << "loops";
    if (loops_.size() > 0)
      this->alg_.print_vector_in_file(loops_, file);
    else
      file << " = []";
    for (uint i=0; i<alg_.Q_aug_.size(); i++)
    {
      file << "Q_aug{" << i+1 << "}";
      this->alg_.print_matrix_in_file(alg_.Q_aug_.at(i), file);
    }
    for (uint i=0; i<alg_.Q_loop_.size(); i++)
    {
      file << "Q_loop{" << i+1 << "}";
      this->alg_.print_matrix_in_file(alg_.Q_loop_.at(i), file);
    }
    for (uint i=0; i<alg_.d_aug_.size(); i++)
    {
      file << "d_aug{" << i+1 << "}";
      this->alg_.print_matrix_in_file(alg_.d_aug_.at(i), file);
    }
    for (uint i=0; i<alg_.d_loop_.size(); i++)
    {
      file << "d_loop{" << i+1 << "}";
      this->alg_.print_matrix_in_file(alg_.d_loop_.at(i), file);
    }
    for (uint i=0; i<alg_.d_loop_.size(); i++)
    {
      file << "success_loop{" << i+1 << "}";
      if (alg_.success_loop_.at(i))
        file << "= 1;\n";
      else
        file << "= 0;\n";
    }
  }
}

void PoseslamAlgNode::recompute_trajectory()
{
  //ROS_INFO("POSE SLAM: Loop Closed --> Update trajectory...");

  // Get trajectory data
  std::vector<VectorXd> non_redundant_trajectory_poses = alg_.get_trajectory();
  std::vector<std::vector<double> > non_redundant_trajectory_cov = alg_.get_trajectory_covariance();

  std::vector<double> empty_cov (9,0);
  
  // Update new trajectory poses
  //ROS_INFO("POSE SLAM: non_redundant_trajectory_poses.size() = %i Trajectory_msg_.states_2_steps.size() = %i", non_redundant_trajectory_poses.size(), Trajectory_msg_.states_2_steps.size());
  for (uint i = 0; i < Trajectory_msg_.states_2_steps.size() - 1; i++)
  {
    uint initial_step = Trajectory_msg_.states_2_steps.at(i);
    uint last_step = Trajectory_msg_.states_2_steps.at(i + 1);
    
    // Recompute segment between non redundant poses
    std::vector<VectorXd> new_segment = recompute_segment(non_redundant_trajectory_poses.at(i), non_redundant_trajectory_poses.at(i + 1), initial_step, last_step);
    //ROS_INFO("POSE SLAM: Recomputed segment between %i and %i: %f, %f, %f and %f, %f, %f", i, i+1, non_redundant_trajectory_poses.at(i)(0),non_redundant_trajectory_poses.at(i)(1),non_redundant_trajectory_poses.at(i)(2), non_redundant_trajectory_poses.at(i + 1)(0), non_redundant_trajectory_poses.at(i + 1)(1), non_redundant_trajectory_poses.at(i + 1)(2));
  

    // Update the state i
    Trajectory_msg_.poses.at(initial_step).pose = create_PoseWithCovariance(non_redundant_trajectory_poses.at(i), non_redundant_trajectory_cov.at(i));
    //ROS_INFO("POSE SLAM: recomputed non-redundant step_: %i", initial_step);
    
    // Update the segment intermediate poses
    for (uint j = 0; j < new_segment.size(); j++)
    {
      //ROS_INFO("POSE SLAM: recomputed redundant step_: %i", initial_step + j + 1);
      Trajectory_msg_.poses.at(initial_step + j + 1).pose = create_PoseWithCovariance(new_segment.at(j), empty_cov);
      Trajectory_poses_.at(initial_step + j + 1) = new_segment.at(j);
    }
  }

  // Update last state
  Trajectory_msg_.poses.back().pose = create_PoseWithCovariance(non_redundant_trajectory_poses.back(), non_redundant_trajectory_cov.back());
  //ROS_INFO("POSE SLAM: Recomputed step_ %i: %f, %f, %f", Trajectory_msg_.poses.size() - 1, non_redundant_trajectory_poses.back()(0), non_redundant_trajectory_poses.back()(1),non_redundant_trajectory_poses.back()(2));
  
  //ROS_INFO("POSE SLAM: recomputed non-redundant step_: %i", Trajectory_msg_.poses.size() - 1);
    
  // Update loops indexs
  for (uint i = Trajectory_msg_.loops.size(); i < loops_.size(); i++)
    Trajectory_msg_.loops.push_back(loops_.at(i));
  
  // Store the trajectory non-redundant poses
  for (uint i = 0; i < Trajectory_msg_.states_2_steps.size(); i++)
    Trajectory_poses_.at(Trajectory_msg_.states_2_steps.at(i)) = non_redundant_trajectory_poses.at(i);
    
  }

std::vector<VectorXd> PoseslamAlgNode::recompute_segment(const VectorXd& new_initial_pose, const VectorXd& new_final_pose, const int& initial_step, const int& final_step)
{
  std::vector<VectorXd> segment_poses;

  // Old values
  std::vector<VectorXd> old_segment_poses;
  old_segment_poses.assign(Trajectory_poses_.begin() + initial_step + 1, Trajectory_poses_.begin() + final_step);

  VectorXd old_initial_pose = Trajectory_poses_.at(initial_step);
  VectorXd old_final_pose = Trajectory_poses_.at(final_step);
  
  VectorXd old_d = old_final_pose - old_initial_pose;
  Vector2d old_dxy = old_d.head(2);
  double old_dist = sqrt(old_dxy.dot(old_dxy));
  
  // New values
  VectorXd new_d = new_final_pose - new_initial_pose;
  Vector2d new_dxy = new_d.head(2);
  double new_dist = sqrt(new_dxy.dot(new_dxy));
  
  // Transformation
  double xy_scale = new_dist / old_dist;
  double th_scale = pi_2_pi(new_d(2)) / pi_2_pi(old_d(2));

  double rotation = angle_between_2D_vectors(old_dxy, new_dxy);
  MatrixXd R = rotation_matrix(rotation);

  for (uint i = 0; i < old_segment_poses.size(); i++)
  {
    // vector from initial pose
    VectorXd d_i = old_segment_poses.at(i) - old_initial_pose;
    
    // scale
    d_i.head(2) = xy_scale * d_i.head(2);
    d_i(2) = th_scale * pi_2_pi(d_i(2));
    
    // xy rotation
    d_i = R * d_i;
    
    // intermediate pose
    VectorXd pose_i = new_initial_pose + d_i;
    
    // store intermediate pose
    segment_poses.push_back(pose_i);
  }

  return segment_poses;
}

geometry_msgs::PoseWithCovarianceStamped PoseslamAlgNode::create_PoseWithCovarianceStamped(const std_msgs::Header& header, const VectorXd& last_pose, const std::vector<double>& last_cov)
{
  geometry_msgs::PoseWithCovarianceStamped new_pose;

  new_pose.header = header;
  new_pose.header.frame_id = "/world";
  new_pose.pose = create_PoseWithCovariance(last_pose, last_cov);

  return new_pose;
}

geometry_msgs::PoseWithCovariance PoseslamAlgNode::create_PoseWithCovariance(const VectorXd& last_pose, const std::vector<double>& last_cov)
{
  geometry_msgs::PoseWithCovariance new_pose;

  new_pose.pose.position.x = last_pose(0);
  new_pose.pose.position.y = last_pose(1);
  new_pose.pose.position.z = dz_footprint_2_base_;
  new_pose.pose.orientation = tf::createQuaternionMsgFromYaw(last_pose(2));
  
  for (uint j = 0; j < 9; j++)
    new_pose.covariance.at(j) = last_cov.at(j); 

  return new_pose;
}

MatrixXd PoseslamAlgNode::rotation_matrix(const double &alpha) const
{
  MatrixXd rot = MatrixXd::Identity(3,3);
  
  rot(0,0) = cos(alpha);
  rot(0,1) = -sin(alpha);
  rot(1,0) = sin(alpha);
  rot(1,1) = cos(alpha);

  return rot;
}

double PoseslamAlgNode::angle_between_2D_vectors(const Vector2d& v, const Vector2d& w) const
{
  Vector3d v3D = MatrixXd::Zero(3,1);
  Vector3d w3D = MatrixXd::Zero(3,1);

  v3D.head(2) = v;
  w3D.head(2) = w;

  Vector3d cross_prod = v3D.cross(w3D);

  double v_norm = sqrt(v.dot(v));
  double w_norm = sqrt(w.dot(w));
  double cross_norm = sqrt(cross_prod.dot(cross_prod));
  double sign = 1;
  if (cross_prod(2) < 0)
    sign = -1;

  return pi_2_pi(asin(sign * cross_norm / (v_norm * w_norm)));
}

double PoseslamAlgNode::pi_2_pi(const double& angle) const
{
  return angle - 2 * M_PI * floor((angle + M_PI)/(2 * M_PI));
}