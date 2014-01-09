#include "poseslam_alg.h"

PoseslamAlgorithm::PoseslamAlgorithm(void)
  :
  inicialitzat_(false)
{ 
}

PoseslamAlgorithm::~PoseslamAlgorithm(void)
{
  delete pose_SLAM_;
  delete pose_SLAM_noLoops_;
}

void PoseslamAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  // Parameters	
  Params Parameters;
  Parameters.matchArea(0) = config_.match_area_x;
  Parameters.matchArea(1) = config_.match_area_y;
  Parameters.matchArea(2) = config_.match_area_th;
  Parameters.pdRange.first = config_.pd_range_1;   // Probability threshold of 2 poses of being closer than 'matchArea' for trying to create a loop.
  Parameters.pdRange.second = config_.pd_range_2;  // Probability threshold of 2 poses of being closer than 'matchArea' for one of them to be redundant.
  Parameters.igRange.first = config_.ig_range_1;   // Information gain threshold for a pose to be redundant	
  Parameters.igRange.second = config_.ig_range_2;  // Information gain threshold for a try loop closure
  Parameters.ignorePrevious = config_.ignore_previous_steps;  // Number of previous states to ignore on loop closure
  
  MatrixXd LoopNoise(3, 3);
  LoopNoise(0, 0) = config_.closing_loop_noise_xx * config_.ICP_covariance_correction_factor;
  LoopNoise(0, 1) = config_.closing_loop_noise_xy * config_.ICP_covariance_correction_factor;
  LoopNoise(1, 0) = config_.closing_loop_noise_xy * config_.ICP_covariance_correction_factor;
  LoopNoise(0, 2) = config_.closing_loop_noise_xth * config_.ICP_covariance_correction_factor;
  LoopNoise(2, 0) = config_.closing_loop_noise_xth * config_.ICP_covariance_correction_factor;
  LoopNoise(1, 1) = config_.closing_loop_noise_yy * config_.ICP_covariance_correction_factor;
  LoopNoise(1, 2) = config_.closing_loop_noise_yth * config_.ICP_covariance_correction_factor;
  LoopNoise(2, 1) = config_.closing_loop_noise_yth * config_.ICP_covariance_correction_factor;
  LoopNoise(2, 2) = config_.closing_loop_noise_thth * config_.ICP_covariance_correction_factor;
  Parameters.LoopNoise = LoopNoise;
  
  if (!inicialitzat_)
  {
    // Initial position
    Vector3d initM;
    initM(0) = config_.initial_position_x;
    initM(1) = config_.initial_position_y;
    initM(2) = config_.initial_position_th;
    
    // Initial covariance
    MatrixXd initS(3, 3);
    initS(0, 0) = config_.initial_covariance_xx;
    initS(0, 1) = config_.initial_covariance_xy;
    initS(1, 0) = config_.initial_covariance_xy;
    initS(0, 2) = config_.initial_covariance_xth;
    initS(2, 0) = config_.initial_covariance_xth;
    initS(1, 1) = config_.initial_covariance_yy;
    initS(1, 2) = config_.initial_covariance_yth;
    initS(2, 1) = config_.initial_covariance_yth;
    initS(2, 2) = config_.initial_covariance_thth;
    
    pose_SLAM_ = new CPoseSLAM(initM, initS, Parameters);
    if (config_.also_no_loops)
      pose_SLAM_noLoops_ = new CPoseSLAM(initM, initS, Parameters);
    
    Q_odom_ = Eigen::MatrixXd::Zero(3,3);
    
    inicialitzat_ = true;
    ROS_INFO("POSE SLAM: ----------- INICIALITZAT ------------");
  }
  else
  {
    pose_SLAM_->set_parameters(Parameters);
    if (config_.also_no_loops)
      pose_SLAM_noLoops_->set_parameters(Parameters);
  }

  this->unlock();
}

// PoseslamAlgorithm Public API
void PoseslamAlgorithm::augmentation(const uint& step, const geometry_msgs::PoseWithCovarianceStamped& odom)
{  

  Q_odom_(0,0) = odom.pose.covariance[0];
  Q_odom_(0,1) = odom.pose.covariance[1];
  Q_odom_(0,2) = odom.pose.covariance[5];
  Q_odom_(1,0) = odom.pose.covariance[6];
  Q_odom_(1,1) = odom.pose.covariance[7];
  Q_odom_(1,2) = odom.pose.covariance[11];
  Q_odom_(2,0) = odom.pose.covariance[30];
  Q_odom_(2,1) = odom.pose.covariance[31];
  Q_odom_(2,2) = odom.pose.covariance[35];
  
  Vector3d d;
  d(0) = odom.pose.pose.position.x;
  d(1) = odom.pose.pose.position.y;
  d(2) = tf::getYaw(odom.pose.pose.orientation);
  
  Q_aug_.push_back(Q_odom_);
  d_aug_.push_back(d);
  
  // covariance correction
  //Q_odom_ = Q_odom_ * config_.overnoise_augmentation_factor;
  //Q_odom_(2, 2) = Q_odom_(2, 2) * config_.overnoise_augmentation_factor;

  if (config_.also_no_loops)
    {
      pose_SLAM_noLoops_->set_redundant(pose_SLAM_->is_redundant());
      pose_SLAM_noLoops_->augmentation(step, d, Q_odom_);
    }
    pose_SLAM_->augmentation(step, d, Q_odom_);

  // Zero motion link
  pose_SLAM_->set_redundant(d.isZero() && Q_odom_.isZero());
}

void PoseslamAlgorithm::create_candidates_list(const bool& realOdometryCov)
{
  if (realOdometryCov)
    pose_SLAM_->create_candidates_list(Q_odom_);
  else
    pose_SLAM_->create_candidates_list();
}

void PoseslamAlgorithm::redundant_evaluation()
{
  pose_SLAM_->redundant_evaluation();
}

bool PoseslamAlgorithm::loop_closure_requeriments() const
{
  return pose_SLAM_->loop_closure_requeriments();
}

bool PoseslamAlgorithm::try_loop_closure(const geometry_msgs::PoseWithCovarianceStamped& odom)
{
  MatrixXd Q(3, 3);

  Q(0,0) = odom.pose.covariance[0];
  Q(0,1) = odom.pose.covariance[1];
  Q(0,2) = odom.pose.covariance[5];
  Q(1,0) = odom.pose.covariance[6];
  Q(1,1) = odom.pose.covariance[7];
  Q(1,2) = odom.pose.covariance[11];
  Q(2,0) = odom.pose.covariance[30];
  Q(2,1) = odom.pose.covariance[31];
  Q(2,2) = odom.pose.covariance[35];
  
  Vector3d d;
  d(0) = odom.pose.pose.position.x;
  d(1) = odom.pose.pose.position.y;
  d(2) = tf::getYaw(odom.pose.pose.orientation);
  
  Q_loop_.push_back(Q);
  d_loop_.push_back(d);
  
  // covariance correction
  //Q = Q * config_.overnoise_augmentation_factor;
  //Q(2, 2) = Q(2, 2) * config_.overnoise_augmentation_factor;
  
  bool success = pose_SLAM_->try_loop_closure(Q, d);
  
  success_loop_.push_back(success);
    
  return success;
}

void PoseslamAlgorithm::update_candidates_list(const bool LoopClosure, const bool& realOdometryCov)
{
  if (realOdometryCov)
    pose_SLAM_->update_candidates_list(LoopClosure, Q_odom_);
  else
    pose_SLAM_->update_candidates_list(LoopClosure);
}

bool PoseslamAlgorithm::any_candidate() const
{
  return pose_SLAM_->any_candidate();
}

void PoseslamAlgorithm::select_best_candidate()
{
  pose_SLAM_->select_best_candidate();
}

uint PoseslamAlgorithm::get_candidate_step() const
{
  return pose_SLAM_->get_candidate_step();
}

geometry_msgs::Pose PoseslamAlgorithm::get_candidate_d() const
{
  geometry_msgs::Pose disp;
  VectorXd d = pose_SLAM_->get_candidate_link().d;
  
  disp.position.x = d(0);
  disp.position.y = d(1);
  disp.position.z = 0;
  
  disp.orientation = tf::createQuaternionMsgFromYaw(d(2));
  
  return disp;
}

double PoseslamAlgorithm::get_candidate_ig() const
{
  return pose_SLAM_->get_candidate_link().iGain;
}

bool PoseslamAlgorithm::is_redundant() const
{
  return pose_SLAM_->is_redundant();
}

std::vector<VectorXd> PoseslamAlgorithm::get_trajectory() const
 {
  return pose_SLAM_->get_trajectory();
}   
   
std::vector<std::vector<double> > PoseslamAlgorithm::get_trajectory_covariance() const
{
  std::vector<MatrixXd> covs = pose_SLAM_->get_trajectory_covariance();
  std::vector<std::vector<double> > trajectory_covariances(covs.size());
  
  uint cov_size = covs.at(0).size();
    
  for (uint k = 0; k < covs.size(); k++)
  {
    std::vector<double> covariance(cov_size);
  
    std::copy(covs.at(k).data(),covs.at(k).data() + cov_size, covariance.begin());
    trajectory_covariances.at(k) = covariance;
  }
  return trajectory_covariances;
}

std::vector<uint> PoseslamAlgorithm::get_trajectory_steps() const
{
  return pose_SLAM_->get_trajectory_steps();
}

std::vector<VectorXd> PoseslamAlgorithm::get_trajectory_no_loops() const
{
  return pose_SLAM_noLoops_->get_trajectory();
}   

std::vector<std::vector<double> > PoseslamAlgorithm::get_trajectory_covariance_no_loops() const
{
  std::vector<MatrixXd> covs = pose_SLAM_noLoops_->get_trajectory_covariance();
  std::vector<std::vector<double> > trajectory_covariances(covs.size());
  
  uint cov_size = covs.at(0).size();
  
  for (uint k = 0; k < covs.size(); k++)
  {
    std::vector<double> covariance(cov_size);
  
    std::copy(covs.at(k).data(),covs.at(k).data() + cov_size, covariance.begin());
    trajectory_covariances.at(k) = covariance;
  }
  return trajectory_covariances;
}

VectorXd PoseslamAlgorithm::get_last_pose() const
{
  return pose_SLAM_-> get_FF().get_PD().back().get_mean().get_vector();
}

std::vector<double> PoseslamAlgorithm::get_last_covariance() const
{
  MatrixXd last_covariance_matrix = pose_SLAM_-> get_FF().get_PD().back().get_S();
  std::vector<double> last_covariance(last_covariance_matrix.size());
  
  std::copy(last_covariance_matrix.data(), last_covariance_matrix.data() + last_covariance_matrix.size(), last_covariance.begin());
  return last_covariance;
}

uint PoseslamAlgorithm::get_last_step() const
{  
  return pose_SLAM_-> get_FF().get_PF().state_2_step(pose_SLAM_-> get_FF().get_nStates() - 1);
}

VectorXd PoseslamAlgorithm::get_last_pose_no_loops() const
{  
  return pose_SLAM_noLoops_-> get_FF().get_PD().back().get_mean().get_vector();
}

std::vector<double> PoseslamAlgorithm::get_last_covariance_no_loops() const
{
  MatrixXd last_covariance_matrix = pose_SLAM_noLoops_-> get_FF().get_PD().back().get_S();
  std::vector<double> last_covariance(last_covariance_matrix.size());
  
  std::copy(last_covariance_matrix.data(),last_covariance_matrix.data() + last_covariance_matrix.size(), last_covariance.begin());
  return last_covariance;
}

uint PoseslamAlgorithm::get_nStates() const
{
  return pose_SLAM_-> get_FF().get_nStates();
}


void PoseslamAlgorithm::print_matrix_in_file(const MatrixXd& M, std::fstream& file) const
{
  pose_SLAM_-> print_matrix_in_file(M, file);
}

void PoseslamAlgorithm::print_vector_in_file(const std::vector<double>& v, std::fstream& file) const
{
  pose_SLAM_-> print_vector_in_file(v, file);
}

void PoseslamAlgorithm::print_vector_in_file(const std::vector<uint>& v, std::fstream& file) const
{
  pose_SLAM_-> print_vector_in_file(v, file);
}

void PoseslamAlgorithm::print_time_file(std::fstream& file) const
{
  pose_SLAM_-> print_time_file(file);
}