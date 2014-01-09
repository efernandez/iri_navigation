#include "no_collision_alg.h"

#include<math.h>
#include<tf/tf.h>
#include<algorithm>

NoCollisionAlgorithm::NoCollisionAlgorithm(void)
{
  // node configuration attributes
  this->MIN_SAFE_DISTANCE_ = 0.4;  // [m]
  this->LASER_SAFE_DIST_   = 0.8;  // [m]
  this->VT_MAX_            = 0.7;  // [m/s]
  this->T_ACC_             = 0.5;  // [m/s2]
  this->DIST_TOL_          = 0.01; // [rad]

  this->MIN_SAFE_ANGLE_    = 10.0*M_PI/180.0;  // [rad]
  this->VR_MAX_            = 1.0;              // [rad/s]
  this->R_ACC_             = 30.0;              // [rad/s2]
  this->ANGLE_TOL_         = 0.1;              // [rad]

  this->OA_CONE_ANGLE_     = 100.0*M_PI/180.0; // [rad]
  this->MAX_ANGLE_TO_STOP_ = 60.0*M_PI/180.0;  // [rad]
  // translational parameters
  this->current_vt_        = 0.0;              // [m/s]
  this->target_vt_         = this->VT_MAX_;    // [m/s]
  this->desired_vt_        = this->VT_MAX_;    // [m/s]
  this->acc_dist_          = 0.0;              // [m]
  this->deacc_dist_        = 0.0;              // [m]
  this->target_dist_       = 0.0;              // [m]
  this->current_dist_      = 0.0;              // [m]
  this->vt_state_          = traj_idle;

  // rotational parameters
  this->current_vr_        = 0.0;              // [rad/s]
  this->target_vr_         = this->VR_MAX_;    // [rad/s]
  this->desired_vr_        = this->VR_MAX_;    // [rad/s]
  this->acc_angle_         = 0.0;              // [rad]
  this->deacc_angle_       = 0.0;              // [rad]
  this->target_angle_      = 0.0;              // rad]
  this->current_angle_     = 0.0;              // [rad]
  this->vr_state_          = traj_idle;
  this->heading            = 0.0;
}

NoCollisionAlgorithm::~NoCollisionAlgorithm(void)
{
}

void NoCollisionAlgorithm::fromCartesian2Polar(const geometry_msgs::Point & p, float & module, float & angle)
{
  module = sqrt( p.x*p.x + p.y*p.y );
  angle  = atan2( p.y, p.x );
}

void NoCollisionAlgorithm::fromPolar2Cartesian(const float & module,const float & angle,geometry_msgs::Point & p)
{
  p.x=module*cos(angle); 
  p.y=module*sin(angle);
}

void NoCollisionAlgorithm::updateTranslationTrajParams(void)
{
  float total_dist;

  // compute the tranlational parameters
  this->acc_dist_=fabs((this->desired_vt_-this->current_vt_)*(this->desired_vt_-this->current_vt_)/(2*this->T_ACC_)+this->current_vt_*(this->desired_vt_-this->current_vt_)/(this->T_ACC_));
  this->deacc_dist_=fabs(this->desired_vt_*this->desired_vt_/(2*this->T_ACC_));
  total_dist=this->acc_dist_+this->deacc_dist_;
  if(fabs(total_dist)>fabs(this->current_dist_))
    this->target_vt_=sqrt(this->T_ACC_*fabs(this->current_dist_)+this->current_vt_*this->current_vt_/2);
  else
    this->target_vt_=this->desired_vt_;
  ROS_INFO("Target translational speed: %f, Acceleration distance: %f, Deceleration distance: %f",this->target_vt_,this->acc_dist_,this->deacc_dist_);
}

void NoCollisionAlgorithm::updateRotationTrajParams(void)
{
  float total_angle;
  int dir=0;

  // compute the rotational parameters
  if(this->target_angle_>=0) dir=1;
  else dir=-1;
  this->acc_angle_=fabs((dir*this->desired_vr_-this->current_vr_)*(dir*this->desired_vr_-this->current_vr_)/(2*this->R_ACC_)+this->current_vr_*(dir*this->desired_vr_-this->current_vr_)/(this->R_ACC_));
  this->deacc_angle_=fabs(this->desired_vr_*this->desired_vr_/(2*this->R_ACC_));
  total_angle=this->acc_angle_+this->deacc_angle_;
  if(fabs(total_angle)>fabs(this->current_angle_))
    this->target_vr_=dir*sqrt(this->R_ACC_*fabs(this->current_angle_)+this->current_vr_*this->current_vr_/2);
  else
    this->target_vr_=dir*this->desired_vr_;
  ROS_INFO("Target rotational speed: %f, Acceleration angle: %f, Deceleration angle: %f",this->target_vr_,this->acc_angle_,this->deacc_angle_);
}

void NoCollisionAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
    // save the current configuration
    MIN_SAFE_DISTANCE_ = new_cfg.min_safety_dist;
    MIN_SAFE_ANGLE_    = new_cfg.min_safety_angle*M_PI/180.0;
    LASER_SAFE_DIST_   = new_cfg.laser_safe_dist;
    VT_MAX_            = new_cfg.vT_max;
    VR_MAX_            = new_cfg.vR_max;
    R_ACC_             = new_cfg.r_acc;
    T_ACC_             = new_cfg.t_acc;
    OA_CONE_ANGLE_     = new_cfg.oa_cone_aperture*M_PI/180.0;
    MAX_ANGLE_TO_STOP_ = new_cfg.max_angle2stop*M_PI/180.0;

    this->config_=new_cfg;
  this->unlock();
}

// NoCollisionAlgorithm Public API
void NoCollisionAlgorithm::setGoal(const geometry_msgs::Pose & local_goal)
{
  this->lock();
  ROS_WARN("NoCollisionAlgorithm::setGoal");

  // get the new target angle and distance
  fromCartesian2Polar(local_goal.position,this->target_dist_,this->target_angle_);
  // remove the safety distance 
  this->target_dist_-=LASER_SAFE_DIST_+MIN_SAFE_DISTANCE_;
  if(this->target_dist_<0) this->target_dist_=0;
  this->current_dist_=this->target_dist_;
  this->current_angle_=this->target_angle_;
  this->target_vt_=this->desired_vt_;
  this->target_vr_=this->desired_vr_;
  // remove the target angle from the desired heading
  this->heading=local_goal.orientation.z-this->target_angle_;
  ROS_INFO("NoCollisionAlgorithm::Distance: %f Angle: %f",this->target_dist_,this->target_angle_);
  this->updateTranslationTrajParams();
  this->updateRotationTrajParams();
  // set the initial time
  this->current_time_ = ros::Time::now();
  this->unlock();
}

bool NoCollisionAlgorithm::isGoalReached()
{
  if(this->vt_state_>=traj_deacc && this->vr_state_>=traj_deacc)
    return true;
  else
    return false;
}

void NoCollisionAlgorithm::setTranslationalSpeed(const double vt)
{
  double speed;

  this->lock();
  if(vt>this->VT_MAX_)
    speed=this->VT_MAX_;
  else
    speed=vt;

  this->desired_vt_=speed;
  // update the current trajectory parameters
  if(this->vt_state_!=traj_deacc)
  {
    this->updateTranslationTrajParams();
    this->updateRotationTrajParams();
  }
  this->unlock();
}

void NoCollisionAlgorithm::setRotationalSpeed(const double vr)
{
  double speed;

  this->lock();
  if(vr>this->VR_MAX_)
    speed=this->VR_MAX_;
  else
    speed=vr;

  this->desired_vr_=speed;
  // update the current trajectory parameters
  if(this->vr_state_!=traj_deacc)
  {
    this->updateTranslationTrajParams();
    this->updateRotationTrajParams();
  }
  this->unlock();
}

void NoCollisionAlgorithm::getDistance2Goal(geometry_msgs::Pose &current_goal)
{
  fromPolar2Cartesian(fabs(this->target_dist_-this->current_dist_),this->target_angle_-this->current_angle_,current_goal.position);
  std::cout << fabs(this->target_dist_-this->current_dist_) << "," << this->target_angle_-this->current_angle_ << std::endl;
}

bool NoCollisionAlgorithm::movePlatform(const sensor_msgs::LaserScan & laser,const geometry_msgs::Pose & local_goal, geometry_msgs::Twist &twist)
{
  ros::Time prev_time  = current_time_;
  current_time_ = ros::Time::now();
  double dist_error,angle_error;
  double dt = current_time_.toSec() - prev_time.toSec();

  this->lock();
    ROS_WARN("NoCollisionAlgorithm::movePlatform::MOVE!");
    // compute the current distance to goal
    fromCartesian2Polar(local_goal.position,this->current_dist_,this->current_angle_);
    this->current_dist_-=LASER_SAFE_DIST_+MIN_SAFE_DISTANCE_;
    if(this->current_dist_<0) 
      this->current_dist_=0;
    ROS_WARN("NoCollisionAlgorithm::Current dist: %f Current angle: %f",this->current_dist_,this->current_angle_);
    // always rotate
    angle_error=fabs(this->current_angle_);
    if(angle_error>=this->ANGLE_TOL_)// still far from the target position
    {
      if(this->vr_state_==traj_idle)// not a new goal but the error has increased.
      {
        this->target_vr_=this->desired_vr_;
        this->target_angle_=this->current_angle_;
        this->updateRotationTrajParams();
      }
      if(this->current_vr_!=this->target_vr_)// it is necessary to change the current speed
      {
        if(this->current_vr_<this->target_vr_)
        {
          this->current_vr_+=this->R_ACC_*dt;
          if(this->current_vr_>this->target_vr_)
            this->current_vr_=this->target_vr_;
        }
        else
        {
          this->current_vr_-=this->R_ACC_*dt;
          if(this->current_vr_<this->target_vr_)
            this->current_vr_=this->target_vr_;
        }
        this->vr_state_=traj_acc;
      }
      else
      {
        if(angle_error>=this->deacc_angle_)
        {
          // keep the current speed
          this->vr_state_=traj_speed;
        }
        else
        {
          if(this->current_vr_>0)
          {
            this->current_vr_-=this->R_ACC_*dt;
            if(this->current_vr_<=0)
              this->current_vr_=0;
            this->target_vr_=this->current_vr_;
          }
          else
          {
            this->current_vr_+=this->R_ACC_*dt;
            if(this->current_vr_>=0)
              this->current_vr_=0;
            this->target_vr_=this->current_vr_;
          }
          this->vr_state_=traj_deacc;
        }
      }
    }
    else
    {
      this->current_vr_=0;
      this->vr_state_=traj_idle;
    }
    // if the angle error is small, start moving forward
    if(angle_error<(MAX_ANGLE_TO_STOP_))
    {
      // check weather the goal is traversable or not
      if(isGoalTraversable(laser,local_goal.position))
      {
        dist_error=fabs(this->current_dist_);
        if(dist_error>=this->DIST_TOL_)// still far from the target position
        {
          if(this->vt_state_==traj_idle)// not a new goal but the error has increased.
          {
            this->target_vt_=this->desired_vt_;
            this->target_dist_=this->current_dist_;
            this->updateTranslationTrajParams();
          }
          if(this->current_vt_!=this->target_vt_)// it is necessary to change the current speed
          {
            if(this->current_vt_<this->target_vt_)
            {
              this->current_vt_+=this->T_ACC_*dt;
              if(this->current_vt_>this->target_vt_)
                this->current_vt_=this->target_vt_;
            }
            else
            { 
              this->current_vt_-=this->T_ACC_*dt;
              if(this->current_vt_<this->target_vt_)
                this->current_vt_=this->target_vt_;
            }
            this->vt_state_=traj_acc;
          }
          else
          {
            if(dist_error>=this->deacc_dist_)
            {
              // keep the current speed
              this->vt_state_=traj_speed;
            }
            else
            {
              if(this->current_vt_>0)
              {
                this->current_vt_-=this->T_ACC_*dt;
                if(this->current_vt_<=0)
                  this->current_vt_=0;
                this->target_vt_=this->current_vt_;
              }
              else
              {
                this->current_vt_+=this->T_ACC_*dt;
                if(this->current_vt_>=0)
                  this->current_vt_=0;
                this->target_vt_=this->current_vt_;
              }
              this->vt_state_=traj_deacc;
            }
          }
        }
        else
        {
          this->current_vt_=0;
          this->vt_state_=traj_idle;
        }
      }
      else// there is an obstacle in the path
      {
        ROS_WARN("NoCollisionAlgorithm::movePlatform::STOP!");
        // start decelerating ??
        this->current_vt_=0;
      }
    }
    else// decelerate if the angle error increases
    {
      if(this->current_vt_==0)
        this->vt_state_=traj_idle;
      else
      {
        if(this->current_vt_>0)
        {
          this->current_vt_-=this->T_ACC_*dt;
          if(this->current_vt_<=0)
          this->current_vt_=0;
          this->target_vt_=this->current_vt_;
        }
        else
        {
          this->current_vt_+=this->T_ACC_*dt;
          if(this->current_vt_>=0)
            this->current_vt_=0;
          this->target_vt_=this->current_vt_;
        }
        this->vt_state_=traj_deacc;
      }
    }
    twist.linear.x=this->current_vt_;
    twist.angular.z=this->current_vr_;
    ROS_INFO("vT=%f\t\tvR=%f,state=%d", twist.linear.x, twist.angular.z,this->vr_state_);
  this->unlock();
  if(this->vt_state_==traj_idle && this->vr_state_==traj_idle)
    return true;
  else
    return false;
}

bool NoCollisionAlgorithm::isGoalTraversable(const sensor_msgs::LaserScan & laser, 
                                             const geometry_msgs::Point & goal)
{
  // convert goal from cartesian to polar coordinates
  float module_goal, angle_goal;
  fromCartesian2Polar(goal, module_goal, angle_goal);

  // index of the laser range corresponding to robot orientation
  const unsigned int central_index_range = round(laser.ranges.size()/2);

  // transform angle goal from robot coordinates to laser ranges
  const unsigned int goal_index_range = round(angle_goal/laser.angle_increment) + central_index_range;

  // total aperture angle in radians
  const double cone_aperture = OA_CONE_ANGLE_; //*M_PI/180.f;
  const unsigned int half_index_cone = abs(round(cone_aperture/(laser.angle_increment*2)));

  // for all cone indexes inside laser scan
  for(unsigned int ii=std::max(goal_index_range-half_index_cone,(unsigned int)0);
                   ii<std::min(goal_index_range+half_index_cone,(unsigned int)laser.ranges.size());
                   ii++)
  {
    // if there is an obstacle
    if ( laser.ranges[ii] > MIN_LASER_RANGE_ && laser.ranges[ii] <= this->LASER_SAFE_DIST_ )
    {
      ROS_DEBUG("NoCollisionAlgorithm::isGoalTraversable::FALSE");
      return false;
    }
  }

  ROS_DEBUG("NoCollisionAlgorithm::isGoalTraversable::TRUE");
  return true;
}
