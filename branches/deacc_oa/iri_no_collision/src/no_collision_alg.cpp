#include "no_collision_alg.h"

#include<math.h>
#include<tf/tf.h>
#include<algorithm>

NoCollisionAlgorithm::NoCollisionAlgorithm(void) :
  MIN_SAFE_DISTANCE_(0.4f),
  MIN_SAFE_ANGLE_(10.f*M_PI/180.f),
  LASER_SAFE_DIST_(0.8f),
  VT_MAX_(0.6f),
  VR_MAX_(0.2f),
  ACC_(0.1f),
  vT_(0.f),
  vR_(0.f),
  OA_CONE_ANGLE_(100.0f),
  MAX_TRANS_DEACC_DIST_((VT_MAX_*VT_MAX_)/(2*ACC_)),
  MAX_ROT_DEACC_DIST_((VR_MAX_*VR_MAX_)/(2*ACC_)),
  dist_to_goal_(2*MIN_SAFE_DISTANCE_),
  current_dist_to_goal_(dist_to_goal_),
  angle_to_goal_(2*MIN_SAFE_ANGLE_),
  current_angle_to_goal_(angle_to_goal_),
  trans_deacc_dist_(MAX_TRANS_DEACC_DIST_),
  rot_deacc_dist_(MAX_ROT_DEACC_DIST_),
  vT_done_(false),
  vR_done_(false)
{
}

NoCollisionAlgorithm::~NoCollisionAlgorithm(void)
{
}

void NoCollisionAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
    // save the current configuration
    MIN_SAFE_DISTANCE_ = new_cfg.min_safety_dist;
    MIN_SAFE_ANGLE_    = new_cfg.min_safety_angle*M_PI/180.f,
    LASER_SAFE_DIST_   = new_cfg.laser_safe_dist;
    VT_MAX_            = new_cfg.vT_max;
    VR_MAX_            = new_cfg.vR_max;
    ACC_               = new_cfg.acc;
    OA_CONE_ANGLE_     = new_cfg.oa_cone_aperture;

    // compute decreasing distances
    MAX_TRANS_DEACC_DIST_ = (VT_MAX_*VT_MAX_)/(2*ACC_);
    MAX_ROT_DEACC_DIST_   = (VR_MAX_*VR_MAX_)/(2*ACC_);

    this->config_=new_cfg;
  this->unlock();
}

// NoCollisionAlgorithm Public API
void NoCollisionAlgorithm::resetDistance2Goal(const geometry_msgs::Pose & local_goal)
{
  this->lock();
  ROS_WARN("NoCollisionAlgorithm::resetDistance2Goal");

  fromCartesian2Polar(local_goal.position,dist_to_goal_,angle_to_goal_);
  dist_to_goal_=dist_to_goal_-LASER_SAFE_DIST_-MIN_SAFE_DISTANCE_;
  ROS_INFO("NoCollisionAlgorithm::Module: %f Angle: %f",dist_to_goal_,angle_to_goal_);
  if(dist_to_goal_<0.1)
  {  
    vT_done_=true;
  }
  else
  {
    vT_=0.0;
    trans_deacc_dist_ = std::min(dist_to_goal_/2.f,  MAX_TRANS_DEACC_DIST_);
    vT_done_=false;
  }
  if(fabs(angle_to_goal_)<0.1)
  {
    vR_done_=true;
  }
  else
  {
    vR_=0.0;
    vR_done_=false;
    rot_deacc_dist_   = std::min((float)fabs(angle_to_goal_)/2.f, MAX_ROT_DEACC_DIST_);
  }
  current_time_ = ros::Time::now();
  ROS_INFO("First Current time:%f",current_time_.toSec());
  this->unlock();
}

bool NoCollisionAlgorithm::isGoalReached()
{
  return (vR_done_ && vT_done_);
}

geometry_msgs::Twist
NoCollisionAlgorithm::movePlatform(const sensor_msgs::LaserScan & scan, 
                                   const geometry_msgs::Pose & local_goal)
{
  geometry_msgs::Twist twist;
  ros::Time prev_time  = current_time_;
  current_time_        = ros::Time::now();
  ROS_INFO("Current time:%f",current_time_.toSec());
  double dt            = current_time_.toSec() - prev_time.toSec();

  this->lock();
    twist.linear.x  = 0.f;
    twist.angular.z = 0.f;
    if( isGoalTraversable(scan, LASER_SAFE_DIST_, local_goal.position) )// check it at the maximum update rate
    {
      ROS_WARN("NoCollisionAlgorithm::movePlatform::MOVE!");
      // compute angle between both vectors (robot turn angle)
      fromCartesian2Polar(local_goal.position,current_dist_to_goal_,current_angle_to_goal_);
      current_dist_to_goal_=current_dist_to_goal_-LASER_SAFE_DIST_-MIN_SAFE_DISTANCE_;
      ROS_WARN("NoCollisionAlgorithm::Current module: %f Current angle: %f",current_dist_to_goal_,current_angle_to_goal_);
      int angle_sign = (current_angle_to_goal_ > 0) ? 1 : -1;
      if( !vR_done_ )
      {
        if(fabs(current_angle_to_goal_)>0.1)
        {
          // update vR
          if( fabs(angle_to_goal_-current_angle_to_goal_) < rot_deacc_dist_ )
          {
            float dR = ACC_*dt*angle_sign;
            vR_ += dR;
            if(fabs(vR_)>VR_MAX_)
            {
              vR_=angle_sign*VR_MAX_;
              rot_deacc_dist_=fabs(angle_to_goal_-current_angle_to_goal_);
            }
            ROS_INFO("Increasing dR=%f vR=%f", dR, vR_);
          }
          else
          {
            if( fabs(current_angle_to_goal_) < rot_deacc_dist_ )
            {
              float dR = ACC_*dt*angle_sign;
              float new_vR = vR_ - dR;
              if( fabs(new_vR) < fabs(vR_) )
              {
                vR_ = new_vR;
                ROS_INFO("Decreasing dR=%f vR=%f", dR, vR_);
              }
              else
              {
                vR_done_ = true;
                vR_ = 0.f;
                ROS_INFO("vR Done!");
              }
            }
            else
            {
              ROS_INFO("Running at max vR=%f VR_MAX_=%f", vR_, VR_MAX_);
            }
          }
        }
        else
        {
          vR_done_=true,
          vR_=0.0;
        }
      }
      if(!vT_done_)
      {
        if(current_dist_to_goal_>0.1)
        {
          // update vT
          if( (dist_to_goal_-current_dist_to_goal_) < trans_deacc_dist_ )
          {
            float dT = ACC_*dt;
            vT_ += dT;
            if(vT_>VT_MAX_)
            {
              vT_=VT_MAX_;
              trans_deacc_dist_=(dist_to_goal_-current_dist_to_goal_);
            }
            ROS_INFO("Increasing dT=%f vT=%f", dT, vT_);
          }
          else
          {
            if( current_dist_to_goal_ < trans_deacc_dist_ )
            {
              float dT = ACC_*dt;
              vT_ -= dT;
              if( vT_ < 0 )
              {
                vT_=0.0;
                vT_done_ = true;
                ROS_INFO("vT Done!");
              }
              ROS_INFO("Decreasing dT=%f vT=%f", dT, vT_);
            }
            else
            {
              ROS_INFO("Running at max vT=%f VT_MAX_=%f", vT_, VT_MAX_);
            }
          }
        }  
        else
        {
          vT_done_=true;
          vT_=0.0;
        }
      }
      // copy velocities to twist
      twist.linear.x  = vT_;
      twist.angular.z = vR_;      
    }
    else
    {
      ROS_WARN("NoCollisionAlgorithm::movePlatform::STOP!");
      twist.linear.x  = 0.f;
      twist.angular.z = 0.f;
      this->unlock();
      resetDistance2Goal(local_goal);
      return twist;
    }

    ROS_INFO("vT=%f\t\tvR=%f", twist.linear.x, twist.angular.z);
  this->unlock();

  return twist;
}

void NoCollisionAlgorithm::fromCartesian2Polar(const geometry_msgs::Point & p, 
                                               float & module, 
                                               float & angle)
{
  module = sqrt( p.x*p.x + p.y*p.y );
  angle  = atan2( p.y, p.x );
}

void NoCollisionAlgorithm::fromPolar2Cartesian(const float & module,const float & angle,geometry_msgs::Point & p)
{
  p.x=module*cos(angle); 
  p.y=module*sin(angle);
}

bool NoCollisionAlgorithm::isGoalTraversable(const sensor_msgs::LaserScan & laser, 
                                             const float & safety_distance)
{
  unsigned int shields_offset = round(10*720.f/180.f);
  for(unsigned int ii=shields_offset; ii<laser.ranges.size()-shields_offset; ii++)
  {
    //TODO
    // check why laser is sending MIN_LASER_RANGE_ values
    if( laser.ranges[ii] > MIN_LASER_RANGE_ )
      if( laser.ranges[ii] <= safety_distance )
        return false;
  }
  
  return true;
}

bool NoCollisionAlgorithm::isGoalTraversable(const sensor_msgs::LaserScan & laser, 
                                             const float & safety_distance, 
                                             const geometry_msgs::Point & goal) const
{
  // convert goal from cartesian to polar coordinates
  float module_goal, angle_goal;
  fromCartesian2Polar(goal, module_goal, angle_goal);

  // index of the laser range corresponding to robot orientation
  const unsigned int central_index_range = round(laser.ranges.size()/2);

  // transform angle goal from robot coordinates to laser ranges
  const unsigned int goal_index_range = round(angle_goal/laser.angle_increment) + central_index_range;

  // total aperture angle in radians
  const double cone_aperture = OA_CONE_ANGLE_*M_PI/180.f;
  const unsigned int half_index_cone = abs(round(cone_aperture/(laser.angle_increment*2)));

  // for all cone indexes inside laser scan
  for(unsigned int ii=std::max(goal_index_range-half_index_cone,(unsigned int)0);
                   ii<std::min(goal_index_range+half_index_cone,laser.ranges.size());
                   ii++)
  {
    // if there is an obstacle
    if ( laser.ranges[ii] > MIN_LASER_RANGE_ && laser.ranges[ii] <= safety_distance )
    {
      ROS_DEBUG("NoCollisionAlgorithm::isGoalTraversable::FALSE");
      return false;
    }
  }

  ROS_DEBUG("NoCollisionAlgorithm::isGoalTraversable::TRUE");
  return true;
}
