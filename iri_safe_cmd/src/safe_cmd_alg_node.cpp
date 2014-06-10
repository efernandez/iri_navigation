#include "safe_cmd_alg_node.h"

SafeCmdAlgNode::SafeCmdAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<SafeCmdAlgorithm>(),
  collision_time_(1),
  min_dist_(0.4),
  max_vel_front_(7),
  max_vel_rear_(7),
  limit_vel_front_(7),
  limit_vel_rear_(7),
  front_laser_received(false),
  rear_laser_received(false)
{
  //init class attributes if necessary
  loop_rate_ = 20;//in [Hz]

  // [init publishers]
  cmd_vel_safe_publisher_ = public_node_handle_.advertise<geometry_msgs::Twist>("cmd_vel_safe", 100);
  
  // [init subscribers]
  cmd_vel_subscriber_ = public_node_handle_.subscribe("cmd_vel", 100, &SafeCmdAlgNode::cmd_vel_callback,this);
  rear_laser_subscriber_ = public_node_handle_.subscribe("rear_laser", 100, &SafeCmdAlgNode::rear_laser_callback, this);
  front_laser_subscriber_ = public_node_handle_.subscribe("front_laser", 100, &SafeCmdAlgNode::front_laser_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

SafeCmdAlgNode::~SafeCmdAlgNode(void)
{
  // [free dynamic memory]
}

void SafeCmdAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->Twist_msg_.data = my_var;
  
  if(!alg_.config_.unsafe)
  {
    if(!this->front_laser_received || !this->rear_laser_received)
      ROS_ERROR("SafeCmdAlgNode::mainNodeThread: laser/s not received");

    if(Twist_msg_.linear.x > fabs(max_vel_front_))
    {
      Twist_msg_.linear.x = fabs(max_vel_front_);
      ROS_WARN("heading to front obstacle, reducing velocity");
    }

    if(Twist_msg_.linear.x < -fabs(max_vel_rear_))
    {
      Twist_msg_.linear.x = -fabs(max_vel_rear_); 
      ROS_WARN("heading to rear obstacle, reducing velocity");
    }
    this->front_laser_received = false;
    this->rear_laser_received  = false;
  }
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  cmd_vel_safe_publisher_.publish(Twist_msg_);
  
}

/*  [subscriber callbacks] */
void SafeCmdAlgNode::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) 
{ 
  //ROS_INFO("SafeCmdAlgNode::cmd_vel_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->cmd_vel_mutex_.enter();

  if(!alg_.config_.unsafe)
  {
    if (msg->linear.x == 0 && msg->angular.z ==0)
      Twist_msg_ = *msg;
    else
    {
      Twist_msg_.linear.x  += (msg->linear.x  - last_twist_.linear.x ); 
      Twist_msg_.linear.y  += (msg->linear.y  - last_twist_.linear.y ); 
      Twist_msg_.linear.z  += (msg->linear.z  - last_twist_.linear.z ); 
      Twist_msg_.angular.x += (msg->angular.x - last_twist_.angular.x); 
      Twist_msg_.angular.y += (msg->angular.y - last_twist_.angular.y); 
      Twist_msg_.angular.z += (msg->angular.z - last_twist_.angular.z); 
    }
    last_twist_= *msg;
  } else
    Twist_msg_ = *msg;

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->cmd_vel_mutex_.exit(); 
}
void SafeCmdAlgNode::rear_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
  //ROS_INFO("SafeCmdAlgNode::rear_laser_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->rear_laser_mutex_.enter(); 

  max_vel_rear_ = std::min(compute_max_velocity_(msg),limit_vel_rear_);
  //ROS_INFO("Max vel r: %f",max_vel_rear_);
  this->rear_laser_received = true;

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->rear_laser_mutex_.exit(); 
}
void SafeCmdAlgNode::front_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
  //ROS_INFO("SafeCmdAlgNode::front_laser_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->front_laser_mutex_.enter(); 

  max_vel_front_ = std::min(compute_max_velocity_(msg),limit_vel_front_);
  //ROS_INFO("Max vel f: %f",max_vel_front_);
  this->front_laser_received = true;

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->front_laser_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void SafeCmdAlgNode::node_config_update(Config &config, uint32_t level)
{
  alg_.lock();
  this->collision_time_ = config.collision_time;
  this->min_dist_ = config.min_dist;
  this->limit_vel_front_ = config.limit_vel_front;
  this->limit_vel_rear_ = config.limit_vel_rear;
  alg_.unlock();
}

void SafeCmdAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<SafeCmdAlgNode>(argc, argv, "safe_cmd_alg_node");
}

float SafeCmdAlgNode::compute_max_velocity_(const sensor_msgs::LaserScan::ConstPtr& scan) const
{
  float max_velocity;

  float min_range = *std::min_element( scan->ranges.begin(), scan->ranges.end() );

  if (min_range < min_dist_)
    max_velocity = 0;
  else
    max_velocity = min_range / collision_time_;

  return max_velocity;
}
