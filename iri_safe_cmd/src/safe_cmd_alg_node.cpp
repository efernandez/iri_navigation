#include "safe_cmd_alg_node.h"

SafeCmdAlgNode::SafeCmdAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<SafeCmdAlgorithm>(),
  collision_time_(1),
  min_dist_(0.4),
  max_vel_front_(7),
  max_vel_rear_(7),
  limit_vel_front_(7),
  limit_vel_rear_(7),
  front_laser_received_(false),
  rear_laser_received_(false)
{
  //init class attributes if necessary
  loop_rate_ = 20;//in [Hz]

  // [init publishers]
  cmd_vel_safe_publisher_ = public_node_handle_.advertise<geometry_msgs::Twist>("cmd_vel_safe", 100);
  
  // [init subscribers]
  cmd_vel_subscriber_     = public_node_handle_.subscribe("cmd_vel", 100, &SafeCmdAlgNode::cmd_vel_callback,this);
  rear_laser_subscriber_  = public_node_handle_.subscribe("rear_laser", 100, &SafeCmdAlgNode::rear_laser_callback, this);
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
  //Twist_msg_.data = my_var;
  
  if(!alg_.config_.unsafe)
  {
    if(!front_laser_received_)
      ROS_FATAL("SafeCmdAlgNode::mainNodeThread: Front laser not received");
    if(!rear_laser_received_)
      ROS_FATAL("SafeCmdAlgNode::mainNodeThread: Rear laser not received");
    
    front_laser_received_ = false;
    rear_laser_received_  = false;

    if(Twist_msg_.linear.x > fabs(max_vel_front_))
    {
      ROS_WARN_STREAM("heading to Front obstacle, reducing velocity"<<fabs(max_vel_front_));
      Twist_msg_.linear.x = fabs(max_vel_front_);
      if(max_vel_front_==0)
        Twist_msg_.angular.z = 0;
    }

    if(Twist_msg_.linear.x < -fabs(max_vel_rear_))
    {
      ROS_WARN_STREAM("heading to Rear obstacle, reducing velocity"<<fabs(max_vel_rear_));
      Twist_msg_.linear.x = -fabs(max_vel_rear_);
      if(max_vel_rear_==0)
        Twist_msg_.angular.z = 0;
    }
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
  //alg_.lock(); 
  //cmd_vel_mutex_.enter();

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
  //alg_.unlock(); 
  //cmd_vel_mutex_.exit(); 
}
void SafeCmdAlgNode::rear_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
  //ROS_INFO("SafeCmdAlgNode::rear_laser_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //alg_.lock(); 
  //rear_laser_mutex_.enter(); 

  max_vel_rear_ = std::min(compute_max_velocity_(msg),limit_vel_rear_);
  rear_laser_received_ = true;
  //ROS_INFO("Max vel r: %f",max_vel_rear_);

  //unlock previously blocked shared variables 
  //alg_.unlock(); 
  //rear_laser_mutex_.exit(); 
}
void SafeCmdAlgNode::front_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
  //ROS_INFO("SafeCmdAlgNode::front_laser_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //alg_.lock(); 
  //front_laser_mutex_.enter(); 

  max_vel_front_ = std::min(compute_max_velocity_(msg),limit_vel_front_);
  front_laser_received_ = true;
  //ROS_INFO("Max vel f: %f",max_vel_front_);

  //unlock previously blocked shared variables 
  //alg_.unlock(); 
  //front_laser_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void SafeCmdAlgNode::node_config_update(Config &config, uint32_t level)
{
  alg_.lock();
    collision_time_  = config.collision_time;
    min_dist_        = config.min_dist;
    limit_vel_front_ = config.limit_vel_front;
    limit_vel_rear_  = config.limit_vel_rear;
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
  float min_range    = *std::min_element( scan->ranges.begin(), scan->ranges.end() );
  int   min_pos      = distance(scan->ranges.begin(),std::min_element( scan->ranges.begin(), scan->ranges.end() ));
  float max_velocity = 0;

  ROS_DEBUG_STREAM("compute_max_velocity frame: " << scan->header.frame_id << 
                   " min range: " << min_range << " at " << min_pos << " of " << scan->ranges.size());

  if (min_range >= min_dist_ && min_range > 0.02)
    max_velocity = min_range / collision_time_;

  return max_velocity;
}
