#include "no_collision_alg_node.h"
#include<tf/tf.h>

NoCollisionAlgNode::NoCollisionAlgNode(void) :
  move_base_aserver_(public_node_handle_, "MoveBase"),
  tf_listener_(ros::Duration(10.f)),
  tf_listener2_(ros::Duration(10.f)),
  is_laser_ready_(false),
  target_frame_("/base_link"),
  fixed_frame_("/odom")
{
  //init class attributes if necessary
  loop_rate_ = 10;//5;//10;//in [Hz]

  // [init publishers]
  goal_marker_publisher_    = public_node_handle_.advertise<visualization_msgs::Marker>("goal_marker", 100);
  segway_cmd_publisher_     = public_node_handle_.advertise<geometry_msgs::Twist>("segway_cmd", 100);
  
  // [init subscribers]
  this->target_vel_subscriber_ = this->public_node_handle_.subscribe("target_vel", 100, &NoCollisionAlgNode::target_vel_callback, this);
  frontal_laser_subscriber_ = public_node_handle_.subscribe("scan", 100, &NoCollisionAlgNode::frontal_laser_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  move_base_aserver_.registerStartCallback(boost::bind(&NoCollisionAlgNode::startCallback, this, _1));
  move_base_aserver_.registerStopCallback(boost::bind(&NoCollisionAlgNode::stopCallback, this));
  move_base_aserver_.registerIsFinishedCallback(boost::bind(&NoCollisionAlgNode::isFinishedCallback, this));
  move_base_aserver_.registerHasSucceedCallback(boost::bind(&NoCollisionAlgNode::hasSucceedCallback, this));
  move_base_aserver_.registerGetResultCallback(boost::bind(&NoCollisionAlgNode::getResultCallback, this, _1));
  move_base_aserver_.registerGetFeedbackCallback(boost::bind(&NoCollisionAlgNode::getFeedbackCallback, this, _1));

  // [init action clients]

  goal_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  goal_marker_.type = visualization_msgs::Marker::CYLINDER;
  goal_marker_.action = visualization_msgs::Marker::ADD;
  goal_marker_.scale.x = 0.5;
  goal_marker_.scale.y = 0.5;
  goal_marker_.scale.z = 0.5;
  goal_marker_.color.a = 1.0;
  goal_marker_.color.r = 0.0;
  goal_marker_.color.g = 1.0;
  goal_marker_.color.b = 0.0;

  std::string tf_prefix;
  public_node_handle_.param<std::string>("tf_prefix", tf_prefix, "");

  target_frame_ = tf_prefix + "/base_link";
  fixed_frame_  = tf_prefix  + "/odom";

  action_running_=false;
}

NoCollisionAlgNode::~NoCollisionAlgNode(void)
{
  // [free dynamic memory]
}

void NoCollisionAlgNode::mainNodeThread(void)
{
  ros::Time target_time=ros::Time::now();
  geometry_msgs::PoseStamped current_local_goal;
  std::string source_frame = goal_pose_.header.frame_id; //(tf_prefix+"/base_link");

  if(!move_base_aserver_.isStarted())
  {
    if(is_laser_ready_)
    {
      move_base_aserver_.start();
      ROS_INFO("NoCollisionAlgNode:: Server Started!"); 
    }
  }
  else
  {
    if(action_running_)
    {
      try
      {
        // update the time stamp of the goal_pose_
        goal_pose_.header.stamp=ros::Time::now();
        // wait for a transformation between the fixed frame and the target_frame to be available now
                                    //waitForTransform(fixed_frame,   target_frame, target_time,        timeout, polling_sleep_duration);
        bool tf_exists = tf_listener_.waitForTransform(fixed_frame_, target_frame_, target_time, ros::Duration(10), ros::Duration(0.01));
        if(tf_exists)
        {
          geometry_msgs::Twist twist;
          // get the transformation of the goal_pose_ with respect to the current target_frame
          //tf_listener_.transformPose(target_frame, target_time, poseIn, fixed_frame, poseOut);
          tf_listener_.transformPose(target_frame_, target_time, goal_pose_, fixed_frame_, current_local_goal);
          // send nav goal marker for rviz visualization
          //goal_marker_.header.seq      = goal_pose_.header.seq;
          //goal_marker_.header.stamp    = target_time;
          //goal_marker_.header.frame_id = target_frame;
          goal_marker_.header          = goal_pose_.header;
          goal_marker_.pose            = goal_pose_.pose;//current_local_goal.pose;
          goal_marker_publisher_.publish(goal_marker_);
          // compute new twist based on laser scan and current goal position
          if(alg_.movePlatform(scan_, current_local_goal.pose,twist))
            action_running_=false;
          // send twist to platform
          segway_cmd_publisher_.publish(twist);
        }
        else
        {
          geometry_msgs::Twist twist;
          // send twist to platform
          segway_cmd_publisher_.publish(twist);
          ROS_INFO("NoCollisionAlgNode::No transform: %s-->%s", fixed_frame_.c_str(), target_frame_.c_str());
          move_base_aserver_.setAborted();
        }
      }
      catch (tf::TransformException &ex)
      {
        geometry_msgs::Twist twist;
        // send twist to platform
        segway_cmd_publisher_.publish(twist);
        ROS_INFO("NoCollisionAlgNode:: %s",ex.what());
        move_base_aserver_.setAborted();
      }
    }
  }

    // [fill msg structures]

    // [fill srv structure and make request to the server]

    // [fill action structure and make request to the action server]

    // [publish messages]
}

/*  [subscriber callbacks] */
void NoCollisionAlgNode::frontal_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
//   ROS_INFO("NoCollisionAlgNode::frontal_laser_callback: New Message Received"); 

  alg_.lock();
    is_laser_ready_ = true;

    scan_.header          = msg->header;
    scan_.angle_min       = msg->angle_min;
    scan_.angle_max       = msg->angle_max;
    scan_.angle_increment = msg->angle_increment;
    scan_.time_increment  = msg->time_increment; 
    scan_.scan_time       = msg->scan_time;
    scan_.range_min       = msg->range_min;
    scan_.range_max       = msg->range_max;
    scan_.ranges          = msg->ranges;
    scan_.intensities     = msg->intensities;

  alg_.unlock();
}

void NoCollisionAlgNode::target_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) 
{ 
  ROS_INFO("NoCollisionAlgNode::target_vel_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->target_vel_mutex_.enter(); 

  this->alg_.setTranslationalSpeed(msg->linear.x);
  this->alg_.setRotationalSpeed(msg->angular.z);
  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->target_vel_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */


void NoCollisionAlgNode::startCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal)
{
  ROS_INFO("NoCollisionAlgNode::start action: New action received"); 

  try
  {
    std::string source_frame = goal->target_pose.header.frame_id;
    ros::Time   source_time  = goal->target_pose.header.stamp;

                                 //waitForTransform(target_frame,       target_time, source_frame, source_time,  fixed_frame,           timeout, polling_sleep_duration);
    bool tf_exists  = tf_listener_.waitForTransform(target_frame_, ros::Time::now(), source_frame, source_time, fixed_frame_, ros::Duration(10), ros::Duration(0.01));

                                   //waitForTransform(target_frame,  source_frame,        time,           timeout, polling_sleep_duration);
    //bool tf_exists  = tf_listener_.waitForTransform(fixed_frame_,  source_frame, source_time, ros::Duration(10), ros::Duration(0.01));
    //bool tf_exists2 = tf_listener_.waitForTransform(target_frame_, fixed_frame_, source_time, ros::Duration(10), ros::Duration(0.01));
    if(tf_exists /*&& tf_exists2*/ )
    {
      geometry_msgs::PoseStamped current_local_goal;
      geometry_msgs::Twist twist;
      // get the position of the goal in the fixed_frame_ reference frame
      tf_listener_.transformPose(fixed_frame_,goal->target_pose,goal_pose_);
      // set the frame of the new goal_pose_ (hust in case)
      goal_pose_.header.frame_id=fixed_frame_;
      // get the position of the goal in the base_link frame
      tf_listener_.transformPose(target_frame_, source_time, goal_pose_, fixed_frame_, current_local_goal);
      // reset distance to goal for new action
      alg_.setGoal(current_local_goal.pose);
      alg_.movePlatform(scan_, current_local_goal.pose,twist);
      // get current time
      action_start_ = ros::Time::now();
      // send twist to platform
      segway_cmd_publisher_.publish(twist);
      action_running_=true;
    }
    else
    {
      ROS_ERROR("NoCollisionAlgNode::No transform");
      move_base_aserver_.setAborted();
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("NoCollisionAlgNode::startCallback: %s", ex.what());
    move_base_aserver_.setAborted(); 
  }

}

void NoCollisionAlgNode::stopCallback(void)
{
  ROS_WARN("NoCollisionAlgNode::stopCallback!!");
  //lock access to driver if necessary
  alg_.lock();
      geometry_msgs::Twist twist;// set both speeds to 0
      // send twist to platform
      segway_cmd_publisher_.publish(twist);
  //lock access to driver if necessary
  alg_.unlock();
}

bool NoCollisionAlgNode::isFinishedCallback(void)
{
  bool ret;

    ros::Duration elapsed_time = action_start_ - ros::Time::now();
    ret = ( alg_.isGoalReached() || elapsed_time.toSec() > ACTION_TIMEOUT );

  return ret;
}

bool NoCollisionAlgNode::hasSucceedCallback(void)
{
  bool ret;

  ret = alg_.isGoalReached();

  return ret;
}

void NoCollisionAlgNode::getResultCallback(move_base_msgs::MoveBaseResultPtr& result)
{
  //lock access to driver if necessary
  alg_.lock();

  //lock access to driver if necessary
  alg_.unlock();
}

void NoCollisionAlgNode::getFeedbackCallback(move_base_msgs::MoveBaseFeedbackPtr& feedback)
{
  ROS_WARN("NoCollisionAlgNode::Feedback");
  // update feedback with distance to goal
//  alg_.getDistance2Goal(feedback->base_position.pose);
//  feedback->base_position.header.stamp = ros::Time::now();
//  feedback->base_position.header.frame_id = target_frame_;
}

/*  [action requests] */

void NoCollisionAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void NoCollisionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<NoCollisionAlgNode>(argc, argv, "no_collision_alg_node");
}
