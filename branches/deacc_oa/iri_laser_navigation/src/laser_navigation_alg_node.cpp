#include "laser_navigation_alg_node.h"
#include <iostream>

LaserNavigationAlgNode::LaserNavigationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<LaserNavigationAlgorithm>(),
  send_goal_client_("move_base", true)
{
  ROS_INFO("Laser Navigation: Warming up..");
  //init class attributes if necessary
  //this->loop_rroscdate_ = 2;//in [Hz]

  // [init publishers]
  this->scans_map_publisher_ = this->public_node_handle_.advertise<sensor_msgs::LaserScan>("scans_map", 100);
  this->checkpoints_publisher_ = this->public_node_handle_.advertise<visualization_msgs::Marker>("checkpoints", 100);

  // [init subscribers]
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 100, &LaserNavigationAlgNode::scan_callback, this);

  // [init services]

  // [init clients]
  localise_client_ = this->public_node_handle_.serviceClient<iri_laser_localisation::DoLocalisation>("/iri_laser_localisation/localise");

  // [init action servers]

  // [init action clients]

  current_ = 0;
  new_scan_ = false;
  waiting_=true;
  first_=true;
}

LaserNavigationAlgNode::~LaserNavigationAlgNode(void)
{
  // [free dynamic memory]
}

/// ----------------   main THREAD   -------------------------------------------

void LaserNavigationAlgNode::mainNodeThread(void)
{
  if(first_)
  {
    sleep(2);
    //load_path_("/home/mmorta/Experiments/iri_laser_navigation/teo/test_laser_localization_2012-03-12-16-49-43.bag");
    //load_path_("/home/pau/experiments/test_laser_localization.bag");
    load_path_(alg_.config_.bag_path);
    first_=false;
  }
  for(uint j=current_;j<pose_path_.size();j++)
      publish_marker(pose_path_[j],0);

  if(waiting_){
    ROS_DEBUG("Laser Navigation: Waiting for a LaserScan..");
    waiting_=false;
  }
  if(new_scan_){
    new_scan_ = false;
    waiting_=true;


    /// >>> MOVE

    ROS_INFO_PRESS("move");

    /// Update the next goal
    /// Send it to Move Base & wait until it reaches (or not) it
    // [fill action structure and make request to the action server]
    if( current_ < scan_path_.size() )
    {
      send_goalMakeActionRequest(pose_path_[current_]);
    }
    else
    {
      /// If it has reached the last goal, wait until the program is closed
      ROS_WARN("Laser Navigation: Last goal reached");

      while(1)
        sleep(1);
    }

    /// >>> SENSE

    ROS_INFO_PRESS("get path pose");

    /// Localise the robot
    // [fill srv structure and make request to the server]
    scan_mutex_.enter();
      localise_srv_.request.scan_sens = scan_sens_;
    scan_mutex_.exit();

    scan_path_[current_].header.stamp = ros::Time::now();

    localise_srv_.request.scan_map = scan_path_[current_];

    this->scans_map_publisher_.publish(scan_path_[current_]);

    ROS_INFO_PRESS("localise");

    ROS_DEBUG("Laser Navigation: Sending localisation request");
    if (localise_client_.call(localise_srv_))
    {
      /// Here the robot is localised with the following pose
      ROS_DEBUG("Laser Navigation: Localised Pose: x=%f y=%f th=%f",
              localise_srv_.response.pose.pose.pose.position.x ,
              localise_srv_.response.pose.pose.pose.position.y ,
              tf::getYaw(localise_srv_.response.pose.pose.pose.orientation));
    }
    else
    {
      ROS_ERROR("Laser Navigation: Failed to Call Server on topic localise ");
    }
    current_++;

    // [publish messages]


  }
}

/*  [subscriber callbacks] */
void LaserNavigationAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("LaserNavigationAlgNode::scan_callback: New Message Received");
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->scan_mutex_.enter();
  scan_sens_ = *msg;
  new_scan_ = true;
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->scan_mutex_.exit();
}

/*  [service callbacks] */

/*  [action callbacks] */
void LaserNavigationAlgNode::send_goalDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result)
{
  if( state.toString().compare("SUCCEEDED") == 0 )
    ROS_INFO("Laser Navigation: Goal Reached");
  else
    ROS_INFO("Laser Navigation: %s", state.toString().c_str());

  //copy & work with requested result
}

void LaserNavigationAlgNode::send_goalActive()
{
  //ROS_INFO("LaserNavigationAlgNode::send_goalActive: Goal just went active!");
}

void LaserNavigationAlgNode::send_goalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
  //ROS_INFO("LaserNavigationAlgNode::send_goalFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    send_goal_client_.cancelGoal();
    //ROS_INFO("LaserNavigationAlgNode::send_goalFeedback: Cancelling Action!");
  }
}

/// ----------------   SEND GOAL  ----------------------------------------------

/*  [action requests] */
void LaserNavigationAlgNode::send_goalMakeActionRequest(const geometry_msgs::PoseStamped & new_goal)
{
  //ROS_INFO("LaserNavigationAlgNode::send_goalMakeActionRequest: Starting New Request!");

  //wait for the action server to start
  //will wait for infinite time
  ROS_DEBUG("Laser Navigation: Send Goal: Waiting for Server...");
  send_goal_client_.waitForServer();

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header = new_goal.header;
  goal.target_pose.header.frame_id = alg_.config_.goal_frame;
  goal.target_pose.pose   = new_goal.pose;

  publish_marker(new_goal,1);

  ROS_INFO_XYR("New Goal", goal.target_pose.pose.position.x,
                           goal.target_pose.pose.position.y,
                           goal.target_pose.pose.orientation);

  //send a goal to the action
  send_goal_client_.sendGoal(goal,
              boost::bind(&LaserNavigationAlgNode::send_goalDone,     this, _1, _2),
              boost::bind(&LaserNavigationAlgNode::send_goalActive,   this),
              boost::bind(&LaserNavigationAlgNode::send_goalFeedback, this, _1));
  ROS_DEBUG("Laser Navigation: Goal Sent. Wait for Result");

    // wait for the action to return
  float server_timeout = 100.f; //in [secs]
  bool finished_before_timeout = send_goal_client_.waitForResult(ros::Duration(server_timeout));

  //if server replies in time
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = send_goal_client_.getState();
    ROS_DEBUG("Laser Navigation: Action Succesfully Accomplished!");
    ROS_DEBUG("Laser Navigation: Send Goal: %s", state.toString().c_str());
  }
  else
  {
    send_goal_client_.cancelGoal();
    ROS_WARN("Laser Navigation: Action did NOT finish before Timeout. Cancelling");
  }
  publish_marker(new_goal,2);
}

void LaserNavigationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void LaserNavigationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<LaserNavigationAlgNode>(argc, argv, "laser_navigation_alg_node");
}

// -----------------------------------------------------------------------------

/// ----------------   LOAD PATH   ---------------------------------------------

bool LaserNavigationAlgNode::load_path_(const std::string & bag_path)
{
  rosbag::Bag bag;
  ROS_INFO("Laser Navigation: Load Path: Opening Bag '%s'",bag_path.c_str());
  bag.open(bag_path, rosbag::bagmode::Read);
//
  std::vector<std::string> topics;
  topics.push_back(std::string(alg_.config_.scan_topic));
  topics.push_back(std::string(alg_.config_.pose_topic));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool get_pose=false;
  int counter=0;


  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == topics[0])
    {
      //&& counter>1000
      if((counter%50==0) || counter ==1)
      {
        sensor_msgs::LaserScan::ConstPtr las;
        las = m.instantiate<sensor_msgs::LaserScan>();
        ROS_DEBUG("Laser Navigation: Load Path: Scan: %d",las->header.seq);
        scan_path_.push_back(*las);
        get_pose=true;
      }
      counter++;
    }

    if(m.getTopic() == topics[1] && get_pose)
    {
      nav_msgs::Odometry::ConstPtr pose;
      pose = m.instantiate<nav_msgs::Odometry>();
      ROS_INFO("Pose: %d",pose->header.seq);
      geometry_msgs::PoseStamped aux;
      aux.header = pose->header;
      aux.pose = pose->pose.pose;
      pose_path_.push_back(aux);
      get_pose=false;
    }
  }
  bag.close();
  ROS_INFO("Laser Navigation: Load Path: Bag loaded and closed");
  ROS_INFO("Laser Navigation: Load Path: scans: %d poses: %d",scan_path_.size(),pose_path_.size());

  return true;
}

/// ----------------   PUBLISH MARKER   ----------------------------------------

void LaserNavigationAlgNode::publish_marker(const geometry_msgs::PoseStamped & pose, const int & type)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = alg_.config_.goal_frame;
    marker.header.stamp    = ros::Time();
    marker.ns              = "checkpoints";
    marker.id              = pose.header.seq;
    marker.type            = visualization_msgs::Marker::ARROW;
    marker.action          = visualization_msgs::Marker::ADD;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.pose.position = pose.pose.position;
    marker.pose.orientation = pose.pose.orientation;

    float c[4]={1,1,0,0.7};
    switch(type){
      case 1:
        c[1]=0;
        break;
      case 2:
        c[0]=c[1]=c[2]=0.5;
        break;
    }
    marker.color.r = c[0];
    marker.color.g = c[1];
    marker.color.b = c[2];
    marker.color.a = c[3];

    checkpoints_publisher_.publish(marker);
    ROS_DEBUG("Marker x:%f y:%f",marker.pose.position.x,marker.pose.position.y);
}

void LaserNavigationAlgNode::ROS_INFO_PRESS(const std::string & str)
{
    ROS_INFO("\033[36mPress 'intro' to %s\033[0m",str.c_str());
    //std::cout << "\033[1A";
    std::cin.get();
}

void LaserNavigationAlgNode::ROS_INFO_XYR(const std::string & str,const float & x,const float & y,const geometry_msgs::Quaternion & r)
{
  ROS_INFO("Laser Navigation: %s",str.c_str());
  ROS_INFO("\033[31mx:\033[0m%f \033[32my:\033[0m%f \033[34mth:\033[0m%f ",
           x,y,tf::getYaw(r));
}

