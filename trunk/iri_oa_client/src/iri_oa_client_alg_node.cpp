#include "iri_oa_client_alg_node.h"

IriOAClientAlgNode::IriOAClientAlgNode(void) :
  MoveBase_client_("MoveBase", true)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  nav_goal_subscriber_  = public_node_handle_.subscribe("nav_goal", 100, &IriOAClientAlgNode::nav_goal_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

IriOAClientAlgNode::~IriOAClientAlgNode(void)
{
  // [free dynamic memory]
}

void IriOAClientAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void IriOAClientAlgNode::nav_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{ 
  ROS_INFO("NoCollisionAlgNode::nav_goal_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  alg_.lock(); 

    ROS_INFO("goal_reqs_=%d frame_id=%s pose=(%f,%f,%f)",
             msg->header.seq, msg->header.frame_id.c_str(),
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    MoveBaseMakeActionRequest(*msg);

  //unlock previously blocked shared variables 
  alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */
void IriOAClientAlgNode::MoveBaseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result) 
{ 
  ROS_INFO("IriOAClientAlgNode::MoveBaseDone: Goal Achieved!"); 
  ROS_INFO("IriOAClientAlgNode::MoveBaseDone: %s", state.toString().c_str()); 

  //copy & work with requested result 
} 

void IriOAClientAlgNode::MoveBaseActive(void) 
{ 
  ROS_INFO("IriOAClientAlgNode::MoveBaseActive: Goal just went active!"); 
} 

void IriOAClientAlgNode::MoveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) 
{ 
//   ROS_INFO("IriOAClientAlgNode::MoveBaseFeedback: Got Feedback!"); 

  //analyze feedback 
  ROS_INFO("IriOAClientAlgNode::MoveBaseFeedback: pose=(%f, %f)", 
           feedback->base_position.pose.position.x, 
           feedback->base_position.pose.position.y);

  //if feedback is not what expected, cancel requested goal 
//   if( !feedback_is_ok ) 
//   { 
//     MoveBase_client_.cancelGoal(); 
//     ROS_INFO("HrengagementAlgNode::MoveBaseFeedback: Cancelling Action!"); 
//   } 
}

/*  [action requests] */
void IriOAClientAlgNode::MoveBaseMakeActionRequest(const geometry_msgs::PoseStamped & new_goal) 
{ 
  ROS_INFO("IriOAClientAlgNode::MoveBaseMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  ROS_INFO("IriOAClientAlgNode::MoveBaseMakeActionRequest: Waiting for Server..."); 
  MoveBase_client_.waitForServer();
  ROS_INFO("IriOAClientAlgNode::MoveBaseMakeActionRequest: Server is Available!"); 

  //set goal
  move_base_msgs::MoveBaseGoal goal;
  
  goal.target_pose.header = new_goal.header;
  goal.target_pose.pose   = new_goal.pose;

  ROS_INFO("[%d] - NEW goal=(%f, %f, %f)", goal.target_pose.header.seq,
                                           goal.target_pose.pose.position.x,
                                           goal.target_pose.pose.position.y, 
                                           goal.target_pose.pose.position.z);
  
  //send a goal to the action 
  MoveBase_client_.sendGoal(goal, 
              boost::bind(&IriOAClientAlgNode::MoveBaseDone,     this, _1, _2), 
              boost::bind(&IriOAClientAlgNode::MoveBaseActive,   this), 
              boost::bind(&IriOAClientAlgNode::MoveBaseFeedback, this, _1)); 
  ROS_INFO("IriOAClientAlgNode::MoveBaseMakeActionRequest: Goal Sent. Wait for Result!"); 
  
  // wait for the action to return 
/*  float server_timeout = 100.f; //in [secs] 
  bool finished_before_timeout = MoveBase_client_.waitForResult(ros::Duration(server_timeout)); 

  //if server replies in time 
  if (finished_before_timeout) 
  { 
    actionlib::SimpleClientGoalState state = MoveBase_client_.getState(); 
    ROS_INFO("IriOAClientAlgNode::MoveBaseMakeActionRequest: Action Succesfully Accomplished!"); 
    ROS_INFO("IriOAClientAlgNode::MoveBaseMakeActionRequest: %s", state.toString().c_str()); 
  } 
  else 
  { 
    MoveBase_client_.cancelGoal(); 
    ROS_INFO("IriOAClientAlgNode::MoveBaseMakeActionRequest: Action did NOT finish before Timeout."); 
  }*/ 
}

void IriOAClientAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void IriOAClientAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<IriOAClientAlgNode>(argc, argv, "iri_oa_client_alg_node");
}
