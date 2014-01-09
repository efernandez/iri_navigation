#include "no_collision_fake_alg_node.h"

NoCollisionFakeAlgNode::NoCollisionFakeAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<NoCollisionFakeAlgorithm>(),
  move_base_aserver_(public_node_handle_, "MoveBase"),
  counter_(0)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  move_base_aserver_.registerStartCallback(boost::bind(&NoCollisionFakeAlgNode::move_baseStartCallback, this, _1)); 
  move_base_aserver_.registerStopCallback(boost::bind(&NoCollisionFakeAlgNode::move_baseStopCallback, this)); 
  move_base_aserver_.registerIsFinishedCallback(boost::bind(&NoCollisionFakeAlgNode::move_baseIsFinishedCallback, this)); 
  move_base_aserver_.registerHasSucceedCallback(boost::bind(&NoCollisionFakeAlgNode::move_baseHasSucceedCallback, this)); 
  move_base_aserver_.registerGetResultCallback(boost::bind(&NoCollisionFakeAlgNode::move_baseGetResultCallback, this, _1)); 
  move_base_aserver_.registerGetFeedbackCallback(boost::bind(&NoCollisionFakeAlgNode::move_baseGetFeedbackCallback, this, _1)); 
  move_base_aserver_.start();
  
  // [init action clients]
}

NoCollisionFakeAlgNode::~NoCollisionFakeAlgNode(void)
{
  // [free dynamic memory]
}

void NoCollisionFakeAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void NoCollisionFakeAlgNode::move_baseStartCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal)
{ 
  alg_.lock(); 
    ROS_INFO("AsMoveBaseAlgNode::START! frame_id=%s goal=(%f,%f)", 
             goal->target_pose.header.frame_id.c_str(), 
             goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
    counter_ = 0;
  alg_.unlock(); 
} 

void NoCollisionFakeAlgNode::move_baseStopCallback(void) 
{ 
  alg_.lock(); 
    //stop action 
  alg_.unlock(); 
} 

bool NoCollisionFakeAlgNode::move_baseIsFinishedCallback(void) 
{ 
  bool ret = false; 

  alg_.lock(); 
    if( counter_ < 3 )
    {
      counter_++;
      sleep(1);
    }
    else
    {
      ROS_INFO("AsMoveBaseAlgNode::DONE!");
      ret = true;
    }
    //if action has finish for any reason 
    //ret = true; 
  alg_.unlock(); 

  return ret; 
} 

bool NoCollisionFakeAlgNode::move_baseHasSucceedCallback(void) 
{ 
  bool ret = true; 

  alg_.lock(); 
    //if goal was accomplished 
    //ret = true 
  alg_.unlock(); 

  return ret; 
} 

void NoCollisionFakeAlgNode::move_baseGetResultCallback(move_base_msgs::MoveBaseResultPtr& result) 
{ 
  alg_.lock(); 
    //update result data to be sent to client 
    //result->data = data; 
  alg_.unlock(); 
} 

void NoCollisionFakeAlgNode::move_baseGetFeedbackCallback(move_base_msgs::MoveBaseFeedbackPtr& feedback) 
{ 
  alg_.lock(); 
    //keep track of feedback 
    //ROS_INFO("feedback: %s", feedback->data.c_str()); 
  alg_.unlock(); 
}

/*  [action requests] */

void NoCollisionFakeAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void NoCollisionFakeAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<NoCollisionFakeAlgNode>(argc, argv, "no_collision_fake_alg_node");
}
