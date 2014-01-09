#include "people_follower_alg_node.h"

#include "exceptions.h"
#include <list>

PeopleFollowerAlgNode::PeopleFollowerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PeopleFollowerAlgorithm>(),
  followTarget_aserver_(public_node_handle_, "followTarget"),
  moveBase_client_("moveBase", true),
  new_req_event_id_("new_req_event"),
  moveBase_done_event_id_("moveBase_done_event"),
  moveBase_preempt_event_id_("moveBase_preempt_event"),
  target_pose_received_event_id_("target_pose_received_event"),
  follow_target_done_event_id_("follow_target_done_event"),
  current_state_(IDLE_STATE),
  request_id_(0),
  dist_to_goal_(0.f)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]

  // [init subscribers]
  target_pose_subscriber_ = public_node_handle_.subscribe("target_pose", 100, &PeopleFollowerAlgNode::target_pose_callback, this);

  // [init services]

  // [init clients]

  // [init action servers]
  followTarget_aserver_.registerStartCallback(boost::bind(&PeopleFollowerAlgNode::followTargetStartCallback, this, _1)); 
  followTarget_aserver_.registerStopCallback(boost::bind(&PeopleFollowerAlgNode::followTargetStopCallback, this)); 
  followTarget_aserver_.registerIsFinishedCallback(boost::bind(&PeopleFollowerAlgNode::followTargetIsFinishedCallback, this)); 
  followTarget_aserver_.registerHasSucceedCallback(boost::bind(&PeopleFollowerAlgNode::followTargetHasSucceedCallback, this)); 
  followTarget_aserver_.registerGetResultCallback(boost::bind(&PeopleFollowerAlgNode::followTargetGetResultCallback, this, _1)); 
  followTarget_aserver_.registerGetFeedbackCallback(boost::bind(&PeopleFollowerAlgNode::followTargetGetFeedbackCallback, this, _1)); 
  followTarget_aserver_.start();

  // [init action clients]

  //init events
  event_server_ = CEventServer::instance();

  event_server_->create_event(new_req_event_id_);
  event_server_->create_event(moveBase_done_event_id_);
  event_server_->create_event(moveBase_preempt_event_id_);
  event_server_->create_event(target_pose_received_event_id_);
  event_server_->create_event(follow_target_done_event_id_);

  public_node_handle_.param<std::string>("tf_prefix", tf_prefix_, "");
  target_frame_ = tf_prefix_ + "/base_link";
  fixed_frame_  = tf_prefix_ + "/odom";
}

PeopleFollowerAlgNode::~PeopleFollowerAlgNode(void)
{
  // [free dynamic memory]

  event_server_->delete_event(new_req_event_id_);
  event_server_->delete_event(moveBase_done_event_id_);
  event_server_->delete_event(moveBase_preempt_event_id_);
  event_server_->delete_event(target_pose_received_event_id_);
  event_server_->delete_event(follow_target_done_event_id_);
}

void PeopleFollowerAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]

  alg_.lock();
  switch(current_state_)
  {
    case IDLE_STATE:
      ROS_INFO("IDLE_STATE: request_id_=%d",request_id_);
//      try
//      {
        //wait for new follow request
        //event_server_->wait_first( std::list<std::string>(1, new_req_event_id_), 1000 );
        if( event_server_->event_is_set(new_req_event_id_) )
        {
          event_server_->reset_event(new_req_event_id_);

          //once we receive a follow target request
          //jumpt to request move base request state
          current_state_ = REQ_MB_STATE;
        }
        else
        {
          ROS_WARN("PeopleFollowerAlgNode::mainNodeThread: No follow request received");
        }

//        //once we receive a follow target request
//        //jumpt to request move base request state
//        current_state_ = REQ_MB_STATE;
//      }
//      catch(CException &e)
//      {
//        ROS_WARN("PeopleFollowerAlgNode::mainNodeThread: No follow request received");
//      }
      break;

    case REQ_MB_STATE:
      ROS_INFO("REQ_MB_STATE request_id_=%d",request_id_);
      //first time in this state, make move action request
//      try
//      {
//          event_server_->wait_first( std::list<std::string>(1, target_pose_received_event_id_), 1000 );
        //wait until tracker finds requested target id, or timeout
        if( event_server_->event_is_set(target_pose_received_event_id_) )
        {
          event_server_->reset_event(target_pose_received_event_id_);

          //update target and make request
          target_pose_mutex_.enter();
            global_target_goal_ = current_global_target_pose_;
          target_pose_mutex_.exit();

          //send a goal to the action
          move_base_msgs::MoveBaseGoal local_goal;
          local_goal.target_pose = global_target_goal_;

          //transform goal to base_link frame
          if( alg_.transformGoal(local_goal.target_pose, target_frame_, fixed_frame_) )
          {
            moveBaseMakeActionRequest(local_goal);
            current_state_ = FOLLOWING_STATE;
          }
          else
          {
            ROS_ERROR("PeopleFollowerAlgNode::mainNodeThread: Could NOT Transform from %s to %s", global_target_goal_.header.frame_id.c_str(), target_frame_.c_str());
            followTarget_aserver_.setPreempted();
            current_state_ = IDLE_STATE;
          }
        }
//      catch(CException &e)
//      {
//        ROS_ERROR("PeopleFollowerAlgNode::mainNodeThread: Follow Goal TIMEOUT: %s", e.what().c_str());
//        followTarget_aserver_.setPreempted();
//        current_state_ = IDLE_STATE;
//      }
      break;

    case FOLLOWING_STATE:
      ROS_INFO("FOLLOWING_STATE request_id_=%d",request_id_);
      //check if move base action has finish successfully
      if( event_server_->event_is_set(moveBase_done_event_id_) )
      {
        current_state_ = SUCCESS_STATE;
      }
      //if it has not finished yet
      else
      {
        //check if move base server has preempted its action
        if( event_server_->event_is_set(moveBase_preempt_event_id_) )
        {
          ROS_WARN("PeopleFollowerAlgNode::mainNodeThread: moveBase_preempt_event=%d", event_server_->event_is_set(moveBase_preempt_event_id_));
          current_state_ = PREEMPT_STATE;
        }
        //move base action is still valid
        else
        {
//          try
//          {
            //wait until tracker finds requested target id, or timeout
//            event_server_->wait_first( std::list<std::string>(1, target_pose_received_event_id_), 500 );
            if( event_server_->event_is_set(target_pose_received_event_id_) )
            {
              event_server_->reset_event(target_pose_received_event_id_);

              target_pose_mutex_.enter();
                bool update = alg_.udpateGoal(global_target_goal_, current_global_target_pose_, dist_to_goal_);
              target_pose_mutex_.exit();

              //if target has moved and needs an update
              if( update )
                current_state_ = UPDATE_STATE;
            }
            //no target pose message received
            else
            {
              no_target_pose_msg_received_++;

              if( no_target_pose_msg_received_ > MAX_ITERS_NO_MSG )
              {
                current_state_ = PREEMPT_STATE;
              }
            }
//          }
//          catch(CException &e)
//          {
//            ROS_ERROR("PeopleFollowerAlgNode::mainNodeThread: Follow Goal TIMEOUT: %s", e.what().c_str());
//            current_state_ = PREEMPT_STATE;
//          }
        }
      }
      break;

    case SUCCESS_STATE:
      ROS_WARN("SUCCESS_STATE request_id_=%d",request_id_);
//      event_server_->wait_first( std::list<std::string>(1, follow_target_done_event_id_) );
      if( event_server_->event_is_set(follow_target_done_event_id_) )
      {
        event_server_->reset_event(follow_target_done_event_id_);
        current_state_ = IDLE_STATE;
      }
      break;

    case UPDATE_STATE:
      ROS_WARN("UPDATE_STATE request_id_=%d",request_id_);
      moveBase_client_.cancelGoal();
      current_state_ = REQ_MB_STATE;
      break;

    case PREEMPT_STATE:
      ROS_WARN("PREEMPT_STATE request_id_=%d",request_id_);
      moveBase_client_.cancelGoal();
      followTarget_aserver_.setPreempted();
      current_state_ = IDLE_STATE;
      break;
  }
  alg_.unlock();
}

/*  [subscriber callbacks] */
void PeopleFollowerAlgNode::target_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_DEBUG("PeopleFollowerAlgNode::target_pose_callback: New Message Received");

  //reset counter
  no_target_pose_msg_received_ = 0;

  geometry_msgs::PoseStamped local_pose;
  local_pose = *msg;

  //transform goal from laser to fixed frame
  if( alg_.transformGoal(local_pose, fixed_frame_, fixed_frame_) )
  {
    //substract safety distance from original goal
    local_pose.pose.position = alg_.substractSafetyDistance(local_pose.pose.position, dist_to_goal_);
    target_pose_mutex_.enter();
      current_global_target_pose_ = local_pose;
    target_pose_mutex_.exit();

    ROS_INFO("PeopleFollowerAlgNode::people_tracker_callback: target_pose_received_event_id_");
    event_server_->set_event(target_pose_received_event_id_);
  }
  //if transform could not be made
  else
  {
    ROS_ERROR("PeopleFollowerAlgNode::people_tracker_callback: Could NOT Transform from %s to %s", 
              global_target_goal_.header.frame_id.c_str(), target_frame_.c_str());
  }
}

/*  [service callbacks] */

/*  [action callbacks] */
void PeopleFollowerAlgNode::followTargetStartCallback(const iri_nav_msgs::followTargetGoalConstPtr& goal)
{
  ROS_INFO("PeopleFollowerAlgNode::followTargetStartCallback: NEW REQUEST!");// target_id=%d",goal->target_id);

  alg_.lock();
    request_id_++;
  alg_.unlock();

  event_server_->set_event(new_req_event_id_);
  ROS_WARN("PeopleFollowerAlgNode::followTargetStartCallback: NEW REQ EVENT!!");
}

void PeopleFollowerAlgNode::followTargetStopCallback(void)
{
  alg_.lock();
    //stop action
  alg_.unlock();
}

bool PeopleFollowerAlgNode::followTargetIsFinishedCallback(void)
{
  bool ret = false;

  //if action has finish for any reason
  if( event_server_->event_is_set(moveBase_done_event_id_) )
    ret = true;

  return ret;
}

bool PeopleFollowerAlgNode::followTargetHasSucceedCallback(void)
{
  bool ret = false;

  //if goal was accomplished
  if( event_server_->event_is_set(moveBase_done_event_id_) )
  {
    ret = true;
    event_server_->set_event(follow_target_done_event_id_);
  }

  return ret;
}

void PeopleFollowerAlgNode::followTargetGetResultCallback(iri_nav_msgs::followTargetResultPtr& result)
{
  alg_.lock();
    //update result data to be sent to client
    //result->data = data;
  alg_.unlock();
}

void PeopleFollowerAlgNode::followTargetGetFeedbackCallback(iri_nav_msgs::followTargetFeedbackPtr& feedback)
{
  alg_.lock();
    //keep track of feedback
    //ROS_INFO("feedback: %s", feedback->data.c_str());
  alg_.unlock();
}

void PeopleFollowerAlgNode::moveBaseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result) 
{
  if( state.toString().compare("SUCCEEDED") == 0 )
  {
    ROS_INFO("PeopleFollowerAlgNode::moveBaseDone: Goal Achieved!");
    event_server_->set_event(moveBase_done_event_id_);
  }
  else
  {
    ROS_WARN("PeopleFollowerAlgNode::moveBaseDone: state=%s", state.toString().c_str());
    event_server_->set_event(moveBase_preempt_event_id_);
  }

  //ROS_INFO("PeopleFollowerAlgNode::moveBaseDone: %s", state.toString().c_str());

  //copy & work with requested result

}

void PeopleFollowerAlgNode::moveBaseActive()
{
  //ROS_INFO("PeopleFollowerAlgNode::moveBaseActive: Goal just went active!");
}

void PeopleFollowerAlgNode::moveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
  //ROS_INFO("PeopleFollowerAlgNode::moveBaseFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    moveBase_client_.cancelGoal();
    //ROS_INFO("PeopleFollowerAlgNode::moveBaseFeedback: Cancelling Action!");
  }
}

/*  [action requests] */
void PeopleFollowerAlgNode::moveBaseMakeActionRequest(const move_base_msgs::MoveBaseGoal & goal) 
{
  event_server_->reset_event(moveBase_done_event_id_);
  event_server_->reset_event(moveBase_preempt_event_id_);
  
  ROS_INFO("PeopleFollowerAlgNode::moveBaseMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  moveBase_client_.waitForServer();  
  ROS_INFO("PeopleFollowerAlgNode::moveBaseMakeActionRequest: Server is Available!"); 

  //send a goal to the action 
  moveBase_client_.sendGoal(goal, 
              boost::bind(&PeopleFollowerAlgNode::moveBaseDone,     this, _1, _2), 
              boost::bind(&PeopleFollowerAlgNode::moveBaseActive,   this), 
              boost::bind(&PeopleFollowerAlgNode::moveBaseFeedback, this, _1)); 
  ROS_INFO("PeopleFollowerAlgNode::moveBaseMakeActionRequest: Goal(%f,%f) Sent. Wait for Result!",
            goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
}

void PeopleFollowerAlgNode::node_config_update(Config &config, uint32_t level)
{
  alg_.lock();
    dist_to_goal_ = config.dist_to_goal;
    ROS_INFO("PeopleFollowerAlgNode::node_config_update: dist_to_goal_=%f",dist_to_goal_);
  alg_.unlock();
}

void PeopleFollowerAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PeopleFollowerAlgNode>(argc, argv, "people_follower_alg_node");
}
