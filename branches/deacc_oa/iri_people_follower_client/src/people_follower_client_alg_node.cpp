#include "people_follower_client_alg_node.h"
#include <tf/transform_listener.h>

PeopleFollowerClientAlgNode::PeopleFollowerClientAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PeopleFollowerClientAlgorithm>(),
  follow_client_("followTarget", true),
  action_active_(false),
  target_id_(-1)
{
  //init class attributes if necessary
  loop_rate_ = 10;//in [Hz]

  // [init publishers]
  target_pos_publisher_ = public_node_handle_.advertise<geometry_msgs::PoseStamped>("target_pose", 10);

  // [init subscribers]
  people_tracker_subscriber_ = public_node_handle_.subscribe("people_tracker", 100, &PeopleFollowerClientAlgNode::people_tracker_callback, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

PeopleFollowerClientAlgNode::~PeopleFollowerClientAlgNode(void)
{
  // [free dynamic memory]
}

void PeopleFollowerClientAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  if(action_active_)
    target_pos_publisher_.publish(PoseStamped_msg_);
}

/*  [subscriber callbacks] */
void PeopleFollowerClientAlgNode::people_tracker_callback(const iri_people_tracking::peopleTrackingArray::ConstPtr& msg) 
{
  //use appropiate mutex to shared variables if necessary
  people_tracker_mutex_.enter();
  ROS_DEBUG("PeopleFollowerClientAlgNode::people_tracker_callback: New Message Received (make_request=%d)", action_active_);

  if(!action_active_)
  {
    unsigned int target_index;

    //if there is someone standing
    if(alg_.isSomeoneStanding(msg, target_index))
    {
      target_id_ = msg->peopleSet[target_index].targetId;
      PoseStamped_msg_.header = msg->header;
      PoseStamped_msg_.pose.position.x = msg->peopleSet[target_index].x;
      PoseStamped_msg_.pose.position.y = msg->peopleSet[target_index].y;
      PoseStamped_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(0.f);

      //make follow request
      followMakeActionRequest();
    }
  }
  //action is active
  else
  {
    bool found = false;

    //search for desired target id
    for(unsigned int ii=0; ii<msg->peopleSet.size(); ii++)
    {
      if(msg->peopleSet[ii].targetId == target_id_)
      {
        target_id_=msg->peopleSet[ii].targetId;
        PoseStamped_msg_.header = msg->header;
        PoseStamped_msg_.pose.position.x = msg->peopleSet[ii].x;
        PoseStamped_msg_.pose.position.y = msg->peopleSet[ii].y;
        PoseStamped_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(0.f);
        found = true;
        break;
      }
    }

    //if target id not found
    if(!found)
    {
      ROS_WARN("PeopleFollowerClientAlgNode::people_tracker_callback: Target ID NOT found!");
      // cancel the current goal
      follow_client_.cancelGoal();
    }
  }

  people_tracker_mutex_.exit();
}

/*  [service callbacks] */

/*  [action callbacks] */
void PeopleFollowerClientAlgNode::followDone(const actionlib::SimpleClientGoalState& state,  const iri_nav_msgs::followTargetResultConstPtr& result)
{
  if( state.toString().compare("SUCCEEDED") == 0 )
    ROS_WARN("PeopleFollowerClientAlgNode::followDone: Goal Achieved! %s", state.toString().c_str());
  else
    ROS_WARN("PeopleFollowerClientAlgNode::followDone: %s", state.toString().c_str());

  people_tracker_mutex_.enter();
  action_active_ = false;
  ROS_INFO("PeopleFollowerClientAlgNode::followDone");
  people_tracker_mutex_.exit();
}

void PeopleFollowerClientAlgNode::followActive()
{
  //ROS_INFO("PeopleFollowerClientAlgNode::followActive: Goal just went active!");
}

void PeopleFollowerClientAlgNode::followFeedback(const iri_nav_msgs::followTargetFeedbackConstPtr& feedback)
{
  //ROS_INFO("PeopleFollowerClientAlgNode::followFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    follow_client_.cancelGoal();
    //ROS_INFO("PeopleFollowerClientAlgNode::followFeedback: Cancelling Action!");
  }
}

/*  [action requests] */
void PeopleFollowerClientAlgNode::followMakeActionRequest()
{
  ROS_INFO("PeopleFollowerClientAlgNode::followMakeActionRequest: Starting New Request!");

  //wait for the action server to start
  //will wait for infinite time
  follow_client_.waitForServer();
  ROS_INFO("PeopleFollowerClientAlgNode::followMakeActionRequest: Server is Available!");

  //send a goal to the action
  ROS_INFO("PeopleFollowerClientAlgNode::followMakeActionRequest: target_id_=%d",this->target_id_);
  follow_client_.sendGoal(follow_goal_,
              boost::bind(&PeopleFollowerClientAlgNode::followDone,     this, _1, _2),
              boost::bind(&PeopleFollowerClientAlgNode::followActive,   this),
              boost::bind(&PeopleFollowerClientAlgNode::followFeedback, this, _1));
  action_active_=true;
  ROS_INFO("PeopleFollowerClientAlgNode::followMakeActionRequest: Goal Sent. Wait for Result!");
}

void PeopleFollowerClientAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void PeopleFollowerClientAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PeopleFollowerClientAlgNode>(argc, argv, "people_follower_client_alg_node");
}
