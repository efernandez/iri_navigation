#include "platform_teleop_alg_node.h"

#include <wiimote/State.h>

PlatformTeleopAlgNode::PlatformTeleopAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PlatformTeleopAlgorithm>(),
  vT_(0.f),
  vR_(0.f),
  dVT_(0.1f),
  dVR_(0.1f),
  check_human_(false),
  human_is_alive_(false)
{
  //init class attributes if necessary
  //loop_rate_ = 2;//in [Hz]

  // [init publishers]
  cmd_vel_publisher_ = public_node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  
  // [init subscribers]
  joy_subscriber_    = public_node_handle_.subscribe("joy", 100, &PlatformTeleopAlgNode::joy_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

PlatformTeleopAlgNode::~PlatformTeleopAlgNode(void)
{
  // [free dynamic memory]
}

void PlatformTeleopAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //Twist_msg_.data = my_var;
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void PlatformTeleopAlgNode::joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) 
{
  bool move=false;
  static bool first=true;
//  std::vector<int> axes_index;
//  bool axes_mov = check_movement_axes_callback( joy_msg->axes, axes_index);
  
  if(first)
  {
    prev_buttons_.resize(joy_msg->buttons.size());
    first=false;
  }

  // AXES
//  if (axes_mov)
//  {
//    if( !compareIndexVectors(axes_index, prev_axes_) )
//      for(unsigned int i=0; i<axes_index.size(); i++)
//        useAxes(joy_msg->axes[axes_index[i]], axes_index[i]);
//  }
//  else
//    prev_axes_.clear();

  
  // BUTTONS
  for(unsigned int i=0; i<joy_msg->buttons.size(); i++)
  {
    if(check_human_)
    {
      if(joy_msg->buttons[wiimote::State::MSG_BTN_B]==0)
      {
        human_is_alive_=false;
        vT_ = vR_ = 0.0f;
        if(prev_buttons_[wiimote::State::MSG_BTN_B]==1)
          move=true;
      }
      else
        human_is_alive_=true;
    }

    if(joy_msg->buttons[i]==1 && prev_buttons_[i]==0)
    {
      if(useButton(i))
        move=true;
    }
  }
  // Publish Twist
  if(move)
  {
    geometry_msgs::Twist msg = generateTwist();
    cmd_vel_publisher_.publish(msg);
  }
  prev_buttons_ = joy_msg->buttons;
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PlatformTeleopAlgNode::node_config_update(Config &config, uint32_t level)
{
  alg_.lock();
    dVT_ = config.inc_trans;
    dVR_ = config.inc_rot;
    check_human_ = config.check_human;
  alg_.unlock();
}

void PlatformTeleopAlgNode::addNodeDiagnostics(void)
{
}

bool PlatformTeleopAlgNode::check_movement_axes_callback(const std::vector<float> & axes, std::vector<unsigned int> & index)
{
  if (axes.size() == 0)
    return false;

  bool new_data = false;
  for (unsigned int i=0; i<axes.size(); i++)
  {
    if (axes[i] != 0.0)
    {
      new_data = true;
      index.push_back(i);
    }
  }
    
  return new_data;
}

geometry_msgs::Twist PlatformTeleopAlgNode::generateTwist(void)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = vT_;
  twist.angular.z = vR_;

  return twist;
}

bool PlatformTeleopAlgNode::useButton(const unsigned int & index)
{
  bool move=true;

  alg_.lock();
    switch(index)
    {
      case wiimote::State::MSG_BTN_A:
        ROS_DEBUG("STOP!");
        vT_ = 0.0f;
        vR_ = 0.0f;
        break;

      case wiimote::State::MSG_BTN_LEFT:
        ROS_DEBUG("left!");
        if((check_human_ && human_is_alive_) || !check_human_)
          vR_ += dVR_;
        else
        {
          ROS_INFO("Human please, press B if you want to move");
          vR_ = 0.0f;
        }
        break;

      case wiimote::State::MSG_BTN_RIGHT:
        ROS_DEBUG("right!");
        if((check_human_ && human_is_alive_) || !check_human_)
          vR_ -= dVR_;
        else
        {
          ROS_INFO("Human please, press B if you want to move");
          vR_ = 0.0f;
        }
        break;

      case wiimote::State::MSG_BTN_UP:
        ROS_DEBUG("forward!");
        if((check_human_ && human_is_alive_) || !check_human_)
          vT_ += dVT_;
        else
        {
          ROS_INFO("Human please, press B if you want to move");
          vT_ = 0.0f;
        }
        break;

      case wiimote::State::MSG_BTN_DOWN:
        ROS_DEBUG("backward!");
        if((check_human_ && human_is_alive_) || !check_human_)
          vT_ -= dVT_;
        else
        {
          ROS_INFO("Human please, press B if you want to move");
          vT_ = 0.0f;
        }
        break;

      default:
        useExtraButton(index);
        move=false;
        break;
    }
  alg_.unlock();

  return move;
}

void PlatformTeleopAlgNode::useExtraButton(const unsigned int & index)
{
  switch(index)
  {
    case wiimote::State::MSG_BTN_1:
      ROS_DEBUG("Button 1!");
      break;
    case wiimote::State::MSG_BTN_2:
      ROS_DEBUG("Button 2!");
      break;
    case wiimote::State::MSG_BTN_B:
      ROS_DEBUG("Button B!");
      break;
    case wiimote::State::MSG_BTN_PLUS:
      ROS_DEBUG("Button (+)!");
      break;
    case wiimote::State::MSG_BTN_MINUS:
      ROS_DEBUG("Button (-)!");
      break;
    case wiimote::State::MSG_BTN_HOME:
      ROS_DEBUG("Button Home!");
      break;
  }
}

void PlatformTeleopAlgNode::useAxes(const float & axe_value, const unsigned int & index)
{
}


/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PlatformTeleopAlgNode>(argc, argv, "platform_teleop_alg_node");
}
