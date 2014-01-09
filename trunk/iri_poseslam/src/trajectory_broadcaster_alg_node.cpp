#include "trajectory_broadcaster_alg_node.h"

TrajectoryBroadcasterAlgNode::TrajectoryBroadcasterAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TrajectoryBroadcasterAlgorithm>()
{
  //init class attributes if necessary

  loop_rate_ = 10;//in [Hz]

  public_node_handle_.getParam("base_frame_id", base_frame_id_);
  public_node_handle_.getParam("odom_frame_id", odom_frame_id_);
  public_node_handle_.getParam("map_frame_id",  map_frame_id_);

  // [init publishers]

  // [init subscribers]
  trajectory_subscriber_ = public_node_handle_.subscribe("trajectory", 100, &TrajectoryBroadcasterAlgNode::trajectory_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TrajectoryBroadcasterAlgNode::~TrajectoryBroadcasterAlgNode(void)
{
  // [free dynamic memory]
}

void TrajectoryBroadcasterAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->recompute_tf_mutex_.enter();
  tfb_.sendTransform( tf::StampedTransform(T_map_odom_, ros::Time::now(), map_frame_id_, odom_frame_id_) );
  ROS_DEBUG("TRAJ BROADCASTER: transform sended"); 
  this->recompute_tf_mutex_.exit();
}

/*  [subscriber callbacks] */
void TrajectoryBroadcasterAlgNode::trajectory_callback(const iri_poseslam::Trajectory::ConstPtr& msg)
{ 
  //ROS_INFO("TRAJ BROADCASTER: New Trajectory Message Received");
  //ROS_INFO_STREAM(*msg);
  this->recompute_tf_mutex_.enter();

  geometry_msgs::PoseWithCovarianceStamped new_pose = msg->poses.back();

  // T_base_odom_ via TF
  tf::StampedTransform T_base_odom_stamped;
  try{
    tfl_.waitForTransform(base_frame_id_, odom_frame_id_, ros::Time::now(), ros::Duration(3.0));

    tfl_.lookupTransform(base_frame_id_, odom_frame_id_, ros::Time(0), T_base_odom_stamped);

    T_base_odom_ = T_base_odom_stamped;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("TRAJ BROADCASTER: Transform exception: %s",ex.what());
  }
  
  // T_map_base_  Es calcula a partir del msg trajectory
  tf::poseMsgToTF(new_pose.pose.pose, T_map_base_);
  
  // T_map_odom_ Calcul final
  T_map_odom_ = T_map_base_ * T_base_odom_;

  this->recompute_tf_mutex_.exit();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TrajectoryBroadcasterAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  
  ROS_WARN("TRAJ BROADCASTER: Config updating...");

  //map_frame_id_  = config.map_frame_id;
  //odom_frame_id_ = config.odom_frame_id;
  //base_frame_id_ = config.base_frame_id;
  
  //ROS_WARN("TRAJ BROADCASTER: Config updated");

  this->alg_.unlock();
}

void TrajectoryBroadcasterAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TrajectoryBroadcasterAlgNode>(argc, argv, "trajectory_broadcaster_alg_node");
}