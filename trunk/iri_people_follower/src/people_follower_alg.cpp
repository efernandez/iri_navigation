#include "people_follower_alg.h"

PeopleFollowerAlgorithm::PeopleFollowerAlgorithm(void) :
  tf_listener_()
{
}

PeopleFollowerAlgorithm::~PeopleFollowerAlgorithm(void)
{
}

void PeopleFollowerAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  lock();
    // save the current configuration
    config_=new_cfg;
  unlock();
}

// PeopleFollowerAlgorithm Public API

bool PeopleFollowerAlgorithm::transformGoal(geometry_msgs::PoseStamped & pose, 
                                            const std::string & target_frame,
                                            const std::string & fixed_frame) const
{
  bool tf_exists = false;

  try
  {
//    ros::Time request_time = ros::Time(0);
    ros::Time request_time = pose.header.stamp;
    ROS_DEBUG("PeopleFollowerAlgorithm::transformGoal:    now=%f", request_time.toSec());
    ROS_DEBUG("PeopleFollowerAlgorithm::transformGoal: header=%f", pose.header.stamp.toSec());

    //waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
    tf_exists = tf_listener_.waitForTransform(target_frame, pose.header.frame_id, request_time, ros::Duration(100));
    ROS_DEBUG("PeopleFollowerAlgorithm::transformGoal: tf_exists=%d",tf_exists);

    if(tf_exists)
    {
      ROS_DEBUG("PeopleFollowerAlgorithm::transformGoal: frame_id=%s\tpose=(%f, %f)",
        pose.header.frame_id.c_str(),
        pose.pose.position.x,
        pose.pose.position.y);

      //transformPose(target_frame, target_time, pose_in, fixed_frame, pose_out);
      tf_listener_.transformPose(target_frame, request_time, pose, fixed_frame, pose);

      ROS_DEBUG("PeopleFollowerAlgorithm::transformGoal: frame_id=%s\tpose=(%f, %f)",
        pose.header.frame_id.c_str(),
        pose.pose.position.x,
        pose.pose.position.y);

    }
  }
  catch(tf::TransformException e)
  {
    ROS_ERROR("PeopleFollowerAlgorithm::TF Error: %s", e.what());
    tf_exists = false;
  }

  return tf_exists;
}

bool PeopleFollowerAlgorithm::udpateGoal(geometry_msgs::PoseStamped & last_pose,
                                         const geometry_msgs::PoseStamped & current_pose,
                                         const float & threshold_dist) const
{
  const float alpha = 0.2f;//1.f;
  //compute distance from last pose to current pose
  float dx   = last_pose.pose.position.x - current_pose.pose.position.x;
  float dy   = last_pose.pose.position.y - current_pose.pose.position.y;
  float dist = sqrt(dx*dx + dy*dy);

  ROS_DEBUG("PeopleFollowerAlgorithm::udpateGoal:: last(%f,%f) current(%f,%f) needs_update=%d (%f>%f)",
           last_pose.pose.position.x,    last_pose.pose.position.y,
           current_pose.pose.position.x, current_pose.pose.position.y, (dist > alpha*threshold_dist),
           dist, alpha*threshold_dist);

  //if distance is greater than given threshold
  if( dist > alpha*threshold_dist )
  {
    //update last pose with current pose and return true
    last_pose = current_pose;
    return true;
  }
  //no need for an update, return false
  else
    return false;
}

geometry_msgs::Point
PeopleFollowerAlgorithm::substractSafetyDistance(const geometry_msgs::Point & goal_pose, 
                                                 const float & min_safety_dist)
{
  geometry_msgs::Point robot_pose;

  //compute safety module by substracting safety distance
  float safety_mod = sqrt(goal_pose.x*goal_pose.x + goal_pose.y*goal_pose.y) - min_safety_dist;

  //recompute coordinates
  robot_pose.x = safety_mod*goal_pose.x;
  robot_pose.y = safety_mod*goal_pose.y;

  ROS_DEBUG("from goal_pose(%f,%f) to robot_pose(%f,%f)",goal_pose.x,goal_pose.y,robot_pose.x,robot_pose.y);
  return robot_pose;
}
