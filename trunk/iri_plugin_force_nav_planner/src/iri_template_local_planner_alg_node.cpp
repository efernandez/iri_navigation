#include "iri_template_local_planner_alg_node.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(iri_template_local_planner, 
                        TemplateLocalPlanner, 
                        TemplateLocalPlanner, 
                        nav_core::BaseLocalPlanner)


TemplateLocalPlanner::TemplateLocalPlanner(void) :
  algorithm_base::IriBaseAlgorithm<IriTemplateLocalPlannerAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 1, &TemplateLocalPlanner::odom_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TemplateLocalPlanner::~TemplateLocalPlanner(void)
{
  // [free dynamic memory]
}

void TemplateLocalPlanner::initialize(std::string name, tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmap_ros)
{
  ROS_INFO("TemplateLocalPlanner::initialize");
  this->tf_          = tf;
  this->costmap_ros_ = costmap_ros;
}

bool TemplateLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  //////////////////////////////////////////////////////
  // TO DO
  //////////////////////////////////////////////////////
  cmd_vel.linear.x=0.0;
  cmd_vel.linear.y=0.0;
  cmd_vel.angular.z=0.0;

  if(this->dist2goal>0.1)
  {
    if(this->goal_x>0)
      cmd_vel.linear.x =  std::max(std::min(this->goal_x/5.0,1.5),0.5);
    else if(this->goal_x<0)
      cmd_vel.linear.x = -std::max(std::min(this->goal_x/5.0,1.5),0.5);

    if(this->angle2goal>0.1)
      cmd_vel.angular.z =  std::min(this->angle2goal,1.5);
    else if(this->angle2goal<0.1)
      cmd_vel.angular.z = std::max(this->angle2goal,-1.5);
  }
  else
  {
    if(heading>0.1)
      cmd_vel.angular.z =  std::min(heading,1.5);
    else if(heading<0.1)
      cmd_vel.angular.z =  std::max(heading,-1.5);
  }

  //////////////////////////////////////////////////////
  // TO DO
  //////////////////////////////////////////////////////

  ROS_INFO("TemplateLocalPlanner::computeVelocityCommands: cmd_vel=(%f,%f,%f)", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  return true;
}

bool TemplateLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  this->global_plan_.clear();
  this->global_plan_ = orig_global_plan;
  ROS_INFO("TemplateLocalPlanner::setPlan: global_plan_ points %d", uint(this->global_plan_.size()));
  return true;
}

bool TemplateLocalPlanner::isGoalReached()
{
  ROS_INFO("TemplateLocalPlanner::isGoalReached");
  bool ok=false;

  //////////////////////////////////////////////////////
  // TO DO
  //////////////////////////////////////////////////////
  std::string base_frame_ = this->costmap_ros_->getBaseFrameID();
  if(!transformGlobalPlan(*this->tf_, this->global_plan_, *this->costmap_ros_, base_frame_, this->local_plan_))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller, %s", base_frame_.c_str());
    return false;
  }

  tf::Stamped<tf::Pose> goal_point;
  tf::poseStampedMsgToTF(this->local_plan_.back(), goal_point);
  this->goal_x  = goal_point.getOrigin().getX();
  this->goal_y  = goal_point.getOrigin().getY();
  double yaw    = tf::getYaw(goal_point.getRotation());
  this->goal_th = yaw;

  this->dist2goal  = sqrt(pow(fabs(goal_x),2)+pow(fabs(goal_y),2));
  this->angle2goal = atan2(goal_y,goal_x);
  this->heading    = tf::getYaw(this->local_plan_.back().pose.orientation);
  if(this->dist2goal<0.1 && fabs(this->heading)<0.1)
  {
    ok=true;
    ROS_INFO("TemplateLocalPlanner::isGoalReached: GOAL REACHED");
  }
  //////////////////////////////////////////////////////
  // TO DO
  //////////////////////////////////////////////////////
  return ok;
}

void TemplateLocalPlanner::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void TemplateLocalPlanner::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{ 
  //ROS_INFO("TemplateLocalPlanner::odom_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); // #threads?
  //this->odom_mutex_.enter(); 

  //std::cout << msg->data << std::endl; 
  this->base_odom_.pose  = msg->pose;
  this->base_odom_.twist = msg->twist;
  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  //this->odom_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TemplateLocalPlanner::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void TemplateLocalPlanner::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TemplateLocalPlanner>(argc, argv, "iri_template_local_planner_alg_node");
}

//copied from goal_functions.cpp in base_local_planner
bool TemplateLocalPlanner::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
      const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
      std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try{
      if (!global_plan.size() > 0)
      {
        ROS_ERROR("Recieved plan with zero length");
        return false;
      }

      tf::StampedTransform transform;
      tf.lookupTransform(global_frame, ros::Time(), 
          plan_pose.header.frame_id, plan_pose.header.stamp, 
          plan_pose.header.frame_id, transform);

      //let's get the pose of the robot in the frame of the plan
      tf::Stamped<tf::Pose> robot_pose;
      robot_pose.setIdentity();
      robot_pose.frame_id_ = costmap.getBaseFrameID();
      robot_pose.stamp_ = ros::Time();
      tf.transformPose(plan_pose.header.frame_id, robot_pose, robot_pose);

      //we'll keep points on the plan that are within the window that we're looking at
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

      unsigned int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = DBL_MAX;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      while(i < (unsigned int)global_plan.size() && sq_dist > sq_dist_threshold){
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        ++i;
      }

      //make sure not to count the first point that is too far away
      if(i > 0)
        --i;

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      //now we'll transform until points are outside of our distance threshold
      while(i < (unsigned int)global_plan.size() && sq_dist < sq_dist_threshold){
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        const geometry_msgs::PoseStamped& pose = global_plan[i];
        poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(transform * tf_pose);
        tf_pose.stamp_ = transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        ++i;
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }