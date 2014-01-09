/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <trajectory_planner_ros.h>
#include <ros/console.h>
#include <sys/time.h>
#include <pluginlib/class_list_macros.h>
#include <boost/tokenizer.hpp>

#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"
#include "goal_functions.h"

using namespace std;
using namespace costmap_2d;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(iri_ackermann_local_planner, TrajectoryPlannerROS, iri_ackermann_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace iri_ackermann_local_planner {

  void TrajectoryPlannerROS::reconfigureCB(AckermannLocalPlannerConfig &config, uint32_t level) {
      if(setup_ && config.restore_defaults) {
        config = default_config_;
        //Avoid looping
        config.restore_defaults = false;
      }
      if(!setup_) {
        default_config_ = config;
        setup_ = true;
      }
      else if(setup_) {
        tc_->reconfigure(config);
      }
  }

  TrajectoryPlannerROS::TrajectoryPlannerROS() : world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), initialized_(false), setup_(false) 
  {
  }

  TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name, tf::TransformListener* tf, Costmap2DROS* costmap_ros) 
    : world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), initialized_(false), setup_(false) {
      //initialize the planner
      initialize(name, tf, costmap_ros);
  }

  void TrajectoryPlannerROS::initialize(std::string name, tf::TransformListener* tf, Costmap2DROS* costmap_ros){
    if(!initialized_)
    {
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double sim_time, sim_granularity, angular_sim_granularity;
      int vx_samples, vtheta_samples;
      double pdist_scale, gdist_scale, occdist_scale, hdiff_scale;
      double ack_acc_max,ack_vel_max,ack_vel_min;
      double ack_steer_acc_max,ack_steer_vel_max,ack_steer_vel_min,ack_steer_angle_max,ack_steer_angle_min;
      double axis_distance;
      bool simple_attractor;
      int heading_points;
      string world_model_type;
      rotating_to_goal_ = false;

      //initialize the copy of the costmap the controller will use
      costmap_ros_->getCostmapCopy(costmap_);

      ros::NodeHandle private_nh("~/" + name);
      
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      private_nh.param("prune_plan", prune_plan_, true);

      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);

      //we'll get the parameters for the robot radius from the costmap we're associated with
      inflation_radius_ = costmap_ros_->getInflationRadius();

      private_nh.param("ack_acc_max", ack_acc_max, 1.0);
      private_nh.param("ack_vel_max", ack_vel_max, 0.6);
      private_nh.param("ack_vel_min", ack_vel_min, -0.6);
      private_nh.param("ack_steer_acc_max", ack_steer_acc_max, 1.0);
      private_nh.param("ack_steer_speed_max", ack_steer_vel_max, 0.5);
      private_nh.param("ack_steer_speed_min", ack_steer_vel_min, -0.5);
      private_nh.param("ack_steer_angle_max", ack_steer_angle_max, 0.35);
      private_nh.param("ack_steer_angle_min", ack_steer_angle_min, -0.35);
      private_nh.param("ack_axis_distance", axis_distance, 1.65);

      private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

      //Assuming this planner is being run within the navigation stack, we can
      //just do an upward search for the frequency at which its being run. This
      //also allows the frequency to be overwritten locally.
      std::string controller_frequency_param_name;

      private_nh.param("sim_time", sim_time, 10.0);
      private_nh.param("sim_granularity", sim_granularity, 0.025);
      private_nh.param("angular_sim_granularity", angular_sim_granularity, sim_granularity);
      private_nh.param("vx_samples", vx_samples, 10);//3
      private_nh.param("vtheta_samples", vtheta_samples, 20);

      private_nh.param("pdist_scale", pdist_scale, 0.6);
      private_nh.param("gdist_scale", gdist_scale, 0.8);
      private_nh.param("hdiff_scale", hdiff_scale, 1.0);
      private_nh.param("occdist_scale", occdist_scale, 0.01);
      private_nh.param("heading_points", heading_points, 8);

      private_nh.param("world_model", world_model_type, string("costmap")); 

      simple_attractor = false;

      //parameters for using the freespace controller
      double min_pt_separation, max_obstacle_height, grid_resolution;
      private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
      private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
      private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
      private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

      ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
      world_model_ = new CostmapModel(costmap_);

      tc_ = new TrajectoryPlanner(*world_model_, costmap_, costmap_ros_->getRobotFootprint(),
          ack_acc_max, ack_vel_max, ack_vel_min, ack_steer_acc_max,ack_steer_vel_max,ack_steer_vel_min,ack_steer_angle_max,ack_steer_angle_min,axis_distance,
          sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,gdist_scale, occdist_scale, hdiff_scale,simple_attractor,angular_sim_granularity,
          heading_points,xy_goal_tolerance_);

      map_viz_.initialize(name, &costmap_, boost::bind(&TrajectoryPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<AckermannLocalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<AckermannLocalPlannerConfig>::CallbackType cb = boost::bind(&TrajectoryPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);

      ros::NodeHandle global_node;
      odom_sub_ = global_node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&TrajectoryPlannerROS::odomCallback, this, _1));
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS(){
    //make sure to clean things up
    delete dsrv_;

    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;
  }

  void TrajectoryPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::recursive_mutex::scoped_lock lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ackermann_state_.linear.z=msg->twist.twist.linear.z;
    ackermann_state_.angular.x=msg->twist.twist.angular.x;
    ackermann_state_.angular.y=msg->twist.twist.angular.y;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
  {
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    ////divide plan by manuveurs
    subPathList.clear();
    subPath.clear();
    subPathIndex      = 0;
    double pathLength = 0;
    for(unsigned int i = 0; i < orig_global_plan.size(); ++i)
    {
      if(i>1 && i<orig_global_plan.size())
      {
        double x0 = orig_global_plan[i  ].pose.position.x;
        double x1 = orig_global_plan[i-1].pose.position.x;
        double x2 = orig_global_plan[i-2].pose.position.x;
        double y0 = orig_global_plan[i  ].pose.position.y;
        double y1 = orig_global_plan[i-1].pose.position.y;
        double y2 = orig_global_plan[i-2].pose.position.y;
        double angle=std::acos(((x0-x1)*(x1-x2)+(y0-y1)*(y1-y2))/(std::sqrt(std::pow(x0-x1,2)+std::pow(y0-y1,2))*std::sqrt(std::pow(x1-x2,2)+std::pow(y1-y2,2))));
        pathLength+= std::sqrt(std::pow(x0-x1,2)+std::pow(y0-y1,2));
        if(fabs(angle)>1.57) //if changes of direction detected
        {
          if(pathLength>1.0)
          {
            ROS_INFO("TrajectoryPlannerROS::setPlan: subPath split at i=%d, angle=%f, length=%f", i, angle, pathLength);
            subPathList.push_back(subPath);
          }
          else //ignored subpaths shorter than 1.0m
          {
            ROS_INFO("TrajectoryPlannerROS::setPlan: subPath split at i=%d, angle=%f, Ignored by length=%f", i, angle, pathLength);
          }
          subPath.clear();
          pathLength=0.0;
        }
      }
      subPath.push_back(orig_global_plan[i]);
    }
    subPathList.push_back(subPath);
    ROS_INFO("TrajectoryPlannerROS::setPlan: subPath last length=%f", pathLength);
    ROS_INFO("TrajectoryPlannerROS::setPlan: Global plan (%lu points) split in %lu paths", orig_global_plan.size(), subPathList.size());
    global_plan_.clear();
    global_plan_ = subPathList[subPathIndex];
    ////

    //reset the global plan
    ////global_plan_.clear();
    ////global_plan_ = orig_global_plan;

    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;

    return true;
  }

  bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    ROS_INFO("TrajectoryPlannerROS::computeVelocityCommands!");

    std::vector<geometry_msgs::PoseStamped> local_plan;
    tf::Stamped<tf::Pose> global_pose;
    if(!costmap_ros_->getRobotPose(global_pose))
      return false;

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if(!transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, global_frame_, transformed_plan)){
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      prunePlan(global_pose, transformed_plan, global_plan_);


    //we also want to clear the robot footprint from the costmap we're using
    costmap_ros_->clearRobotFootprint();

    //make sure to update the costmap we'll use for this cycle
    costmap_ros_->getCostmapCopy(costmap_);

    // Set current velocities from odometry
    geometry_msgs::Twist global_vel;

    odom_lock_.lock();
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;
    odom_lock_.unlock();

    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = robot_base_frame_;

    tf::Stamped<tf::Pose> robot_vel;
    robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
    robot_vel.frame_id_ = robot_base_frame_;
    robot_vel.stamp_ = ros::Time();

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double yaw = tf::getYaw(goal_point.getRotation());

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if(goalPositionReached(global_pose, goal_x, goal_y, xy_goal_tolerance_) || xy_tolerance_latch_){

      ////check if there are manuveurs remaining
      if(subPathIndex < subPathList.size()-1)
      {
        subPathIndex++;
        global_plan_.clear();
        global_plan_ = subPathList[subPathIndex];
        return true;
      }
      ////

      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if(latch_xy_goal_tolerance_)
        xy_tolerance_latch_ = true;

      //check to see if the goal orientation has been reached
      if(goalOrientationReached(global_pose, goal_th, yaw_goal_tolerance_)){
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
      }

      //publish an empty plan because we've reached our goal position
      publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    tc_->updatePlan(transformed_plan);

    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds,ackermann_state_);

    ROS_INFO("Speed command x,y,yaw: %f,%f,%f",drive_cmds.getOrigin().getX(),drive_cmds.getOrigin().getY(),tf::getYaw(drive_cmds.getRotation()));

    map_viz_.publishCostCloud();

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    yaw = tf::getYaw(drive_cmds.getRotation());

    cmd_vel.angular.z = yaw;

    //if we cannot move... tell someone
    if(path.cost_ < 0){
      local_plan.clear();
      publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
      return false;
    }

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(p_th), tf::Point(p_x, p_y, 0.0)), ros::Time::now(), global_frame_);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
    publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
    return true;
  }

  bool TrajectoryPlannerROS::checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    tf::Stamped<tf::Pose> global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we also want to clear the robot footprint from the costmap we're using
        costmap_ros_->clearRobotFootprint();

        //make sure to update the costmap we'll use for this cycle
        costmap_ros_->getCostmapCopy(costmap_);

        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::PoseStamped> plan;
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        plan.push_back(pose_msg);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      nav_msgs::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->checkTrajectory(global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

    }
    ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return false;
  }


  double TrajectoryPlannerROS::scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    // Copy of checkTrajectory that returns a score instead of True / False
    tf::Stamped<tf::Pose> global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we also want to clear the robot footprint from the costmap we're using
        costmap_ros_->clearRobotFootprint();

        //make sure to update the costmap we'll use for this cycle
        costmap_ros_->getCostmapCopy(costmap_);

        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::PoseStamped> plan;
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        plan.push_back(pose_msg);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      nav_msgs::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->scoreTrajectory(global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

    }
    ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return -1.0;
  }

  bool TrajectoryPlannerROS::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::recursive_mutex::scoped_lock lock(odom_lock_);
      base_odom = base_odom_;
    }

    return iri_ackermann_local_planner::isGoalReached(*tf_, global_plan_, *costmap_ros_, global_frame_, base_odom, 
        rot_stopped_velocity_, trans_stopped_velocity_, xy_goal_tolerance_, yaw_goal_tolerance_);
  }
};
