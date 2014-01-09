// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _iri_template_local_planner_alg_node_h_
#define _iri_template_local_planner_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "iri_template_local_planner_alg.h"

#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

// [publisher subscriber headers]
#include <nav_msgs/Odometry.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class TemplateLocalPlanner : 
  public algorithm_base::IriBaseAlgorithm<IriTemplateLocalPlannerAlgorithm>,
  public nav_core::BaseLocalPlanner
{
  private:
    // [publisher attributes]

    // [subscriber attributes]
    ros::Subscriber odom_subscriber_;
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    CMutex odom_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

    tf::TransformListener* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    nav_msgs::Odometry base_odom_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> local_plan_;
    double goal_x;
    double goal_y;
    double goal_th;
    double dist2goal;
    double angle2goal;
    double heading;

    bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
      const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
      std::vector<geometry_msgs::PoseStamped>& transformed_plan);

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    TemplateLocalPlanner(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~TemplateLocalPlanner(void);

    /**
      * @brief  Constructs the ros wrapper
      * @param name The name to give this instance of the trajectory planner
      * @param tf A pointer to a transform listener
      * @param costmap The cost map to use for assigning costs to trajectories
      */
    void initialize(std::string name, tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmap_ros);

    /**
      * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
      * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
      * @return True if a valid trajectory was found, false otherwise
      */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
      * @brief  Set the plan that the controller is following
      * @param orig_global_plan The plan to pass to the controller
      * @return True if the plan was updated successfully, false otherwise
      */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
      * @brief  Check if the goal pose has been achieved
      * @return True if achieved, false otherwise
      */
    bool isGoalReached();

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
