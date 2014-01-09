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

#ifndef _no_collision_alg_node_h_
#define _no_collision_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "no_collision_alg.h"

// [publisher subscriber headers]
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// [service client headers]

// [action server client headers]
#include <iri_action_server/iri_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class NoCollisionAlgNode : public algorithm_base::IriBaseAlgorithm<NoCollisionAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher goal_marker_publisher_;
    visualization_msgs::Marker goal_marker_;
    ros::Publisher segway_cmd_publisher_;
    
    // [subscriber attributes]
    ros::Subscriber target_vel_subscriber_;
    void target_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    CMutex target_vel_mutex_;
    ros::Subscriber frontal_laser_subscriber_;
    void frontal_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    sensor_msgs::LaserScan scan_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]
    IriActionServer<move_base_msgs::MoveBaseAction> move_base_aserver_;
    void startCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    void stopCallback(void);
    bool isFinishedCallback(void);
    bool hasSucceedCallback(void);
    void getResultCallback(move_base_msgs::MoveBaseResultPtr& result);
    void getFeedbackCallback(move_base_msgs::MoveBaseFeedbackPtr& feedback);

    // [action client attributes]

    tf::TransformListener tf_listener_, tf_listener2_;
    geometry_msgs::PoseStamped goal_pose_;
    ros::Time action_start_;
    static const double ACTION_TIMEOUT = 300; //[s]
    bool is_laser_ready_;
    bool action_running_;

    std::string target_frame_;
    std::string fixed_frame_;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    NoCollisionAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~NoCollisionAlgNode(void);

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
