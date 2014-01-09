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

#ifndef _laser_navigation_alg_node_h_
#define _laser_navigation_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "laser_navigation_alg.h"

// read bag headers
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

// read pose
#include <tf/transform_datatypes.h>

// [publisher subscriber headers]
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

// [service client headers]
#include <iri_laser_localisation/DoLocalisation.h>

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class LaserNavigationAlgNode : public algorithm_base::IriBaseAlgorithm<LaserNavigationAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher scans_map_publisher_;
    sensor_msgs::LaserScan LaserScan_msg_;
    ros::Publisher checkpoints_publisher_;
    visualization_msgs::Marker Marker_msg_;

    // [subscriber attributes]
    ros::Subscriber scan_subscriber_;
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    CMutex scan_mutex_;

    // [service attributes]

    // [client attributes]
    ros::ServiceClient localise_client_;
    iri_laser_localisation::DoLocalisation localise_srv_;

    // [action server attributes]

    // [action client attributes]
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> send_goal_client_;
    void send_goalMakeActionRequest(const geometry_msgs::PoseStamped & new_goal);
    void send_goalDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
    void send_goalActive();
    void send_goalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    // read bag
    bool load_path_(const std::string & bag_path);

    sensor_msgs::LaserScan scan_sens_;
    std::vector<sensor_msgs::LaserScan> scan_path_;
    std::vector<geometry_msgs::PoseStamped> pose_path_;
    uint current_;
    bool new_scan_;
    bool waiting_;
    bool first_;
    void publish_marker(const geometry_msgs::PoseStamped & pose, const int & type);

  public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    LaserNavigationAlgNode(void);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~LaserNavigationAlgNode(void);

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

    void ROS_INFO_PRESS(const std::string & str);
    void ROS_INFO_XYR(const std::string & str,const float & x,const float & y,const geometry_msgs::Quaternion & r);


    // [diagnostic functions]

    // [test functions]
};

#endif
