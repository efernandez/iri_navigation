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

#ifndef _people_follower_alg_node_h_
#define _people_follower_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "people_follower_alg.h"

// [publisher subscriber headers]
#include <geometry_msgs/PoseStamped.h>
#include <iri_people_tracking/peopleTrackingArray.h>

// [service client headers]

// [action server client headers]
#include <iri_action_server/iri_action_server.h>
#include <iri_nav_msgs/followTargetAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "eventserver.h"

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PeopleFollowerAlgNode : public algorithm_base::IriBaseAlgorithm<PeopleFollowerAlgorithm>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]
    ros::Subscriber target_pose_subscriber_;
    void target_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    CMutex target_pose_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]
    IriActionServer<iri_nav_msgs::followTargetAction> followTarget_aserver_;
    void followTargetStartCallback(const iri_nav_msgs::followTargetGoalConstPtr& goal);
    void followTargetStopCallback(void);
    bool followTargetIsFinishedCallback(void);
    bool followTargetHasSucceedCallback(void);
    void followTargetGetResultCallback(iri_nav_msgs::followTargetResultPtr& result);
    void followTargetGetFeedbackCallback(iri_nav_msgs::followTargetFeedbackPtr& feedback);

    // [action client attributes]
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBase_client_;
    void moveBaseMakeActionRequest(const move_base_msgs::MoveBaseGoal & goal);
    void moveBaseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
    void moveBaseActive();
    void moveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    // Event server manager
    CEventServer * event_server_;

    // Events identifiers
    std::string new_req_event_id_;
    std::string moveBase_done_event_id_;
    std::string moveBase_preempt_event_id_;
    std::string target_pose_received_event_id_;
    std::string follow_target_done_event_id_;

    typedef enum {IDLE_STATE, REQ_MB_STATE, FOLLOWING_STATE, SUCCESS_STATE, UPDATE_STATE, PREEMPT_STATE} states;
    states current_state_;

    unsigned int request_id_;
    float dist_to_goal_;
    unsigned int no_target_pose_msg_received_;
    static const unsigned int MAX_ITERS_NO_MSG = 200;

    std::string tf_prefix_;
    std::string target_frame_;
    std::string fixed_frame_;
    
    geometry_msgs::PoseStamped global_target_goal_;
    geometry_msgs::PoseStamped current_global_target_pose_;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PeopleFollowerAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PeopleFollowerAlgNode(void);

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
