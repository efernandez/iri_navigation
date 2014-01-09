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

#ifndef _force_robot_companion_learning_alg_node_h_
#define _force_robot_companion_learning_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "force_robot_companion_learning_alg.h"


// [publisher subscriber headers]
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
//#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <iri_perception_msgs/peopleTrackingArray.h>

// [service client headers]
#include <std_srvs/Empty.h>

// [action server client headers]
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
//#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>
#include "nav/force_reactive_robot_companion_learning.h"
#include <string>

using namespace std;


/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class ForceRobotCompanionLearningAlgNode : public algorithm_base::IriBaseAlgorithm<ForceRobotCompanionLearningAlgorithm>
{
    enum sim_state {Init_Sim=0, Find_Nearest, Test_Sim, End_Sim };
    enum learning_mode {Density=0, Initial_paramters };
  private:
     // [publisher attributes]
    ros::Publisher angle_robot_person_publisher_;
    std_msgs::Float64 angle_robot_person_msg_;
    ros::Publisher person_robot_dist_publisher_;
    std_msgs::Float64 person_robot_dist_msg_;
	ros::Publisher target_person_position_publisher_;
    geometry_msgs::PointStamped target_person_position_msg_;
	ros::Publisher robot_position_publisher_;
    geometry_msgs::PoseStamped robot_position_msg_;
    ros::Publisher force_param_publisher_;
    geometry_msgs::PointStamped force_param_msg_;
    ros::Publisher trajectories_publisher_;
    visualization_msgs::MarkerArray MarkerArray_trajectories_msg_;
    ros::Publisher forces_publisher_;
    visualization_msgs::MarkerArray MarkerArray_forces_msg_;
	ros::Publisher destinations_publisher_;
    visualization_msgs::MarkerArray MarkerArray_destinations_msg_;
	ros::Publisher predictions_publisher_;
    visualization_msgs::MarkerArray MarkerArray_predictions_msg_;
	//variables and functions to fill the visualization messages
	visualization_msgs::Marker traj_marker_, traj_robot_marker_, robot_marker_;
	visualization_msgs::Marker target_marker_;
	visualization_msgs::Marker dest_marker_;
	visualization_msgs::Marker pred_marker_;
	visualization_msgs::Marker text_marker_;
	visualization_msgs::Marker force_marker_, force_goal_marker_, force_goal2_marker_, force_int_person_marker_,
			force_int_robot_marker_, force_obstacle_map_marker_, force_obstacle_laser_marker_;
	void init_node();
	void vis_trajectories();
	void clear_force_markers();
	void vis_intentionality_prediction();
	void vis_destinations();
	void vis_predictions();
  void growing_density_test();
  void initial_params_learning();
	
    // [subscriber attributes]
    ros::Subscriber laser_subscriber_;
    CMutex laser_mutex_;
    ros::Subscriber dest_subscriber_;
    void dest_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    CMutex dest_mutex_;
    ros::Subscriber tracks_subscriber_;
    void tracks_callback(const iri_perception_msgs::peopleTrackingArray::ConstPtr& msg);
    CMutex tracks_mutex_;
	

    // [service attributes]

    // [client attributes]
    ros::ServiceClient reset_client_;
    std_srvs::Empty reset_srv_;

    // [action server attributes]

    // [action client attributes]
//    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
 //   move_base_msgs::MoveBaseGoal move_base_goal_;
  //  void move_baseMakeActionRequest();
   // void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
   // void move_baseActive();
    //void move_baseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

	Cforce_reactive_robot_companion_learning pred_;
	std::string target_frame_, robot_frame_;
	tf::TransformListener tf_listener_;
	
	//for the debugging mode
	SpointV_cov robot_desired_position_, target_followed_pose_;
	Sforce f1_, f2_, f3_, f4_, f5_, f_;
  ros::Time now_;
//	bool move_base_mode_, robot_sim_, using_prediction_, robot_stop_;
	bool using_prediction_;
	double param_f1_, param_f2_, param_f3_;
	std::vector<int> prev_buttons_;
//	bool ready_to_read_laser_; //not finally used??
	
	int nearest_target_;
	string force_map_path_,destination_map_path_;
	double laser_radi_;

	//simulation variables
	sim_state simulation_state_;
	int number_of_virtual_people_,num_experiment_,timer_;
	FILE * file_results_out_;
	bool results_file_is_open_;
	string results_folder_;
	double robot_x_ini_,robot_y_ini_;

    //learning mode
    learning_mode learning_mode_;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    ForceRobotCompanionLearningAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~ForceRobotCompanionLearningAlgNode(void);

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
