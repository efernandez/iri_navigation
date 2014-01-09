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

#ifndef _platform_teleop_alg_node_h_
#define _platform_teleop_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "platform_teleop_alg.h"

// [publisher subscriber headers]
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PlatformTeleopAlgNode : public algorithm_base::IriBaseAlgorithm<PlatformTeleopAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher cmd_vel_publisher_;

    // [subscriber attributes]
    ros::Subscriber joy_subscriber_;
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
    CMutex joy_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

   /**
    * \brief translational velocity
    *
    * Translational platform velocity in [m/s] to be published with twist.
    */
    float vT_;

   /**
    * \brief rotational velocity
    *
    * Rotational platform velocity in [rad/s] to be published with twist.
    */
    float vR_;

   /**
    * \brief translational increment velocity
    *
    * Translational platform velocity in [m/s] to increment vT_.
    */
    float dVT_;

   /**
    * \brief translational increment velocity
    *
    * Rotational platform velocity in [rad/s] to increment vR_.
    */
    float dVR_;

   /**
    * \brief last pressed joysting buttons
    *
    * Vector with indexes to last joystick pressed buttons to avoid multiple 
    * commands without releasing a button.
    */
    std::vector<int> prev_buttons_;
    
   /**
    * \brief last used joystick axes
    *
    * Vector with indexes to last used axes to avoid multiple 
    * commands without releasing a button.
    */
    std::vector<int> prev_axes_;

    /**
    * \brief check or not if human is alive
    *
    * If this option is true, the user has to press the button B all time 
    * in order to give velocity to the robot
    */
    bool check_human_;

    /**
    * \brief Tells if the human user is alive
    *
    * Depending on the button B this will be true or false. This is an option 
    * through check_human_
    */
    bool human_is_alive_;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PlatformTeleopAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PlatformTeleopAlgNode(void);

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
    
   /**
    * \brief check used axes
    *
    * This function receives a vector with the state from all joystick axes.
    * If none of the axes has been used, returns false, otherwise returns a 
    * vector with the indexes for the used axes.
    *
    * \param axes vector with the current state of each axe
    * \param index vector with the indexes for each used axe
    * \return bool true if at least one axe has been updated, false otherwise
    */
    static bool check_movement_axes_callback(const std::vector<float> & axes, std::vector<unsigned int> & index);
    
   /**
    * \brief generate twist
    *
    * This function fills up a twist message with current translational and
    * rotational velocities (vT, vR).
    *
    * \return Twist twist message with last updated state
    */
    geometry_msgs::Twist generateTwist(void);
    
   /**
    * \brief use button
    *
    * This function receives a single button index and updates current state
    * according to the behavior defined. Updates velocities according to axes
    * and calls useExtraButton for further actions with other buttons.
    *
    * \param index joystick button index
    */
    bool useButton(const unsigned int & index);
    
   /**
    * \brief use extra button
    *
    * This function receives a single button index and updates current state
    * according to the behavior defined. Called by useButton, for pruposes
    * beyond navigation.
    *
    * \param index joystick button index
    */
    void useExtraButton(const unsigned int & index);
    
   /**
    * \brief use button
    *
    * This function receives the value of a single axe and its index
    * and triggers corresponding action defined.
    *
    * \param axe_value axe value
    * \param index joystick button index
    */
    void useAxes(const float & axe_value, const unsigned int & index);
};

#endif
