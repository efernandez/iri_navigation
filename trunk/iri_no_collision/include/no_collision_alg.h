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

#ifndef _no_collision_alg_h_
#define _no_collision_alg_h_

#include <iri_no_collision/NoCollisionConfig.h>
#include "mutex.h"

//include no_collision_alg main library
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

typedef enum {traj_acc,traj_speed,traj_deacc,traj_idle} TrajStates;

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class NoCollisionAlgorithm
{
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the NoCollisionConfig. All driver implementations
    * will then use the same variable type Config.
    */
    CMutex alg_mutex_;

    // private attributes and methods
    static const float MIN_LASER_RANGE_ = 0.005f; //[m]
    // node configuration attributes
    float MIN_SAFE_DISTANCE_; // [m]
    float LASER_SAFE_DIST_; // [m]
    float VT_MAX_; // [m/s]
    float T_ACC_; // [m/s2]
    float ANGLE_TOL_; // [rad]

    float MIN_SAFE_ANGLE_; // [rad]
    float VR_MAX_; // [rad/s]
    float R_ACC_; // [rad/s2]
    float DIST_TOL_; // [meters]
  
    float OA_CONE_ANGLE_; // [rad]

    float MAX_ANGLE_TO_STOP_; //[rad]

    // translational parameters
    float current_vt_; // [m/s]
    float target_vt_;//[m/s]
    float desired_vt_;//[m/s]
    float acc_dist_;//[m]
    float deacc_dist_;//[m]
    float target_dist_;//[m]
    float current_dist_;//[m]
    TrajStates vt_state_;

    // rotational parameters
    float current_vr_; // [rad/s]
    float target_vr_;//[rad/s]
    float desired_vr_;//[rad/s]
    float acc_angle_;//[rad]
    float deacc_angle_;//[rad]
    float target_angle_;//rad]
    float current_angle_;//[rad]
    TrajStates vr_state_;
    float heading;//[rad]

    ros::Time current_time_;
   /**
    * \brief cartesian to polar conversion
    *
    * This function converts a cartesian point into module and angle polar values.
    *
    * \param p input cartesian point
    *
    * \param module output polar module
    *
    * \param agnle output polar angle
    */
    void fromCartesian2Polar(const geometry_msgs::Point & p, float & module, float & angle);
                                 
    void fromPolar2Cartesian(const float & module, const float & angle, geometry_msgs::Point &p);

    void updateTranslationTrajParams(void);

    void updateRotationTrajParams(void);
  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the NoCollisionConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_no_collision::NoCollisionConfig Config;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;

   /**
    * \brief constructor
    *
    * In this constructor parameters related to the specific driver can be
    * initalized. Those parameters can be also set in the openDriver() function.
    * Attributes from the main node driver class IriBaseDriver such as loop_rate,
    * may be also overload here.
    */
    NoCollisionAlgorithm(void);

   /**
    * \brief Lock Algorithm
    *
    * Locks access to the Algorithm class
    */
    void lock(void) { alg_mutex_.enter(); };

   /**
    * \brief Unlock Algorithm
    *
    * Unlocks access to the Algorithm class
    */
    void unlock(void) { alg_mutex_.exit(); };

   /**
    * \brief Tries Access to Algorithm
    *
    * Tries access to Algorithm
    * 
    * \return true if the lock was adquired, false otherwise
    */
    bool try_enter(void) { return alg_mutex_.try_enter(); };

   /**
    * \brief config update
    *
    * In this function the driver parameters must be updated with the input
    * config variable. Then the new configuration state will be stored in the 
    * Config attribute.
    *
    * \param new_cfg the new driver configuration state
    *
    * \param level level in which the update is taken place
    */
    void config_update(const Config& new_cfg, uint32_t level=0);

    // here define all no_collision_alg interface methods to retrieve and set
    // the driver parameters
   /**
    * \brief reset distance to goal
    *
    * In this function distance and angle to goal are reset to default to
    * ensure the new request will be tracked properly.
    */
    void setGoal(const geometry_msgs::Pose & local_goal);

   /**
    * \brief goal is reached
    *
    * This function returns true if the distance and angle to the requested 
    * goal are less than previously defined thresholds: MIN_SAFE_DISTANCE,
    * MIN_SAFE_ANGLE.
    *
    * \return true if both distance and angle values are less than thresholds.
    */
    bool isGoalReached(void);

    void getDistance2Goal(geometry_msgs::Pose &current_goal);

    void setTranslationalSpeed(const double vt);

    void setRotationalSpeed(const double vr);

   /**
    * \brief move platform
    *
    * This function computes appropiate Twist values to command a platform given 
    * a laser scan and a local pose goal. Distance and angle to goal are updated
    * with respect the local pose goal. The isGoalTraversable function determines
    * whether the goal is clear of obstacles or not. The Twist command is computed
    * based on distance and angle to goal values.
    *
    * \param scan current laser scan
    *
    * \param local_goal requested pose goal in local coordinates
    *
    * \return Twist linear and angular velocites to command the platform
    */
    bool movePlatform(const sensor_msgs::LaserScan & laser,const geometry_msgs::Pose & local_goal,geometry_msgs::Twist &twist);

   /**
    * \brief goal traversable
    *
    * This function returns true if any of the laser bins detects an obstacle
    * (bin range is less than platform safety_distance).
    *
    * \param laser current laser scan
    *
    * \param safety_distance platfom's safety/clear distance
    *
    * \return bool true if all laser bins are clear of obstacles
    */
    bool isGoalTraversable(const sensor_msgs::LaserScan & laser, 
                           const geometry_msgs::Point & goal);
  
   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~NoCollisionAlgorithm(void);
};

#endif
