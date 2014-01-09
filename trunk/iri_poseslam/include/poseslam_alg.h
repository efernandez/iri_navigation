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

#ifndef _poseslam_alg_h_
#define _poseslam_alg_h_

#include <iri_poseslam/PoseslamConfig.h>
#include "mutex.h"
#include "poseSLAM.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//include poseslam_alg main library

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class PoseslamAlgorithm
{
  protected:
   
    CMutex alg_mutex_;
    

    // private attributes and methods

  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the PoseslamConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_poseslam::PoseslamConfig Config;

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
    PoseslamAlgorithm(void);

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
    void config_update(Config& new_cfg, uint32_t level=0);

    /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~PoseslamAlgorithm(void);
    
    // here define all poseslam_alg interface methods to retrieve and set
    // the driver parameters
    
    /**
    * \brief augmentation
    *
    * Does the state augmentation of the FlashFilter overwritting the previous pose
    * if was redundant.
    *
    * \param step the step index of the new pose
    *
    * \param d the odometry with the previous pose
    *
    * \param Q the covariance of the odometry
    */
    void augmentation(const uint& step, const geometry_msgs::PoseWithCovarianceStamped& odom);
    
    /**
    * \brief create candidates list
    *
    * Creates a list of the link candidates for loop closure with actual pose.
    *
    * \param realOdometryCov if the information gain of link with previous must take the real covariance of the sensor
    */
    void create_candidates_list(const bool& realOdometryCov);
    
    /**
    * \brief redundant evaluation
    *
    * Executes the conditions of a pose for be redundant for overwriting the
    * next pose if it's the case.
    *
    */
    void redundant_evaluation();
    
    /**
    * \brief loop closure requeriments
    *
    * Executes the conditions of a link candidate for try to close the loop.
    *
    * \return true if the link can be a loop closed, false otherwise
    */
    bool loop_closure_requeriments() const;
    
    /**
    * \brief try loop closure
    *
    * Try to close a loop if it's information gain enough with the real values of 
    * odometry and its noise and update the filter.
    *
    * \param LoopClosureNoise the real covariance of the odometry of the loop closure
    *
    * \param LoopClosureD the real odometry of the loop closure
    *
    * \return true if the loop closure, false otherwise
    */
    bool try_loop_closure(const geometry_msgs::PoseWithCovarianceStamped& odom);
    
    /**
    * \brief update candidates list
    *
    * Update candidates list.
    *
    * \param LoopClosure if the loop was closed
    *
    * \param realOdometryCov if the information gain of link with previous must take the real covariance of the sensor
    */
    void update_candidates_list(const bool LoopClosure, const bool& realOdometryCov);
    
    /**
    * \brief any candidate
    *
    * Check if the candidate's list is empty or not.
    *
    * \return true if there is any candidate, false otherwise
    */
    bool any_candidate() const;
    
    /**
    * \brief select best candidate
    *
    * Select the best candidate of the list for trying loop closure.
    *
    */
    void select_best_candidate();
    
    /**
    * \brief get candidate step
    *
    * Get the step of current candidate.
    * 
    * \return index of current candidate
    */
    uint get_candidate_step() const;
    
    /**
    * \brief get candidate d
    *
    * Get the displacement of current link candidate.
    * 
    * \return d of current link candidate
    */
    geometry_msgs::Pose get_candidate_d() const;
    
    /**
     * \brief get candidate Information Gain
     *
     * Get the information gain of current link candidate.
     *
     * \return the information gain of the candidate
     */
    double get_candidate_ig() const;
    
    /**
    * \brief is redundant
    *
    * Check if last pose was redundant.
    * 
    * \return true if last pose was redundant, false otherwise
    */
    bool is_redundant() const;
    
    /**
    * \brief get trajectory
    *
    * Get all not-redundant positions in the trajectory.
    * 
    * \return std::vector of eigen::VectorXd of all non-redundant positions in trajectory
    */
    std::vector<VectorXd> get_trajectory() const;
    
    /**
    * \brief get trajectory covariance
    *
    * Get all not-redundant positions' covariances in the trajectory.
    * 
    * \return std::vector double of all non-redundant positions' covariances in trajectory
    */
    std::vector<std::vector<double> >  get_trajectory_covariance() const;
    
    /**
    * \brief get trajectory steps
    *
    * Get all not-redundant positions index in the trajectory.
    * 
    * \return std::vector of integers, indexs of all non-redundant positions in trajectory
    */
    std::vector<uint> get_trajectory_steps() const;
    
    /**
    * \brief get trajectory no loops
    *
    * Get all positions in the trajectorywith no loop closure.
    * 
    * \return std::vector of row-major matrix floats of all positions in no the loop closure trajectory
    */
    std::vector<VectorXd> get_trajectory_no_loops() const;
    
    /**
    * \brief get trajectory covariance no loops
    *
    * Get all positions' covariances in the trajectory with no loop closure.
    * 
    * \return std::vector double of all non-redundant positions' covariances in trajectory
    */
    std::vector<std::vector<double> >  get_trajectory_covariance_no_loops() const;
    
    /**
    * \brief get last pose
    *
    * Get last not-redundant position in the trajectory.
    * 
    * \return vector of last non-redundant position in trajectory
    */
    VectorXd get_last_pose() const;

    /**
    * \brief get last covariance
    *
    * Get last not-redundant position covariance in the trajectory.
    * 
    * \return std::vector double of last non-redundant position' covariance in trajectory
    */
    std::vector<double> get_last_covariance() const;

    /**
    * \brief get last step
    *
    * Get last not-redundant position index in the trajectory.
    * 
    * \return integer, index of last non-redundant position in trajectory
    */
    uint get_last_step() const;

    /**
    * \brief get last pose no loops
    *
    * Get last not-redundant position in the trajectory with no loop closure.
    * 
    * \return vector of last non-redundant position in trajectory
    */
    VectorXd get_last_pose_no_loops() const;

    /**
    * \brief get last covariance no loops
    *
    * Get last not-redundant position covariance in the trajectory with no loop closure.
    * 
    * \return std::vector double of last non-redundant position' covariance in trajectory
    */
    std::vector<double> get_last_covariance_no_loops() const;

    /**
    * \brief get nStates
    *
    * Get number of non redundant steps.
    * 
    * \return integer
    */
    uint get_nStates() const;
    
    /**
    * \brief print matrix in file
    *
    * Prints the given matrix in a matlab file.
    * \param M Matrix to be printed
    *
    * \param file M-File to print
    */
    void print_matrix_in_file(const MatrixXd& M, std::fstream& file) const;    
    
    /**
    * \brief print vector in file
    *
    * Prints the given matrix in a matlab file.
    * \param v std::vector<float> to be printed
    *
    * \param file M-File to print
    */
    void print_vector_in_file(const std::vector<double>& v, std::fstream& file) const;    
    
    /**
    * \brief print vector in file
    *
    * Prints the given matrix in a matlab file.
    * \param v std::vector<int> to be printed
    *
    * \param file M-File to print
    */
    void print_vector_in_file(const std::vector<uint>& v, std::fstream& file) const;    
    
    /**
    * \brief print time file
    *
    * Prints all computation time data in a matlab file.
    * 
    * \param file M-File to print
    */
    void print_time_file(std::fstream& file) const;    
    
    //Create object
    CPoseSLAM* pose_SLAM_;
    CPoseSLAM* pose_SLAM_noLoops_;
    bool inicialitzat_;
    std::vector<Eigen::MatrixXd> Q_aug_;
    std::vector<Eigen::MatrixXd> Q_loop_;
    std::vector<Eigen::VectorXd> d_aug_;
    std::vector<Eigen::VectorXd> d_loop_;
    std::vector<bool> success_loop_;
    Eigen::MatrixXd Q_odom_;
    std::vector<double> timePub;
    std::vector<double> timeSensOpen;
    std::vector<double> timeSensClose;
    double tPub, tSens;
};

#endif
