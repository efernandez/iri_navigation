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

#ifndef _localization3d_alg_node_h_
#define _localization3d_alg_node_h_

//common headers

//external localization library and iri-ros
#include <iri_base_algorithm/iri_base_algorithm.h>
#include "localization3d_alg.h"
#include "basicPF.h"

// [publisher subscriber headers]
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iri_segway_rmp_msgs/SegwayRMP200Status.h>

// [service client headers]

// [action server client headers]

//CONSTANTS
static const unsigned int MAX_RANGE_DEVICES = 3;
static const string mapFrame = "/map";
static const string odomFrame = "odom";
static const string baseFrame = "base_link";

//specification of a range device (i.e. laser scanner) to be modelled
struct rangeDeviceConfig
{
	//unsigned int deviceId;
	string frameName;
	int deviceType; 
	int nRays;
	double aperture;
	double rangeMin;
	double rangeMax;
	double stdDev;
};

/**
 * \brief IRI ROS Specific Driver Class
 *
 */
class Localization3dAlgNode : public algorithm_base::IriBaseAlgorithm<Localization3dAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher tf_publisher_;
    tf::tfMessage tfMessage_msg_;
    ros::Publisher particleSet_publisher_;
    geometry_msgs::PoseArray PoseArray_msg_;
    ros::Publisher position_publisher_;
    geometry_msgs::PoseWithCovarianceStamped PoseWithCovarianceStamped_msg_;
    
    // [tf broadcaster and listener]
    tf::TransformBroadcaster tfBroadcaster;
    tf::TransformListener tfListener;

    // [subscriber attributes]
    ros::Subscriber platformData_subscriber_;
    void platformData_callback(const iri_segway_rmp_msgs::SegwayRMP200Status::ConstPtr& msg);
    CMutex platformData_mutex_;
    ros::Subscriber platformOdometry_subscriber_;
    void platformOdometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    CMutex platformOdometry_mutex_;
    ros::Subscriber laser0_subscriber_;
    void laser0_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    //CMutex laser0_mutex_;
    ros::Subscriber laser1_subscriber_;
    void laser1_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    //CMutex laser1_mutex_;
    ros::Subscriber laser2_subscriber_;
    void laser2_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    //CMutex laser1_mutex_;
    CMutex laser_mutex[MAX_RANGE_DEVICES];//callback mutexes

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    
    //dynamic reconfigure mutex
    CMutex config_mutex;
    
    //map_localization library objects
    CodometryObservation odometry;
    Ccompass3axisObservation inclinometers;
    ClaserObservation laserObs[MAX_RANGE_DEVICES];
    rangeDeviceConfig rDevConfig[MAX_RANGE_DEVICES];
    CposCovObservation locEstimate;
    CbasicPF *pFilter;
    int numberOfParticles;//number of particles
    int resamplingStyle;
    double initX,initY,initH; //initial estimate (tracking case)
    string mapFileName, gridFileName; //file names for environment models (3D and grid)
    tf::Pose odoPose;//odometry pose starting at 0. Updated at each odometry callback
    ros::Time odoTime; //keeps the last odometry time stamp
    ros::Time priorTime; //keeps the time stamp of the prior estimate (after odometry propagation)
    double infoGain; //accumulates all info gain of each iteration;
    ros::Duration transform_tolerance_;
    double dT; //DEBUGGING!!; this variable should be local at platformOdometry_callback();
    
    void initDevicePositions();//initializes device positions with respect to base_link frame. It is assumed that they are constant during the operation
    void printUserConfiguration();//prints user configurations

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    Localization3dAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~Localization3dAlgNode(void);

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
    
    //other member functions
    //setLaserObservation(const sensor_msgs::LaserScan::ConstPtr& msg, ClaserObservation & laserData);
};

#endif
