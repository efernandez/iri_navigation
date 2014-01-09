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

#ifndef _ry_oa_bridge_alg_node_h_
#define _ry_oa_bridge_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "ry_oa_bridge_alg.h"

//yarp headers
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

// [publisher subscriber headers]
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iri_ry_oa_bridge/oaStatus.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// [service client headers]

// [action server client headers]

using namespace yarp::os;

class CvelocityCommandPort : public BufferedPort<Bottle>
{
	protected:
		double vv,ww;
		CMutex velocities_mutex_;
	
	public:
		virtual void onRead(Bottle& bot) //callback function
		{
			unsigned int ii=0;
			velocities_mutex_.enter();
			vv = bot.get(ii++).asDouble();
			ww = bot.get(ii++).asDouble();
			velocities_mutex_.exit();
		}	
		double getV()
		{
			velocities_mutex_.enter();
			return vv;
			velocities_mutex_.exit();
		}
		double getW()
		{
			velocities_mutex_.enter();
			return ww;
			velocities_mutex_.exit();
		}
};

class CoaStatusPort : public BufferedPort<Bottle>
{
	protected:
		int status, state;
		CMutex st_mutex_;
	
	public:
		virtual void onRead(Bottle& bot) //callback function
		{
			unsigned int ii=0;
			st_mutex_.enter();
			status = bot.get(ii++).asInt();
			state = bot.get(ii++).asInt();
			st_mutex_.exit();
		}	
		int getStatus()
		{
			st_mutex_.enter();
			return status;
			st_mutex_.exit();
		}
		int getState()
		{
			st_mutex_.enter();
			return state;
			st_mutex_.exit();
		}
};

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class RyOaBridgeAlgNode : public algorithm_base::IriBaseAlgorithm<RyOaBridgeAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher status_publisher_;
    iri_ry_oa_bridge::oaStatus oaStatus_msg_;
    ros::Publisher command_publisher_;
    geometry_msgs::Twist Twist_msg_;

    // [subscriber attributes]
    ros::Subscriber localization_subscriber_;
    void localization_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    CMutex localization_mutex_;
    ros::Subscriber odometry_subscriber_;
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    CMutex odometry_mutex_;
    ros::Subscriber vertical_laser_subscriber_;
    void vertical_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    CMutex vertical_laser_mutex_;
    ros::Subscriber front_laser_subscriber_;
    void front_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    CMutex front_laser_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    
    //yarp comm's. In/Out tag from the point of view of this node
    BufferedPort<Bottle> yarpOdometryOut;
    BufferedPort<Bottle> yarpVerticalLaserOut;
    BufferedPort<Bottle> yarpFrontLaserOut;
    BufferedPort<Bottle> yarpLocalizationOut;
    CvelocityCommandPort yarpVelocitiesIn;
    CoaStatusPort yarpOaStatusIn;
    
    double lastOdoTs; //last odometry time stamp;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    RyOaBridgeAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~RyOaBridgeAlgNode(void);

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
