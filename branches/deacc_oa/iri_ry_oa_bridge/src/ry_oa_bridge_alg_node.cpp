#include "ry_oa_bridge_alg_node.h"

RyOaBridgeAlgNode::RyOaBridgeAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<RyOaBridgeAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->status_publisher_ = this->public_node_handle_.advertise<iri_ry_oa_bridge::oaStatus>("status", 100);
  this->command_publisher_ = this->public_node_handle_.advertise<geometry_msgs::Twist>("command", 100);
  
  // [init subscribers]
  this->localization_subscriber_ = this->public_node_handle_.subscribe("localization", 100, &RyOaBridgeAlgNode::localization_callback, this);
  this->odometry_subscriber_ = this->public_node_handle_.subscribe("odometry", 100, &RyOaBridgeAlgNode::odometry_callback, this);
  this->vertical_laser_subscriber_ = this->public_node_handle_.subscribe("vertical_laser", 100, &RyOaBridgeAlgNode::vertical_laser_callback, this);
  this->front_laser_subscriber_ = this->public_node_handle_.subscribe("front_laser", 100, &RyOaBridgeAlgNode::front_laser_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  
  //yarp init's
  Network yarp;
  yarpOdometryOut.open("/oa_bridge/odometry");
  yarpVerticalLaserOut.open("/oa_bridge/verticalLaser");
  yarpFrontLaserOut.open("/oa_bridge/frontLaser");
  yarpLocalizationOut.open("/oa_bridge/localization");
  yarpVelocitiesIn.open("/oa_bridge/velocities");
  yarpVelocitiesIn.useCallback();
  yarpOaStatusIn.open("/oa_bridge/status");
  yarpOaStatusIn.useCallback();
  
  //other init's
  lastOdoTs = ros::Time::now().toSec();
}

RyOaBridgeAlgNode::~RyOaBridgeAlgNode(void)
{
  // [free dynamic memory]
}

void RyOaBridgeAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->oaStatus_msg.data = my_var;
  //this->Twist_msg.data = my_var;
  this->oaStatus_msg_.status = yarpOaStatusIn.getStatus();
  this->oaStatus_msg_.state = yarpOaStatusIn.getState();
  this->Twist_msg_.linear.x = yarpVelocitiesIn.getV();
  this->Twist_msg_.linear.y = 0;
  this->Twist_msg_.linear.z = 0;
  this->Twist_msg_.angular.x = 0;
  this->Twist_msg_.angular.y = 0;
  this->Twist_msg_.angular.z = yarpVelocitiesIn.getW();
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->status_publisher_.publish(this->oaStatus_msg_);
  this->command_publisher_.publish(this->Twist_msg_);
}

/*  [subscriber callbacks] */
void RyOaBridgeAlgNode::localization_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
{ 
	double now, dT;
	
	Bottle & botW = yarpLocalizationOut.prepare(); 
	botW.clear();
	botW.addInt(2);//indicates basic success
	botW.addDouble(msg->header.stamp.toSec());
	botW.addInt(0); 
	botW.addInt(0);
	botW.addDouble(msg->pose.pose.position.x);
	botW.addDouble(msg->pose.pose.position.y);
	botW.addDouble(msg->pose.pose.position.z);
	//fill orientation if required by ry_path_execution
	//...
	yarpLocalizationOut.write();

//   ROS_INFO("RyOaBridgeAlgNode::localization_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->localization_mutex_.enter(); 

  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->localization_mutex_.exit(); 
}
void RyOaBridgeAlgNode::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{ 
	double now, dT;
	
	Bottle & botW = yarpOdometryOut.prepare(); 
	botW.clear();
	botW.addInt(2);//indicates basic success
	botW.addDouble(msg->header.stamp.toSec());
	botW.addInt(2); //indicates two-wheel balanced
	now = msg->header.stamp.toSec();
	dT = now - lastOdoTs;
	lastOdoTs = now;
	botW.addDouble(dT*msg->twist.twist.linear.x);
	botW.addDouble(dT*msg->twist.twist.angular.z);
	botW.addDouble(msg->twist.twist.linear.x);
	botW.addDouble(msg->twist.twist.angular.z);
	botW.addDouble(-1);//torque1
	botW.addDouble(-1);//torque2
	botW.addDouble(-1);//voltage1
	botW.addDouble(-1);//voltage2
	botW.addDouble(-1);//reserved1
	botW.addDouble(-1);//reserved2
	yarpOdometryOut.write();

  //ROS_INFO("RyOaBridgeAlgNode::odometry_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->odometry_mutex_.enter(); 
  
  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->odometry_mutex_.exit(); 
}
void RyOaBridgeAlgNode::vertical_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
	unsigned int ii;
	
	Bottle & botW = yarpVerticalLaserOut.prepare(); 
	botW.addInt(2);//indicates basic success
	botW.addDouble(msg->header.stamp.toSec());
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(fabs(msg->angle_max - msg->angle_min));
	botW.addDouble(msg->angle_min);
	botW.addDouble(msg->range_min);
	botW.addDouble(msg->range_max);
	botW.addDouble(0.05);//sigma. I think not used by ry-oa
	botW.addInt(msg->ranges.size());
	for (ii=0;ii<msg->ranges.size();ii++){ botW.addDouble(msg->ranges.at(ii)); } //add laser data
	botW.addDouble(-1);//reserved1
	botW.addDouble(-1);//reserved2
	yarpVerticalLaserOut.write();

  //ROS_INFO("RyOaBridgeAlgNode::vertical_laser_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->vertical_laser_mutex_.enter(); 

  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->vertical_laser_mutex_.exit(); 
}
void RyOaBridgeAlgNode::front_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
	unsigned int ii;
	
	Bottle & botW = yarpFrontLaserOut.prepare(); 
	botW.addInt(2);//indicates basic success
	botW.addDouble(msg->header.stamp.toSec());
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(0);//laser mounting position. Not taken into account by ry-oa
	botW.addDouble(fabs(msg->angle_max - msg->angle_min));
	botW.addDouble(msg->angle_min);
	botW.addDouble(msg->range_min);
	botW.addDouble(msg->range_max);
	botW.addDouble(0.05);//sigma. I think not used by ry-oa
	botW.addInt(msg->ranges.size());
	for (ii=0;ii<msg->ranges.size();ii++){ botW.addDouble(msg->ranges.at(ii)); } //add laser data
	botW.addDouble(-1);//reserved1
	botW.addDouble(-1);//reserved2
	yarpFrontLaserOut.write();

  //ROS_INFO("RyOaBridgeAlgNode::front_laser_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  //this->front_laser_mutex_.enter(); 

  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  //this->front_laser_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void RyOaBridgeAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void RyOaBridgeAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<RyOaBridgeAlgNode>(argc, argv, "ry_oa_bridge_alg_node");
}
