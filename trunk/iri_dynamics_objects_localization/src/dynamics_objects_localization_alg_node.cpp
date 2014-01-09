#include "dynamics_objects_localization_alg_node.h"

DynamicsObjectsLocalizationAlgNode::DynamicsObjectsLocalizationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<DynamicsObjectsLocalizationAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->scan_filtered_publisher_ = this->public_node_handle_.advertise<sensor_msgs::LaserScan>("scan_filtered", 100);
  
  // [init subscribers]
  this->people_subscriber_ = this->public_node_handle_.subscribe("people", 100, &DynamicsObjectsLocalizationAlgNode::people_callback, this);
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 100, &DynamicsObjectsLocalizationAlgNode::scan_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]

}

DynamicsObjectsLocalizationAlgNode::~DynamicsObjectsLocalizationAlgNode(void)
{
  // [free dynamic memory]
}

void DynamicsObjectsLocalizationAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
	// this function delete people segments in scan laser information
	deleteSegments();
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

	

  // [publish messages]
  this->scan_filtered_publisher_.publish(this->LaserScan_msg_);
}

/*  [subscriber callbacks] */
void DynamicsObjectsLocalizationAlgNode::people_callback(const iri_nav_msgs::PoseWithCovarianceStampedArray::ConstPtr& msg) 
{ 
  //ROS_INFO("DynamicsObjectsLocalizationAlgNode::people_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
  this->people_mutex_.enter(); 

	this->people_ = *msg;

  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  this->people_mutex_.exit();

}
void DynamicsObjectsLocalizationAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
  //ROS_INFO("DynamicsObjectsLocalizationAlgNode::scan_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
  this->scan_mutex_.enter();

	this->laser_ = *msg;

  //std::cout << msg->data << std::endl; 

  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  this->scan_mutex_.exit(); 

}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void DynamicsObjectsLocalizationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void DynamicsObjectsLocalizationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<DynamicsObjectsLocalizationAlgNode>(argc, argv, "dynamics_objects_localization_alg_node");
}

/*
	Looks for the center of the people and delete points in a square around them.
*/

void DynamicsObjectsLocalizationAlgNode::deleteSegments(void)
	{
		unsigned int peopleSize = people_.poses.size();
		unsigned int laserSize = laser_.ranges.size();
		
		unsigned int U = 1.0;

		for (unsigned int i = 0; i < laserSize; i++)
		{
			float angle = laser_.angle_min + (laser_.angle_increment * i);

			float x_laser = laser_.ranges[i] * cos(angle);
			float y_laser = laser_.ranges[i] * sin(angle);

			for(unsigned int j = 0; j < peopleSize; j++)
			{
				float x_people = people_.poses[j].pose.position.x;
				float y_people = people_.poses[j].pose.position.y;

				if ((x_laser >= x_people - U) && (x_laser <= x_people + U) && 
					(y_laser >= y_people - U) && (y_laser <= y_people + U))
				{
					laser_.ranges[i] = 0.0;
				}
			}
    }
		this->LaserScan_msg_ = laser_;
	}
