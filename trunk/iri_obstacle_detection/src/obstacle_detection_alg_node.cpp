#include "obstacle_detection_alg_node.h"

ObstacleDetectionAlgNode::ObstacleDetectionAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<ObstacleDetectionAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->pub_cloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("pub_cloud", 100);
  
  // [init subscribers]
  this->cloud_raw_subscriber_ = this->public_node_handle_.subscribe("cloud_raw", 100, &ObstacleDetectionAlgNode::cloud_raw_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

ObstacleDetectionAlgNode::~ObstacleDetectionAlgNode(void)
{
  // [free dynamic memory]
}

void ObstacleDetectionAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->PointCloud2_msg.data = my_var;
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
 // this->pub_cloud_publisher_.publish(this->PointCloud2_msg_);
}

/*  [subscriber callbacks] */
void ObstacleDetectionAlgNode::cloud_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{ 
  ROS_INFO("ObstacleDetectionAlgNode::cloud_raw_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->alg_.lock(); 
  this->cloud_raw_mutex_.enter(); 
  this->alg_.pcl_camera= *msg;
  //std::cout << msg->data << std::endl; 
   this->pub_cloud_publisher_.publish(this->alg_.getPCLfiltered());
  //unlock previously blocked shared variables 
  //this->alg_.unlock(); 
  this->cloud_raw_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void ObstacleDetectionAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void ObstacleDetectionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<ObstacleDetectionAlgNode>(argc, argv, "obstacle_detection_alg_node");
}
