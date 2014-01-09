#include "obstacle_detection_alg.h"

ObstacleDetectionAlgorithm::ObstacleDetectionAlgorithm(void)
{
}

ObstacleDetectionAlgorithm::~ObstacleDetectionAlgorithm(void)
{
}

void ObstacleDetectionAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// ObstacleDetectionAlgorithm Public API
void ObstacleDetectionAlgorithm::CloudConsensusSegmentation(const sensor_msgs::PointCloud2& ros_cloud_in,sensor_msgs::PointCloud2& ros_cloud_out)
{
	 pcl::PointCloud<pcl::PointXYZ> cloud;
     pcl::fromROSMsg (ros_cloud_in, cloud);
     pcl::SACSegmentation<pcl::PointXYZRGB> seg;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setDistanceThreshold (0.01);
     seg.setInputCloud(cloud.makeShared());
     pcl::toROSMsg (*seg.getInputCloud(), ros_cloud_out);
}
sensor_msgs::PointCloud2 ObstacleDetectionAlgorithm::getPCLfiltered()
{
	sensor_msgs::PointCloud2 out;
	CloudConsensusSegmentation(pcl_camera,out);
	return out;
}

