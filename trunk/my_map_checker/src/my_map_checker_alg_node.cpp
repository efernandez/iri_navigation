#include "my_map_checker_alg_node.h"


MyMapCheckerAlgNode::MyMapCheckerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<MyMapCheckerAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 0.1;//in [Hz]
	this->public_node_handle_.getParam("force_map_path", force_map_path_);
	cout << "path = " << force_map_path_ << endl;
	if ( !pred_.read_force_map(  force_map_path_.c_str() ) )
	{
		ROS_WARN("Could not read map force file !!!");
	}
	pred_.get_map_params( min_x_, max_x_, min_y_, max_y_, resolution_, map_number_x_, map_number_y_);

  // [init publishers]
  this->force_map_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("force_map", 100);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
	force_marker_.ns =  "force_map";
	force_marker_.type = visualization_msgs::Marker::ARROW;
	force_marker_.action = visualization_msgs::Marker::ADD;
	force_marker_.lifetime = ros::Duration(10.0f);
	force_marker_.scale.x = 0.1;
	force_marker_.scale.y = 0.15;
	force_marker_.color.a = 1.0;
	force_marker_.color.r = 0.0;
	force_marker_.color.g = 0.0;
	force_marker_.color.b = 1.0;
	force_marker_.header.frame_id = "/map";

	obstacle_marker_.ns =  "force_map";
	obstacle_marker_.type = visualization_msgs::Marker::CUBE;
	obstacle_marker_.action = visualization_msgs::Marker::ADD;
	obstacle_marker_.lifetime = ros::Duration(10.0f);
	obstacle_marker_.scale.x = 0.2;
	obstacle_marker_.scale.y = 0.2;
	obstacle_marker_.scale.z = 0.2;
	obstacle_marker_.color.a = 1.0;
	obstacle_marker_.color.r = 1.0;
	obstacle_marker_.color.g = 0.0;
	obstacle_marker_.color.b = 0.0;
	obstacle_marker_.header.frame_id = "/map";

}

MyMapCheckerAlgNode::~MyMapCheckerAlgNode(void)
{
  // [free dynamic memory]
}

void MyMapCheckerAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->MarkerArray_msg_.data = my_var;

	geometry_msgs::Point ros_point;
	Sforce map_force;
	force_marker_.header.stamp = ros::Time::now();
	MarkerArray_msg_.markers.clear();


	for(unsigned int i = 0; i< map_number_x_; i+=3)
	{
		for(unsigned int j = 0; j< map_number_y_; j+=3)
		{
			
			force_marker_.points.clear();
			force_marker_.id = i * map_number_x_ + j;
			ros_point.x = min_x_ + i * resolution_;
			ros_point.y = min_y_ + j * resolution_;
			force_marker_.points.push_back(  ros_point  );
			map_force = pred_.get_force_map(ros_point.x , ros_point.y);
			ros_point.x += 0.1*map_force.fx;
			ros_point.y += 0.1*map_force.fy;
			force_marker_.points.push_back(  ros_point  );
			MarkerArray_msg_.markers.push_back(  force_marker_  );
			
			/*obstacle_marker_.points.clear();
			obstacle_marker_.id = i * map_number_x_ + j;
			ros_point.x = min_x_ + i * resolution_;
			ros_point.y = min_y_ + j * resolution_;
			//ROS_INFO( "%d" , (int)pred_.is_cell_clear_map(ros_point.x , ros_point.y) );
			if( !pred_.is_cell_clear_map(ros_point.x , ros_point.y) )
			{
				obstacle_marker_.pose.position =  ros_point;
				MarkerArray_msg_.markers.push_back(  obstacle_marker_  );
			}*/
		}
	}
/*	for (unsigned int i = 0; i<map_number_x_*map_number_y_; i+=5)
	{
		if( !pred_.get_map_obstacle( i ) )
		{
			ros_point.y = min_y_ + i/map_number_x_*resolution_;
			ros_point.x = min_x_ + (i - i/map_number_x_ * map_number_x_ ) * resolution_;
			obstacle_marker_.id = i;
			obstacle_marker_.pose.position =  ros_point;
			MarkerArray_msg_.markers.push_back(  obstacle_marker_  );
		}
	}
  */
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->force_map_publisher_.publish(this->MarkerArray_msg_);

}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void MyMapCheckerAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
	force_map_path_ = config.force_map_path;
  this->alg_.unlock();
}

void MyMapCheckerAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<MyMapCheckerAlgNode>(argc, argv, "my_map_checker_alg_node");
}
