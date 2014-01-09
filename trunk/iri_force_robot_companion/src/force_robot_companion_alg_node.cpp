#include "force_robot_companion_alg_node.h"
#include "tf/transform_datatypes.h"
#include <deque>
#include <wiimote/State.h>
#include <stdio.h>
//#include <string.h>


ForceRobotCompanionAlgNode::ForceRobotCompanionAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<ForceRobotCompanionAlgorithm>(),
	move_base_client_("move_base", true),
  node_hz_(5.0),
	tf_listener_(ros::Duration(30.f)),
	tf_laser_listener_(ros::Duration(30.f)),
	robot_stop_(false),
    ready_to_read_laser_(false),
    nearest_target_(0)
{
  //init class attributes if necessary
  this->loop_rate_ = node_hz_;//in [Hz]

	this->public_node_handle_.getParam("force_map_path", force_map_path_);
    this->public_node_handle_.getParam("reference_frame", target_frame_);
    this->public_node_handle_.getParam("robot_frame", robot_frame_);
    this->public_node_handle_.getParam("robot_sim", robot_sim_);
	this->public_node_handle_.getParam("robot_x_ini", robot_x_ini_);
	this->public_node_handle_.getParam("robot_y_ini", robot_y_ini_);

  // [init publishers]
  this->angle_robot_person_publisher_ = this->public_node_handle_.advertise<std_msgs::Float64>("angle_robot_person", 100);
  this->person_robot_dist_publisher_ = this->public_node_handle_.advertise<std_msgs::Float64>("person_robot_dist", 100);
  //this->target_pred_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseArray>("target_pred", 100);
  this->target_person_position_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PointStamped>("target_position", 100);
  this->robot_position_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PointStamped>("robot_position", 100);
  this->force_param_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PointStamped>("force_param", 100);
  this->velocity_publisher_ = this->public_node_handle_.advertise<geometry_msgs::Twist>("velocity", 100);
  this->trajectories_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/trajectories", 100);
  this->forces_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/forces", 100);
  this->destinations_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/destinations", 100);
  this->predictions_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/predictions", 100);
  // [init subscribers]
  this->laser_subscriber_ = this->public_node_handle_.subscribe("laser", 100, &ForceRobotCompanionAlgNode::laser_callback, this);
  this->dest_subscriber_ = this->public_node_handle_.subscribe("dest", 100, &ForceRobotCompanionAlgNode::dest_callback, this);
  this->joy_subscriber_ = this->public_node_handle_.subscribe("joy", 100, &ForceRobotCompanionAlgNode::joy_callback, this);
  this->tracks_subscriber_ = this->public_node_handle_.subscribe("tracks", 100, &ForceRobotCompanionAlgNode::tracks_callback, this);
  
  // [init services]
  //parameter import example
  //private_node_handle.param<std:tring>("scanTopicString", scanTopicString, "scan");
  
  init_node();
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

ForceRobotCompanionAlgNode::~ForceRobotCompanionAlgNode(void)
{
  // [free dynamic memory]

}

void ForceRobotCompanionAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->Float32MultiArray_msg.data = my_var;
  //this->person_robot_dist_msg_.data = my_var;
  //this->PoseArray_msg.data = my_var;
  //this->PointStamped_msg.data = my_var;
  //this->peopleTrackingArray_msg.data = my_var;
  //this->Twist_msg.data = my_var;
  //this->MarkerArray_msg.data = my_var;
  //this->Path_msg.data = my_var;
	//configure parameters

	
	//prediction
    this->alg_.lock();
	pred_.scene_intentionality_prediction_bhmip();

	//Visualization
	//ROS_INFO( "vis trajectories\n" );
  	vis_trajectories();
	vis_destinations();
	//vis_intentionality_prediction();
// 	vis_predictions();
	
	
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]
  
	//robot control
	robot_desired_position_ = pred_.robot_approach_pose( f1_, f2_, f3_, f4_,f5_, f_  );
	//ROS_WARN( "Current robot state =  %d" , pred_.get_robot_state() );
	//ROS_INFO( "Desired robot velocity = %f  Current robot velocity = %f" , robot_desired_position_.v, pred_.get_robot().get_current_pose().v);
	//ROS_INFO( "Current robot goal = ( %f , %f )" , robot_desired_position_.x , robot_desired_position_.y );
       // ROS_INFO("Forces : goal = %f, person = %f, int = %f" , f1_.fx,f2_.fy,f3_.fx );
        //ROS_INFO("Interaction forces : laser = %f, map = %f" , f4_.module(),f5_.module() );

	if( robot_sim_ )
	{
		if (robot_desired_position_.x !=  robot_desired_position_.x)//when invlaid commands are given, nan positions are sent...
		{
			robot_desired_position_.x = 0.0;
			robot_desired_position_.y = 0.0;
		}
		pred_.update_robot( SdetectionObservation( 0, ros::Time::now().toSec(), 
									   robot_desired_position_.x, robot_desired_position_.y ) );
    }


	if( !move_base_mode_ )
	{
		//ROS_INFO( "Debug mode On");
		Twist_msg_.linear.x = 0.0;
	}
	else
	{
		//fill performance messages
		angle_robot_person_msg_.data = target_followed_pose_.angle_heading_point( pred_.get_robot()->get_current_pointV() );
		if( pred_.get_is_target_person_visible() && !robot_stop_)
		{
			// node sends commands to the no_collision node (move robot)
			ROS_INFO( "Sending robot commands");
	 		move_baseMakeActionRequest();
		    Twist_msg_.linear.x = robot_desired_position_.v();
		}
		if ( robot_stop_ )
		{
			// node sends stop command (invalid command)
			ROS_INFO( "Stop mode, sending robot commands");
			robot_desired_position_.x = 1.0/0.0;
			robot_desired_position_.y = 1.0/0.0;
	 		move_baseMakeActionRequest();
		    Twist_msg_.linear.x = 0.0;
			robot_stop_ = false;//only sends the stop command once
		}

	}
	//if(robot_stop_) ROS_INFO("value robot True");
	//else ROS_INFO("value robot False");



  ready_to_read_laser_ = true;
  this->alg_.unlock();


  // [publish messages]
  this->angle_robot_person_publisher_.publish(this->angle_robot_person_msg_);
  this->person_robot_dist_publisher_.publish(this->person_robot_dist_msg_);
  force_param_msg_.header.stamp = ros::Time::now();
    this->target_person_position_publisher_.publish(this->target_person_position_msg_);
    this->robot_position_publisher_.publish(this->robot_position_msg_);
  this->force_param_publisher_.publish(this->force_param_msg_);
  this->velocity_publisher_.publish(this->Twist_msg_);
	this->trajectories_publisher_.publish(this->MarkerArray_trajectories_msg_);
	this->forces_publisher_.publish(this->MarkerArray_forces_msg_);
	this->destinations_publisher_.publish(this->MarkerArray_destinations_msg_);
	this->predictions_publisher_.publish(this->MarkerArray_predictions_msg_);
}


//common fields, to be filled just once...
void ForceRobotCompanionAlgNode::init_node()
{

  pred_.set_dt( 1.0 / node_hz_ );
  this->public_node_handle_.getParam("reference_frame", target_frame_);
	this->public_node_handle_.getParam("force_map_path", force_map_path_);
  this->public_node_handle_.getParam("destination_map_path", destination_map_path_);
	if ( !pred_.read_destination_map(  destination_map_path_.c_str() ) )
	{
		ROS_ERROR("Could not read map destinations file !!!");
	}
    else{
		ROS_WARN("read destinations map file : SUCCESS!!!");
	}

    //read destinations
	if ( !pred_.read_force_map(  force_map_path_.c_str() ) )
	{
		ROS_ERROR("Could not read map force file !!!");
	}
    else{
		ROS_WARN("read map force file : SUCCESS!!!");
	}
	
	//Robot platform initial config
	force_param_msg_.header.frame_id = target_frame_;
	Twist_msg_.angular.z = 100.0;
	ros::Time now = ros::Time::now();

    //if simulation, update robot position
    if ( robot_sim_ )
    {
    	Spose robot_current_position( robot_x_ini_,robot_y_ini_, 0.0, now.toSec() );
	    pred_.update_robot( SdetectionObservation( 0, robot_current_position.time_stamp, 
											   robot_current_position.x, robot_current_position.y ) );
    }
	
	//action messages
	move_base_goal_.target_pose.header.frame_id = target_frame_;
	move_base_goal_.target_pose.pose.orientation.y = 1.0;
											   
	//trajectory marker										   
	traj_marker_.ns = "trajectories";
	traj_marker_.type = visualization_msgs::Marker::LINE_STRIP;
	traj_marker_.action = visualization_msgs::Marker::ADD;
	traj_marker_.lifetime = ros::Duration(1.0f);
	traj_marker_.scale.x = 0.1;
	traj_marker_.color.a = 0.5;
	traj_marker_.color.r = 1.0;
	traj_marker_.color.g = 0.4;
	traj_marker_.color.b = 0.0;

	//robot trajectory marker										   
	traj_robot_marker_.ns = "trajectories";
	traj_robot_marker_.type = visualization_msgs::Marker::LINE_STRIP;
	traj_robot_marker_.action = visualization_msgs::Marker::ADD;
	traj_robot_marker_.lifetime = ros::Duration(1.0f);
	traj_robot_marker_.scale.x = 0.1;
	traj_robot_marker_.color.a = 0.6;
	traj_robot_marker_.color.r = 0.26;
	traj_robot_marker_.color.g = 0.0;
	traj_robot_marker_.color.b = 0.5;

	//robot marker
	robot_marker_.ns = "trajectories";
	robot_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
	robot_marker_.mesh_resource = "package://tibi_dabo_base/model/meshes/tibi.stl";
	robot_marker_.action = visualization_msgs::Marker::ADD;
	robot_marker_.lifetime = ros::Duration(1.0f);
	robot_marker_.scale.x = 1.0;
	robot_marker_.scale.y = 0.6;
	robot_marker_.scale.z = 1.0;
	robot_marker_.color.a = 1.0;
	robot_marker_.color.r = 0.7;
	robot_marker_.color.g = 0.5;
	robot_marker_.color.b = 1.0;

	
	//target marker
	target_marker_.ns = "trajectories";
	target_marker_.type = visualization_msgs::Marker::CYLINDER;
	target_marker_.action = visualization_msgs::Marker::ADD;
	target_marker_.lifetime = ros::Duration(1.0f);
	target_marker_.scale.x = 0.5;
	target_marker_.scale.y = 0.5;


	//text marker
	text_marker_.ns = "trajectories";
	text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text_marker_.action = visualization_msgs::Marker::ADD;
	text_marker_.lifetime = ros::Duration(1.0f);
	text_marker_.scale.z = 0.5;
	text_marker_.pose.position.z = 1.5;
	text_marker_.color.a = 0.5;
	text_marker_.color.r = 0.0;
	text_marker_.color.g = 0.0;
	text_marker_.color.b = 0.0;
	
	//force markers: resultant force (red)
	force_marker_.ns =  "trajectories";
	force_marker_.type = visualization_msgs::Marker::ARROW;
	force_marker_.action = visualization_msgs::Marker::ADD;
	force_marker_.lifetime = ros::Duration(1.0f);
	force_marker_.scale.x = 0.2;
	force_marker_.scale.y = 0.25;
	force_marker_.color.a = 0.8;
	force_marker_.color.r = 1.0;
	force_marker_.color.g = 0.0;
	force_marker_.color.b = 0.0;
	
	//force makers: force to goal: ( blue )
	force_goal_marker_ = force_marker_;
	force_goal_marker_.color.r = 0.0;
	force_goal_marker_.color.g = 0.4;
	force_goal_marker_.color.b = 1.0;

	//force markers: goal force to person - goal (orange)
	force_goal2_marker_ = force_marker_;
	force_goal2_marker_.color.r = 1.0;
	force_goal2_marker_.color.g = 0.4;
	force_goal2_marker_.color.b = 0.0;

	//force markers: scaled person interaction force (green)
	force_int_person_marker_ = force_marker_;
	force_int_person_marker_.color.r = 0.2;
	force_int_person_marker_.color.g = 0.85;
	force_int_person_marker_.color.b = 0.2;

	//force markers: robot interation force only valid to persons (pink)
	force_int_robot_marker_ = force_marker_;
    force_int_robot_marker_.color.r = 0.26;
    force_int_robot_marker_.color.g = 0.0;
    force_int_robot_marker_.color.b = 0.66;

	//force markers: scaled laser obstacles interaction force (cyan)
	force_obstacle_laser_marker_ = force_marker_;
	force_obstacle_laser_marker_.color.r = 0.0;
	force_obstacle_laser_marker_.color.g = 1.0;
	force_obstacle_laser_marker_.color.b = 1.0;

	//force markers: map obstacles interaction force (black)
	force_obstacle_map_marker_ = force_marker_;
    force_obstacle_map_marker_.color.r = 0.0;
    force_obstacle_map_marker_.color.g = 0.0;
    force_obstacle_map_marker_.color.b = 0.0;

	//destination marker
	dest_marker_.ns = "destinations";
	dest_marker_.type = visualization_msgs::Marker::CYLINDER;
	dest_marker_.action = visualization_msgs::Marker::ADD;
	dest_marker_.lifetime = ros::Duration(1.0f);
	dest_marker_.color.a = 0.8;
	
	//predictions marker
	pred_marker_.ns = "predictions";
	pred_marker_.type = visualization_msgs::Marker::LINE_STRIP;
	pred_marker_.action = visualization_msgs::Marker::ADD;
	pred_marker_.scale.x = 0.1;
	pred_marker_.lifetime = ros::Duration(1.0f);
	pred_marker_.color.a = 0.5;
	pred_marker_.color.r = 0.0;
	pred_marker_.color.g = 0.0;
	pred_marker_.color.b = 0.0;


}

void ForceRobotCompanionAlgNode::vis_trajectories()
{

	int cont = 0;
	geometry_msgs::Point ros_point, ros_point_ini;

	//fill headers
	traj_marker_.header.stamp = ros::Time::now();
	traj_marker_.header.frame_id = target_frame_;
	traj_robot_marker_.header.stamp = traj_marker_.header.stamp;
	traj_robot_marker_.header.frame_id = target_frame_;
	robot_marker_.header.stamp = traj_marker_.header.stamp;
	robot_marker_.header.frame_id = target_frame_;
	target_marker_.header.stamp = traj_marker_.header.stamp;
	target_marker_.header.frame_id = target_frame_;
	text_marker_.header.stamp = traj_marker_.header.stamp;
	text_marker_.header.frame_id = target_frame_;
	force_marker_.header.stamp = traj_marker_.header.stamp;
	force_marker_.header.frame_id = target_frame_;
	force_goal_marker_.header.stamp = force_marker_.header.stamp;
	force_goal_marker_.header.frame_id = force_marker_.header.frame_id;
	force_goal2_marker_.header.stamp = force_marker_.header.stamp;
	force_goal2_marker_.header.frame_id = force_marker_.header.frame_id;
	force_int_person_marker_.header.stamp = force_marker_.header.stamp;
	force_int_person_marker_.header.frame_id = force_marker_.header.frame_id;
	force_int_robot_marker_.header.stamp = force_marker_.header.stamp;
	force_int_robot_marker_.header.frame_id = force_marker_.header.frame_id;
	force_obstacle_map_marker_.header.stamp = force_marker_.header.stamp;
	force_obstacle_map_marker_.header.frame_id = force_marker_.header.frame_id;
	force_obstacle_laser_marker_.header.stamp = force_marker_.header.stamp;
	force_obstacle_laser_marker_.header.frame_id = force_marker_.header.frame_id;

	MarkerArray_trajectories_msg_.markers.clear();
	MarkerArray_forces_msg_.markers.clear();
	

	//drawing robot forces ---------------------------------------------------------------
	clear_force_markers();
	//Sforce  force_to_goal, force_int_person , force_int_robot, force_obstacle;//TODO: change code to new function get_forces
	//pred_.get_robot().get_forces_person( force_to_goal, force_int_person , force_int_robot, force_obstacle );

	//initial point
	ros_point_ini.x = pred_.get_robot()->get_current_pointV().x;
	ros_point_ini.y = pred_.get_robot()->get_current_pointV().y;

	//drawing robot trajectory. As we are just plotting the target path, we will read the current pose
	if ( traj_robot_marker_.points.size() > 500)
	{
		vector<geometry_msgs::Point> temp_poses;
		temp_poses.reserve(250);
		for( unsigned int i = 251; i <= 500; ++i)
			temp_poses.push_back(traj_robot_marker_.points[i]);
		traj_robot_marker_.points = temp_poses;
	}
	traj_robot_marker_.points.push_back(    ros_point_ini   );
	traj_robot_marker_.id = cont;
	++cont;
	MarkerArray_trajectories_msg_.markers.push_back(  traj_robot_marker_  );
	

	//scaled force to goal: ( blue )
	force_goal_marker_.points.push_back(    ros_point_ini   );
	ros_point.x = ros_point_ini.x + f1_.fx;
	ros_point.y = ros_point_ini.y + f1_.fy;
	force_goal_marker_.points.push_back(    ros_point   );
	force_goal_marker_.id = cont;
	++cont;
	MarkerArray_trajectories_msg_.markers.push_back(  force_goal_marker_  );

	//scaled force to person - goal (orange)
	force_goal2_marker_.points.push_back(    ros_point_ini   );
	ros_point.x = ros_point_ini.x + f2_.fx;
	ros_point.y = ros_point_ini.y + f2_.fy;
	force_goal2_marker_.points.push_back( ros_point );
	force_goal2_marker_.id = cont;
	++cont;
	MarkerArray_trajectories_msg_.markers.push_back(  force_goal2_marker_  );

	//scaled person interaction force (green)
	force_int_person_marker_.points.push_back(    ros_point_ini   );
	ros_point.x = ros_point_ini.x + f3_.fx;
	ros_point.y = ros_point_ini.y + f3_.fy;
	force_int_person_marker_.points.push_back( ros_point);
	force_int_person_marker_.id = cont;
	++cont;
	MarkerArray_trajectories_msg_.markers.push_back(  force_int_person_marker_  );

	//scaled laser obstacles interaction force (yellow)
	force_obstacle_laser_marker_.points.push_back(    ros_point_ini   );
	ros_point.x = ros_point_ini.x + f4_.fx;
	ros_point.y = ros_point_ini.y + f4_.fy;
	force_obstacle_laser_marker_.points.push_back( ros_point);
	force_obstacle_laser_marker_.id = cont;
	++cont;
	MarkerArray_trajectories_msg_.markers.push_back(  force_obstacle_laser_marker_  );

	//map obstacles interaction force (black)
	force_obstacle_map_marker_.points.push_back(    ros_point_ini   );
    ros_point.x = ros_point_ini.x + f5_.fx;
    ros_point.y = ros_point_ini.y + f5_.fy;
    force_obstacle_map_marker_.points.push_back( ros_point);
    force_obstacle_map_marker_.id = cont;
    ++cont;
    MarkerArray_trajectories_msg_.markers.push_back(  force_obstacle_map_marker_  );

	//weighted resultant force (red)
	force_marker_.points.push_back(    ros_point_ini   );
	ros_point.x = ros_point_ini.x + f_.fx;
	ros_point.y = ros_point_ini.y + f_.fy;
	force_marker_.points.push_back(  ros_point);
	force_marker_.id = cont;
	++cont;
	MarkerArray_trajectories_msg_.markers.push_back(  force_marker_  );

	//Draw robot as a (pink) .stl mesh
	double robot_orientation = pred_.get_robot()->get_current_pointV().orientation() - PI/2.0;
	robot_marker_.pose.position.x = ros_point_ini.x + 0.3*cos(robot_orientation);
	robot_marker_.pose.position.y = ros_point_ini.y + 0.3*sin(robot_orientation);
	robot_marker_.pose.position.z = 0.4;
	//geometry_msgs::Quaternion  tf::createQuaternionMsgFromRollPitchYaw(double roll,double pitch,double yaw)
	robot_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI/2.0,0.0, robot_orientation );
	robot_marker_.id = cont;
	++cont;
	MarkerArray_trajectories_msg_.markers.push_back(  robot_marker_  );
	
	
	//draw pedestrians' trajectories ------------------------------------------------
	vector<SpointV_cov> scene_targets_pointV;//TODO es esto lo mejor o sacar una estuctura ...time to go home
  vector<vector<Sforce> > scene_targets_force;
	vector<unsigned int> scene_targets_ids;
  pred_.get_scene_observation(scene_targets_pointV,scene_targets_ids,scene_targets_force);
	Sforce  force_to_goal, force_int_person , force_int_robot, force_obstacle, force_total;
	for( unsigned int i = 0; i < (unsigned int)scene_targets_pointV.size() ; i++)
	{
		SpointV_cov target_pose = scene_targets_pointV[i];

		//drawing person forces ---------------------------------------------------------------
		clear_force_markers();
    force_to_goal = scene_targets_force[i][0];
    force_int_person  = scene_targets_force[i][1];
    force_int_robot  = scene_targets_force[i][2];
    force_obstacle = scene_targets_force[i][3];
    force_total = force_to_goal+force_int_person+force_int_robot+force_obstacle;

		//initial point
		ros_point_ini.x = target_pose.x;
		ros_point_ini.y = target_pose.y;

		//scaled force to goal: ( blue )
		force_goal_marker_.points.push_back(    ros_point_ini   );
		ros_point.x = ros_point_ini.x + force_to_goal.fx;
		ros_point.y = ros_point_ini.y + force_to_goal.fy;
		force_goal_marker_.points.push_back(    ros_point   );
		force_goal_marker_.id = cont;
		++cont;
		MarkerArray_forces_msg_.markers.push_back(  force_goal_marker_  );

		//scaled person interaction force (green)
		force_int_person_marker_.points.push_back(    ros_point_ini   );
		ros_point.x = ros_point_ini.x + force_int_person.fx;
		ros_point.y = ros_point_ini.y + force_int_person.fy;
		force_int_person_marker_.points.push_back( ros_point);
		force_int_person_marker_.id = cont;
		++cont;
		MarkerArray_forces_msg_.markers.push_back(  force_int_person_marker_  );

		//scaled robot force (pink)
		force_int_robot_marker_.points.push_back(    ros_point_ini   );
		ros_point.x = ros_point_ini.x + force_int_robot.fx;
		ros_point.y = ros_point_ini.y + force_int_robot.fy;
		force_int_robot_marker_.points.push_back( ros_point);
		force_int_robot_marker_.id = cont;
		++cont;
		MarkerArray_forces_msg_.markers.push_back(  force_int_robot_marker_  );

		//map obstacles interaction force (black)
		force_obstacle_map_marker_.points.push_back(    ros_point_ini   );
		ros_point.x = ros_point_ini.x + force_obstacle.fx;
		ros_point.y = ros_point_ini.y + force_obstacle.fy;
		force_obstacle_map_marker_.points.push_back( ros_point);
		force_obstacle_map_marker_.id = cont;
		++cont;
		MarkerArray_forces_msg_.markers.push_back(  force_obstacle_map_marker_  );

		//laser obstacles interaction force (cyan)

		//weighted resultant force (red)
		force_marker_.points.push_back(    ros_point_ini   );
		ros_point.x = ros_point_ini.x + force_total.fx;
		ros_point.y = ros_point_ini.y + force_total.fy;
		force_marker_.points.push_back(  ros_point);
		force_marker_.id = cont;
		++cont;
		MarkerArray_forces_msg_.markers.push_back(  force_marker_  );

		if (scene_targets_ids[i] == (unsigned int)pred_.get_target_person())
		{	
			//ROS_INFO( "Current person velocity = %f" , target_pose.v);
			target_followed_pose_ = target_pose;
			//drawing person trajectory. As we are just plotting the target path, we will read the current pose
			ros_point.x = target_pose.x;
			ros_point.y = target_pose.y;
			if ( traj_marker_.points.size() > 500)
			{
				vector<geometry_msgs::Point> temp_poses;
				temp_poses.reserve(250);
				for( unsigned int i = 251; i <= 500; ++i)
					temp_poses.push_back(traj_marker_.points[i]);
				traj_marker_.points = temp_poses;
			}
			traj_marker_.points.push_back(    ros_point   );
			traj_marker_.id = cont;
			++cont;
			MarkerArray_trajectories_msg_.markers.push_back(  traj_marker_  );
			
			//target person drawn differently
			target_marker_.scale.z = 0.8;
			target_marker_.color.a = 0.8;
			target_marker_.color.r = 1.0;
			target_marker_.color.g = 0.4;
			target_marker_.color.b = 0.0;
			//draw targets as cylinders
			target_marker_.pose.position.x = target_pose.x;
			target_marker_.pose.position.y = target_pose.y;
			target_marker_.pose.position.z = 0.4;
			++cont;
			target_marker_.id = cont;
			MarkerArray_trajectories_msg_.markers.push_back(  target_marker_  );
			
			//fill  current target pose message
			target_person_position_msg_.point.x = target_pose.x;
			target_person_position_msg_.point.y = target_pose.y;
			target_person_position_msg_.point.z = target_pose.v();//overloading the pose message...not good-> TODO
			target_person_position_msg_.header = traj_marker_.header;

			//fill  current robot pose message
			robot_position_msg_.point.x = pred_.get_robot()->get_current_pointV().x;
			robot_position_msg_.point.y = pred_.get_robot()->get_current_pointV().y;
			robot_position_msg_.point.z = pred_.get_robot()->get_current_pointV().v();
			robot_position_msg_.header = traj_marker_.header;

			//calcultate the absolute robot-person distance and create->fill topic
			person_robot_dist_msg_.data = target_pose.distance( pred_.get_robot()->get_current_pointV() );
		}
		else
		{
			target_marker_.scale.z = 0.6;
			target_marker_.color.a = 0.4;
			target_marker_.color.r = 0.0;
			target_marker_.color.g = 0.8;
			target_marker_.color.b = 0.0;
			//draw targets as cylinders
			target_marker_.pose.position.x = target_pose.x;
			target_marker_.pose.position.y = target_pose.y;
			target_marker_.pose.position.z = 0.3;
			++cont;
			target_marker_.id = cont;
			MarkerArray_trajectories_msg_.markers.push_back(  target_marker_  );
		}
		
		//draw target id
		text_marker_.pose.position.x = target_pose.x;
		text_marker_.pose.position.y = target_pose.y;
		++cont;
		text_marker_.id = cont;
		std::ostringstream target_id;
		target_id << scene_targets_ids[i];
		text_marker_.text = target_id.str();
		MarkerArray_trajectories_msg_.markers.push_back(  text_marker_  );
		++cont;
	}
	
}

void ForceRobotCompanionAlgNode::clear_force_markers()
{
	force_marker_.points.clear();
	force_goal_marker_.points.clear();
	force_goal2_marker_.points.clear();
	force_int_person_marker_.points.clear();
	force_int_robot_marker_.points.clear();
	force_obstacle_map_marker_.points.clear();
	force_obstacle_laser_marker_.points.clear();
}

void ForceRobotCompanionAlgNode::vis_intentionality_prediction()
{
	//return destinations [#person] [#destination] of all current observed persons
	 vector<vector<Sdestination> >&  intentionality_pred = pred_.get_scene_intentionality_prediction_bhmip(  );
	//ROS_INFO("size of predictions = %d" , intentionality_pred.size() );
	dest_marker_.header.frame_id = target_frame_;
	MarkerArray_destinations_msg_.markers.clear();
	dest_marker_.color.r = 1.0;
	dest_marker_.color.g = 0.0;
	dest_marker_.color.b = 0.0;
	dest_marker_.scale.x = 0.5;
	dest_marker_.scale.y = 0.5;
	for (unsigned int i = 0;  i < intentionality_pred.size() ; ++i)
	{
		for (unsigned int j = 0;  j < intentionality_pred[i].size() ; ++j)
		{
 			dest_marker_.scale.z = intentionality_pred[i][j].prob;
			dest_marker_.pose.position.z = 0.5*intentionality_pred[i][j].prob;
// 			ROS_INFO( " prob value at i = %d , j = %d  --- prob = %f " , i , j , intentionality_pred[i][j].prob );
// 			intentionality_pred[i][j].print();
			dest_marker_.pose.position.x = intentionality_pred[i][j].x;
 			dest_marker_.pose.position.y = intentionality_pred[i][j].y;
 			dest_marker_.id = i*intentionality_pred[i].size() + j;
 			MarkerArray_destinations_msg_.markers.push_back( dest_marker_ );
		}
	}
}

void ForceRobotCompanionAlgNode::vis_destinations()
{
	// vector<Sdestination>& dest = pred_.get_destinations();
	dest_marker_.header.frame_id = target_frame_;
	MarkerArray_destinations_msg_.markers.clear();
	int cont = 0;
	//ROS_INFO( "number of destinations %d" , dest.size()  );
	
	//Print scene destinations
	dest_marker_.color.r = 0.0;
	dest_marker_.color.g = 0.4;
	dest_marker_.color.b = 1.0;
	dest_marker_.scale.x = 1;
	dest_marker_.scale.y = 1;
	dest_marker_.scale.z = 0.2;
    dest_marker_.pose.position.z = 0.1;
	for ( unsigned int i = 0; i < pred_.get_destinations()->size(); ++i)
	{
		dest_marker_.pose.position.x = pred_.get_destinations()->at(i).x;
		dest_marker_.pose.position.y = pred_.get_destinations()->at(i).y;
		dest_marker_.id = cont;
		++cont;
		MarkerArray_destinations_msg_.markers.push_back( dest_marker_ );
	}
	
	//print robot destinations
	dest_marker_.color.r = 1.0;
	dest_marker_.color.g = 1.0;
	dest_marker_.color.b = 1.0;
	dest_marker_.scale.x = 0.8;
	dest_marker_.scale.y = 0.8;
	dest_marker_.scale.z = 0.1;
	for ( unsigned int i = 0; i < pred_.get_robot()->get_destinations()->size(); ++i)
	{
		dest_marker_.pose.position.x = pred_.get_robot()->get_destinations()->at(i).x;
		dest_marker_.pose.position.y = pred_.get_robot()->get_destinations()->at(i).y;
		dest_marker_.id = cont;
		++cont;
		MarkerArray_destinations_msg_.markers.push_back( dest_marker_ );
	}
	
	//print target destinations + probabilities
	Cperson_abstract* iit;
	if( pred_.find_person( pred_.get_target_person() , &iit ) )
	{
		dest_marker_.color.r = 0.0;
		dest_marker_.color.g = 0.9;
		dest_marker_.color.b = 0.9;
		dest_marker_.scale.x = 0.5;
		dest_marker_.scale.y = 0.5;
		dest_marker_.scale.z = 0;
		//ROS_INFO("Printing target destinations");
		for ( unsigned int i = 0; i < iit->get_destinations()->size(); ++i)
		{
			dest_marker_.scale.z = iit->get_destinations()->at(i).prob;
			dest_marker_.pose.position.z = 0.5*dest_marker_.scale.z;
			dest_marker_.pose.position.x = iit->get_destinations()->at(i).x;
			dest_marker_.pose.position.y = iit->get_destinations()->at(i).y;
			dest_marker_.id = cont;
			++cont;
			MarkerArray_destinations_msg_.markers.push_back( dest_marker_ );
		}                    

	}

	//print robot current desired position
	dest_marker_.color.r = 0.8;
	dest_marker_.color.g = 0.26;
	dest_marker_.color.b = 0.0;
	dest_marker_.scale.x = 0.2;
	dest_marker_.scale.y = 0.2;
	dest_marker_.scale.z = 0.1;
	dest_marker_.pose.position.x = robot_desired_position_.x;
	dest_marker_.pose.position.y = robot_desired_position_.y;
	dest_marker_.id = cont;
	++cont;
	MarkerArray_destinations_msg_.markers.push_back( dest_marker_ );


}

void ForceRobotCompanionAlgNode::vis_predictions()
{
	//a vector of pointers to the predicted trajectories to all destinations...
	 vector< vector<vector<Spose> >* > scene_pred = pred_.get_scene_motion_prediction();
	int cont = 0;

	//fill headers
	pred_marker_.header.frame_id = target_frame_;
	pred_marker_.header.stamp = ros::Time::now();
	MarkerArray_predictions_msg_.markers.clear();
	//ROS_INFO( "size of prediction trajectories = %d" , scene_pred.size()  );
	//fill body
	for( unsigned int i = 0; i < scene_pred.size(); ++i )
	{
		for(  unsigned int j = 0; j < scene_pred[i]->size() ; ++j  )
		{
			//ROS_INFO( "IN** pred traj size = %d" , scene_pred[i]->size()  );
			pred_marker_.points.clear();
			for(  unsigned int t = 0; t < scene_pred[i]->at(j).size() ; ++t  )
			{
				geometry_msgs::Point ros_point;
				ros_point.x = scene_pred.at(i)->at(j).at(t).x;
				ros_point.y = scene_pred.at(i)->at(j).at(t).y;
				pred_marker_.points.push_back(  ros_point );
			}
			pred_marker_.id = cont;
			++cont;
			MarkerArray_predictions_msg_.markers.push_back(  pred_marker_  );
		}
	}
}


/*  [subscriber callbacks] */
void ForceRobotCompanionAlgNode::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 

  //ROS_INFO("ForceRobotCompanionAlgNode::laser_callback: New Message Received"); 
  //use appropiate mutex to shared variables if necessary 
  this->laser_mutex_.enter();
	//variable to control the reading rate of laser callback (normally at a higher rate) at least at the same rate of the node, otherwise is unnecessary.
if ( !ready_to_read_laser_ || strcmp( msg->header.frame_id.c_str() , "/dabo/front_laser" ) != 0 )
//hardcoded temporal solution to read the front laser TODO:read front+rear
{
	this->laser_mutex_.exit();
	return;
}

  try
  {
	//get transformation
	//ROS_INFO("observation frame = %s" , msg->header.frame_id.c_str() );
	//ROS_INFO("target frame  = %s" , target_frame_.c_str() );
	/*
	bool 	waitForTransform (const std::string &target_frame,
	const std::string &source_frame, const ros::Time &time, const ros::Duration &timeout, 
	const ros::Duration &polling_sleep_duration=ros::Duration(0.01), 
	std::string *error_msg=NULL) const 
	*/
	bool tf_exists = tf_laser_listener_.waitForTransform(target_frame_,
							msg->header.frame_id, 
							msg->header.stamp,
							ros::Duration(1), ros::Duration(0.01));
					
	if(tf_exists)
	{
		/*//pointcloud function
		//void 	transformLaserScanToPointCloud (const std::string &target_frame, const sensor_msgs::LaserScan &scan_in, sensor_msgs::PointCloud &cloud_out, tf::Transformer &tf, int channel_options=channel_option::Default)
		sensor_msgs::PointCloud cloud;
		laser_projector_.transformLaserScanToPointCloud( target_frame_ , *msg, cloud, tf_laser_listener_);
		*/
		//iterative transformation
		geometry_msgs::PointStamped observation_point, target_point;
		observation_point.header = msg->header;
		observation_point.point.z = 0.0;

		//read the laser data
	  this->alg_.lock(); 
		pred_.read_laser_scan_clear();
		double angle = msg->angle_min;
		for( unsigned int i = 0; i < msg->ranges.size() ; ++i )
		{
			//pred_.read_laser_scan_point ( Spoint( cloud.points[i].x , cloud.points[i].y ) );
			if ( msg->ranges[i] < laser_radi_ ) //filtering further scan lasers than 5[m]
			{
				observation_point.point.x = msg->ranges[i] * cos(angle);
				observation_point.point.y = msg->ranges[i] * sin(angle);
				//tf_listener_.transformPoint( target_frame_,  msg->header.stamp, target_point, msg->header.frame_id, observation_point);
				tf_laser_listener_.transformPoint( target_frame_, observation_point, target_point);
				pred_.read_laser_scan_point ( Spoint( target_point.point.x , target_point.point.y ) );
			}
			angle += msg->angle_increment;
		}
	  this->alg_.unlock();
	}
	else
	{
		ROS_ERROR("ForceRobotCompanionAlgNode::No transform in laser callback function");
	}
  }
  catch (tf::TransformException &ex)
  {
	ROS_ERROR("ForceRobotCompanionLaser:: %s",ex.what());
  }

  //unlock previously blocked shared variables 
  ready_to_read_laser_ = false;
  this->laser_mutex_.exit(); 
}
void ForceRobotCompanionAlgNode::dest_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{ 
  //ROS_INFO("ForceRobotCompanionAlgNode::dest_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
  //this->dest_mutex_.enter(); 

  
  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  //this->dest_mutex_.exit(); 
}
void ForceRobotCompanionAlgNode::joy_callback(const sensor_msgs::Joy::ConstPtr& msg) 
{ 
  //ROS_INFO("joy_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
  //this->joy_mutex_.enter();
  
	//Initialization
	static bool first=true;
	if(first)
	{
		prev_buttons_.resize(msg->buttons.size());
		first=false;
	}

	//assign keys if necessary
	for(unsigned int i=0; i<msg->buttons.size(); i++)
	{
		if(msg->buttons[i]==1 && prev_buttons_[i]==0)
		{
		switch(i)
		{
			case wiimote::State::MSG_BTN_UP:
				ROS_INFO(" ++ UP : robot comes nearer (more force to the person)");
				//pred_.robot_parameters_feedback_adjustment(1.0 );
				if( !using_prediction_ )
				{
					//pred_.set_force_params( 0.0, 1.0 );
				}
				//pred_.get_force_params(  );
				break;

			case wiimote::State::MSG_BTN_DOWN:
				ROS_INFO(" -- DOWN : robot stays away (more force to destination)");
				//pred_.robot_parameters_feedback_adjustment(-1.0);
				if( !using_prediction_ )
				{
					//pred_.set_force_params( 0.0, 1.0 );
				}
				break;
				
			case wiimote::State::MSG_BTN_A:
				ROS_INFO("Stop robot approach");
				pred_.set_target_person( 0 );
				robot_stop_ = true;
				break;
				
			case wiimote::State::MSG_BTN_B:
				ROS_INFO("Following nearest target (id: %d)", nearest_target_);
                pred_.set_target_person( nearest_target_ );
				break;

			default:
				break;
		}
		 std::vector<double> force_params = pred_.get_force_params();
		ROS_INFO("Force params : goal = %f, person = %f, int = %f" , 
				force_params[0],force_params[1],force_params[2] );
		force_param_msg_.point.x = force_params[0];
		force_param_msg_.point.y = force_params[1];
		force_param_msg_.point.z = force_params[2];

			
		}
	}
	prev_buttons_ = msg->buttons;
	
  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  //this->joy_mutex_.exit(); 
}
void ForceRobotCompanionAlgNode::tracks_callback(const iri_perception_msgs::peopleTrackingArray::ConstPtr& msg) 
{ 
  //ROS_INFO("tracks_callback: New Message Received"); 
  nearest_target_=0;

  //use appropiate mutex to shared variables if necessary 
  this->tracks_mutex_.enter();
  
	try
    {
		//get transformation
		//ROS_INFO("observation frame = %s" , msg->header.frame_id.c_str() );
		//ROS_INFO("target frame  = %s" , target_frame_.c_str() );
		/*
		bool 	waitForTransform (const std::string &target_frame,
		const std::string &source_frame, const ros::Time &time, const ros::Duration &timeout, 
		const ros::Duration &polling_sleep_duration=ros::Duration(0.01), 
		std::string *error_msg=NULL) const 
		*/
		bool tf_exists = tf_listener_.waitForTransform(target_frame_,
								msg->header.frame_id, 
								msg->header.stamp,
								ros::Duration(1), ros::Duration(0.01));
								
		if(tf_exists)
		{
			//Tracks detection observations
			vector<SdetectionObservation> obs;
			geometry_msgs::PointStamped observation_point, target_point;
			observation_point.header = msg->header;
			observation_point.point.z = 0.0;
			float minDist_=10;
			for(unsigned int i = 0; i < msg->peopleSet.size() ; ++i)
			{
				observation_point.point.x = msg->peopleSet[i].x;
				observation_point.point.y = msg->peopleSet[i].y;
				
				//distance to each person, save nearest_target_ plus other conditions (x>0)
				float dist = sqrt(observation_point.point.x*observation_point.point.x + observation_point.point.y*observation_point.point.y);
				if(minDist_ > dist )//&&  observation_point.point.x > 0)
				{
				  nearest_target_= msg->peopleSet[i].targetId;
                                  minDist_ = dist;
				}
				
				/*void 	transformPoint (const std::string &target_frame, 
					const geometry_msgs::PointStamped &stamped_in, 
					geometry_msgs::PointStamped &stamped_out) const 
				*/
				//tf_listener_.transformPoint( target_frame_,  msg->header.stamp, target_point, msg->header.frame_id, observation_point);
				tf_listener_.transformPoint( target_frame_, observation_point, target_point);
				//SdetectionObservation( int id , double time_stamp ,double x, double y, double vx, double vy)
				obs.push_back( SdetectionObservation(  msg->peopleSet[i].targetId , 
										msg->header.stamp.toSec() ,   target_point.point.x ,
										target_point.point.y , 0.0 , 0.0	) );
			}
			this->alg_.lock(); 
			pred_.update_scene( obs );
			this->alg_.unlock(); 
			
			//robot not simulated -> update
			if( !robot_sim_ )
			{
				observation_point.point.x = 0.0;
				observation_point.point.y = 0.0;
				observation_point.header.frame_id=robot_frame_;
				//tf_listener_.transformPoint( target_frame_, msg->header.stamp, target_point, msg->header.frame_id, observation_point);
				tf_listener_.transformPoint( target_frame_, observation_point, target_point);
		//		ROS_INFO("ForceRobotCompanionTracks::robot position (x,y) = (%f , %f)" , target_point.point.x , target_point.point.y);
				this->alg_.lock(); 
				pred_.update_robot( SdetectionObservation( 0, msg->header.stamp.toSec(), 
											target_point.point.x , target_point.point.y , 0.0 , 0.0	) );
				this->alg_.unlock(); 
			}
		}
		else
		{
			ROS_ERROR("ForceRobotCompanionAlgNode::No transform in tracks callback function");
		}
	}
    catch (tf::TransformException &ex)
	{
		ROS_ERROR("ForceRobotCompanionTracks:: %s",ex.what());
	}


	this->tracks_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */
void ForceRobotCompanionAlgNode::move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result) 
{ 
  if( state.toString().compare("SUCCEEDED") == 0 ) 
    ROS_INFO("ForceRobotCompanionAlgNode::move_baseDone: Goal Achieved!"); 
  else 
    ROS_INFO("ForceRobotCompanionAlgNode::move_baseDone: %s", state.toString().c_str()); 

  //copy & work with requested result 
} 

void ForceRobotCompanionAlgNode::move_baseActive() 
{ 
  //ROS_INFO("ForceRobotCompanionAlgNode::move_baseActive: Goal just went active!"); 
} 

void ForceRobotCompanionAlgNode::move_baseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) 
{ 
  //ROS_INFO("ForceRobotCompanionAlgNode::move_baseFeedback: Got Feedback!"); 

  bool feedback_is_ok = true; 

  //analyze feedback 
  //my_var = feedback->var; 

  //if feedback is not what expected, cancel requested goal 
  if( !feedback_is_ok ) 
  { 
    move_base_client_.cancelGoal(); 
    //ROS_INFO("ForceRobotCompanionAlgNode::move_baseFeedback: Cancelling Action!"); 
  } 
}

/*  [action requests] */
void ForceRobotCompanionAlgNode::move_baseMakeActionRequest() 
{ 
//  ROS_INFO("ForceRobotCompanionAlgNode::move_baseMakeActionRequest: Starting New Request!"); 

  //wait for the action server to start 
  //will wait for infinite time 
  move_base_client_.waitForServer();  
//  ROS_INFO("ForceRobotCompanionAlgNode::move_baseMakeActionRequest: Server is Available!"); 

  //send a goal to the action 
	move_base_goal_.target_pose.header.frame_id = target_frame_;
	move_base_goal_.target_pose.header.stamp = ros::Time::now();
	move_base_goal_.target_pose.pose.position.x = robot_desired_position_.x;
	move_base_goal_.target_pose.pose.position.y = robot_desired_position_.y;
  
  move_base_client_.sendGoal(move_base_goal_, 
              boost::bind(&ForceRobotCompanionAlgNode::move_baseDone,     this, _1, _2), 
              boost::bind(&ForceRobotCompanionAlgNode::move_baseActive,   this), 
              boost::bind(&ForceRobotCompanionAlgNode::move_baseFeedback, this, _1)); 
 // ROS_INFO("ForceRobotCompanionAlgNode::move_baseMakeActionRequest: Goal Sent. Wait for Result!"); 
}

void ForceRobotCompanionAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  
  	ROS_INFO("         *******  algorithm config update  *******\n\n");
	move_base_mode_ = config.move_base;
	target_frame_ = config.reference_frame;
	robot_frame_ = config.robot_frame;
	if( config.target_person_id != pred_.get_target_person() )
		traj_marker_.points.clear();
	pred_.set_target_person( config.target_person_id );//updated via dynamic reconfigure. Def = 0
	pred_.set_v_max( config.v_max );
	pred_.set_v_cruise( config.v_cruise );
	pred_.set_time_horizon( config.time_horizon );
	if( config.force_parameters )
	{
		pred_.set_force_params( config.force_goal, config.force_toperson, config.force_interaction ,
								config.force_laser, config.force_map);
		force_param_msg_.point.x = config.force_goal;
		force_param_msg_.point.y = config.force_toperson;
		force_param_msg_.point.z = config.force_interaction;
	}
	laser_radi_ = config.laser_radi;
	pred_.set_companion_position(config.r , config.theta/180*3.14);
	using_prediction_ = config.using_prediction;
	if(config.follow_nearest)
	{
		ROS_INFO("Following nearest target (id: %d)", nearest_target_);
                pred_.set_target_person( nearest_target_ );
		config.target_person_id = nearest_target_;
	}
	force_map_path_ = config.force_map_path;
  destination_map_path_ = config.destination_map_path;
	//robot-person parameters
	vector<double> param;
	param.push_back(config.K_rob_per);//K
	param.push_back(config.lambda_rob_per);//lambda
	param.push_back(config.A_rob_per);//A
	param.push_back(config.B_rob_per);//B
	param.push_back(config.d_rob_per);//d
	pred_.get_robot()->set_social_force_parameters_person_robot( param );
	//Obstacle spherical parameters obtained using our optimization method
	param.clear();
	param.push_back(config.K_obstacle);//K
	param.push_back(config.lambda_obs);//lambda
	param.push_back(config.A_obstacle);//A
	param.push_back(config.B_obstacle);//B
	param.push_back(config.d_obstacle);//d
	pred_.get_robot()->set_social_force_parameters_obstacle( param );
	if( config.clear_target_traj )
		traj_marker_.points.clear();
		traj_robot_marker_.points.clear();
	robot_x_ini_ = config.robot_x_ini;
	robot_y_ini_ = config.robot_y_ini;
  this->alg_.unlock();
}

void ForceRobotCompanionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<ForceRobotCompanionAlgNode>(argc, argv, "force_robot_companion_alg_node");
}
