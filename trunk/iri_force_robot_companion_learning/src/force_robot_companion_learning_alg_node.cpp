#include "force_robot_companion_learning_alg_node.h"
#include <deque>
#include <stdio.h>
//#include <string.h>

ForceRobotCompanionLearningAlgNode::ForceRobotCompanionLearningAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<ForceRobotCompanionLearningAlgorithm>() ,
	tf_listener_(ros::Duration(30.f)),
    nearest_target_(0),
	simulation_state_(ForceRobotCompanionLearningAlgNode::Init_Sim),
	number_of_virtual_people_(0),
	num_experiment_(0),
	results_file_is_open_(false),
    learning_mode_( ForceRobotCompanionLearningAlgNode::Density )
{

  //init class attributes if necessary
  this->loop_rate_ = 5;//in [Hz]

  // [init publishers]
  this->angle_robot_person_publisher_ = this->public_node_handle_.advertise<std_msgs::Float64>("angle_robot_person", 100);
  this->person_robot_dist_publisher_ = this->public_node_handle_.advertise<std_msgs::Float64>("person_robot_dist", 100);
  //this->target_pred_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseArray>("target_pred", 100);
  this->target_person_position_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PointStamped>("target_position", 100);
  this->robot_position_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("robot_position", 100);
  this->force_param_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PointStamped>("force_param", 100);
  this->trajectories_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/trajectories", 100);
  this->forces_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/forces", 100);
  this->destinations_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/destinations", 100);
  this->predictions_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/predictions", 100);
  init_node();
  // [init subscribers]
//  this->laser_subscriber_ = this->public_node_handle_.subscribe("laser", 100, &ForceRobotCompanionLearningAlgNode::laser_callback, this);
  this->dest_subscriber_ = this->public_node_handle_.subscribe("dest", 100, &ForceRobotCompanionLearningAlgNode::dest_callback, this);
  this->tracks_subscriber_ = this->public_node_handle_.subscribe("tracks", 100, &ForceRobotCompanionLearningAlgNode::tracks_callback, this);
  
  // [init services]
  //parameter import example
  //private_node_handle.param<std:tring>("scanTopicString", scanTopicString, "scan");
  
  // [init clients]
  reset_client_ = this->public_node_handle_.serviceClient<std_srvs::Empty>("reset");
  
  // [init action servers]
  
  // [init action clients]
}

ForceRobotCompanionLearningAlgNode::~ForceRobotCompanionLearningAlgNode(void)
{
  // [free dynamic memory]
	if ( results_file_is_open_ )
		fclose( file_results_out_ );
}

void ForceRobotCompanionLearningAlgNode::mainNodeThread(void)
{

  // [fill msg structures]
  //this->Float64MultiArray_msg_.data = my_var;
	//prediction
    this->alg_.lock();
	pred_.scene_intentionality_prediction_bhmip();
	//Visualization
	now_ = ros::Time::now();
  	vis_trajectories();
	vis_destinations();
//	vis_intentionality_prediction();
// 	vis_predictions();
	
	
  // [fill srv structure and make request to the server]

  
  // [fill action structure and make request to the action server]



    angle_robot_person_msg_.data = target_followed_pose_.angle_heading_point( pred_.get_robot()->get_current_pointV() );
	switch( learning_mode_ )
	{
    // growing density, code not tested under the new structure TODO
    case ForceRobotCompanionLearningAlgNode::Density:
      growing_density_test();
      break;


    // learning mode: fixed density and learning of parameters
    case ForceRobotCompanionLearningAlgNode::Initial_paramters :
    default:
        initial_params_learning();
        break;
	}
  //ROS_INFO("Robot state = %d || (Init 0, Find 1, Test 2, End 3)  " , simulation_state_);

  this->alg_.unlock();


  // [publish messages]
  this->angle_robot_person_publisher_.publish(this->angle_robot_person_msg_);
  this->person_robot_dist_publisher_.publish(this->person_robot_dist_msg_);
  force_param_msg_.header.stamp = now_;
    this->target_person_position_publisher_.publish(this->target_person_position_msg_);
    this->robot_position_publisher_.publish(this->robot_position_msg_);
  this->force_param_publisher_.publish(this->force_param_msg_);
	this->trajectories_publisher_.publish(this->MarkerArray_trajectories_msg_);
	this->forces_publisher_.publish(this->MarkerArray_forces_msg_);
	this->destinations_publisher_.publish(this->MarkerArray_destinations_msg_);
	this->predictions_publisher_.publish(this->MarkerArray_predictions_msg_);
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */








//common fields, to be filled just once...
void ForceRobotCompanionLearningAlgNode::init_node()
{
  pred_.set_dt( 1.0 / 5.0 );
  this->public_node_handle_.getParam("reference_frame", target_frame_);
	this->public_node_handle_.getParam("force_map_path", force_map_path_);
    this->public_node_handle_.getParam("destination_map_path", destination_map_path_);
	if ( !pred_.read_force_map(  force_map_path_.c_str() ) )
	{
		ROS_WARN("Could not read map force file !!!");
	}
	this->public_node_handle_.getParam("robot_x_ini", robot_x_ini_);
	this->public_node_handle_.getParam("robot_y_ini", robot_y_ini_);


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
	now_ = ros::Time::now();
	Spose robot_current_position( robot_x_ini_,robot_y_ini_, 0.0, now_.toSec() );
	pred_.update_robot( SdetectionObservation( 0, robot_current_position.time_stamp, 
											   robot_current_position.x, robot_current_position.y ,
											   0.0, 0.0	) );
  robot_position_msg_.header.stamp = now_;
  robot_position_msg_.header.frame_id = target_frame_;
  robot_position_msg_.pose.position.x = robot_x_ini_;
  robot_position_msg_.pose.position.y = robot_y_ini_;
	
											   
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


void ForceRobotCompanionLearningAlgNode::growing_density_test()
{
/*		std::stringstream experimentString;
		//update robot virtual position
		//ROS_INFO("robot position (x,y) = (%f , %f)" , robot_desired_position_.x , robot_desired_position_.y);
		if (robot_desired_position_.x !=  robot_desired_position_.x)//when invlaid commands are given, nan positions are sent...
		{
			robot_desired_position_.x = 0.0;
			robot_desired_position_.y = 0.0;
		}
		pred_.update_robot( SdetectionObservation( 0, ros::Time::now().toSec(), 
									   robot_desired_position_.x, robot_desired_position_.y ,
									   0.0, 0.0	) );


		//automation of the simulation mode: 4 states {Init_Sim, Find_nearest, Test_Sim, End_Sim}
		switch(simulation_state_)
		{
			case ForceRobotCompanionLearningAlgNode::Init_Sim:
				//update next experiments paramters
				timer_ = 0;
				++num_experiment_;
				ROS_INFO( "experiment number %d", num_experiment_ );
				experimentString << num_experiment_;
				//open file
				file_results_out_ = fopen (  (results_folder_ + experimentString.str() ).c_str() , "w");
				if (file_results_out_ == NULL)
					results_file_is_open_ = false;
				else{
					results_file_is_open_ = true;
					fprintf (file_results_out_, "%%distance_robot-person, robot_angle_position_wrt_personheading, distance_to_nearest_person, v_robot, v_person, time\n");
				}
				simulation_state_ = ForceRobotCompanionLearningAlgNode::Find_Nearest;
				break;
			case ForceRobotCompanionLearningAlgNode::Find_Nearest:
				//checks persons have been initialized (tracks_callback has entered) and
				//looks for the nearest person
				if( pred_.set_nearest_target_person() ) //if someone nearer 20m
					simulation_state_ = ForceRobotCompanionLearningAlgNode::Test_Sim;
				break;
			case ForceRobotCompanionLearningAlgNode::Test_Sim:
				//write results
				if ( results_file_is_open_ )
				{
					fprintf (file_results_out_, "%f %f %f %f %f %f\n" ,
								person_robot_dist_msg_.data, 
								target_followed_pose_.angle_heading_pose( pred_.get_robot().get_current_pointV() ), 
								pred_.distance_to_nearest_person( pred_.get_robot() ),
								pred_.get_robot().get_current_pointV().v , 
								target_followed_pose_.v , 
								ros::Time::now().toSec());
				}
				//check if target has been removed == it has achieved its goal
				if( !pred_.find_person( pred_.get_target_person() ) )
					simulation_state_ = End_Sim;
				//if timer exceeds 5 minutes, experiment not valid (probably entering local minima when obstacles in the environment)
				if( timer_ > 5*60*5 ){
					simulation_state_ = ForceRobotCompanionLearningAlgNode::End_Sim;
					--num_experiment_;
				}
				else
					++timer_;
				break;
			case ForceRobotCompanionLearningAlgNode::End_Sim:
			default:
				//close results file
				if ( results_file_is_open_ ){
					fclose( file_results_out_ );
					results_file_is_open_ = false;
				}
				//clear all vectors etc
				pred_.set_number_virtual_people( 0 );
				pred_.clear_people_scene();
				pred_.get_robot().reset();
				pred_.update_robot( SdetectionObservation( 0, ros::Time::now().toSec(), 
									   robot_x_ini_, robot_y_ini_ ,
									   0.0, 0.0	) );
				target_followed_pose_ = Spose();
				traj_marker_.points.clear();
				traj_robot_marker_.points.clear();
				simulation_state_ = ForceRobotCompanionLearningAlgNode::Init_Sim;
				break;
		}
*/
}

void ForceRobotCompanionLearningAlgNode::initial_params_learning()
{

  std::stringstream experimentString;
  vector<double> params;
  switch(simulation_state_)
	{
		case ForceRobotCompanionLearningAlgNode::Init_Sim:

      //update next experiments paramters
      timer_ = 0;
      ++num_experiment_;
      ROS_INFO( "experiment number %d", num_experiment_ );
      experimentString << num_experiment_;
      //open file
      file_results_out_ = fopen (  (results_folder_ + experimentString.str() ).c_str() , "w");
      if (file_results_out_ == NULL)
      {
	      results_file_is_open_ = false;
        break;
      }
      else{
	      results_file_is_open_ = true;
	      fprintf (file_results_out_, "%%distance_robot-person, robot_angle_position_wrt_personheading, distance_to_nearest_person, v_robot, v_person, time\n");
      }


      //get sim paramters
      //params = pred_.learning_force_companion_params_MC( );
      params = pred_.learning_force_companion_params_alphabeta( );
      if( results_file_is_open_ )
      {
        simulation_state_ = ForceRobotCompanionLearningAlgNode::Find_Nearest;
        fprintf (file_results_out_, "%% alpha, beta, gamma,  delta(laser), delta(map)\n");
        fprintf (file_results_out_, "%f %f %f %f %f 0.0\n", params[0],params[1],params[2],params[3],params[4]);
      }
			break;

    case ForceRobotCompanionLearningAlgNode::Find_Nearest:
			//checks persons have been initialized (tracks_callback has entered) and
			//looks for the nearest person
			if( pred_.set_nearest_target_person() ) //if someone nearer 20m
				simulation_state_ = ForceRobotCompanionLearningAlgNode::Test_Sim;
      break;

    case ForceRobotCompanionLearningAlgNode::Test_Sim:
        //update robot position and rosms robot control
	      robot_desired_position_ = pred_.robot_approach_pose( f1_, f2_, f3_, f4_,f5_, f_  );
	      //fill  current robot pose message
	      pred_.update_robot( SdetectionObservation( 0, now_.toSec(), 
											         robot_desired_position_.x, robot_desired_position_.y ,
											         0.0, 0.0	) );
	      robot_position_msg_.pose.position.x = robot_desired_position_.x;
	      robot_position_msg_.pose.position.y = robot_desired_position_.y;
	      robot_position_msg_.header.stamp = now_;

				//write results
				if ( results_file_is_open_ )
				{
					fprintf (file_results_out_, "%f %f %f %f %f %f\n" ,
								person_robot_dist_msg_.data, 
								target_followed_pose_.angle_heading_point( pred_.get_robot()->get_current_pointV() ), 
								pred_.distance_to_nearest_person( pred_.get_robot() ),
								pred_.get_robot()->get_current_pointV().v() , 
								target_followed_pose_.v() , 
								now_.toSec());
				}
				//check if target has been removed == it has achieved its goal
				if( !pred_.find_person( pred_.get_target_person() ) )
					simulation_state_ = End_Sim;
				//if timer exceeds 5 minutes, experiment not valid (probably entering local minima when obstacles in the environment)
				if( timer_ > 5*60*5 ){
					simulation_state_ = ForceRobotCompanionLearningAlgNode::End_Sim;
					--num_experiment_;
				}
				else
					++timer_;
				break;

    case ForceRobotCompanionLearningAlgNode::End_Sim:
    default:
				//close results file
				if ( results_file_is_open_ ){
					fclose( file_results_out_ );
					results_file_is_open_ = false;
				}
              //reset configuration
      pred_.clear_scene();
      pred_.get_robot()->reset();
      pred_.update_robot( SdetectionObservation( 0, now_.toSec(), 
                                robot_x_ini_, robot_y_ini_ ,
                                0.0, 0.0	) );
      target_followed_pose_ = SpointV_cov();
      traj_marker_.points.clear();
      traj_robot_marker_.points.clear();
      //reset people simulation node : send service
      ROS_INFO("ForceRobotCompanionLearningAlgNode:: Sending New Request!"); 
      if (reset_client_.call(reset_srv_)) 
      { 
        ROS_INFO("ForceRobotCompanionLearningAlgNode:: Response: "); 
        simulation_state_ = ForceRobotCompanionLearningAlgNode::Init_Sim;//only changes its state when service is sent
      } 
      else 
      { 
        ROS_INFO("ForceRobotCompanionLearningAlgNode:: Failed to Call Server on topic reset "); 
      }
      break;
  }

}

void ForceRobotCompanionLearningAlgNode::vis_trajectories()
{
	std::vector<SpointV_cov> scene_targets_pointV;
	std::vector<unsigned int> scene_targets_ids;
  pred_.get_scene_observation(scene_targets_pointV,scene_targets_ids);
	//list<Cperson>& scene = pred_.get_scene();
	int cont = 0;
	geometry_msgs::Point ros_point, ros_point_ini;

	//fill headers
	traj_marker_.header.stamp = now_;
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
  robot_position_msg_.pose.orientation = robot_marker_.pose.orientation;
	robot_marker_.id = cont;
	++cont;
	MarkerArray_trajectories_msg_.markers.push_back(  robot_marker_  );
	
	
	//draw pedestrians' trajectories ------------------------------------------------
	//ROS_INFO( "size of trajectories =%d" , scene_targets_pointV.size()  );
	Sforce  force_to_goal, force_int_person , force_int_robot, force_obstacle;
	for( unsigned int i = 0; i < (unsigned int)scene_targets_pointV.size() ; i++)
	{
		SpointV_cov target_pose = scene_targets_pointV[i];

  //no plot of forces....

		if ( scene_targets_ids[i] == (unsigned int)pred_.get_target_person())
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
		//MarkerArray_trajectories_msg_.markers.push_back(  text_marker_  );
		++cont;
	}
	
}

void ForceRobotCompanionLearningAlgNode::clear_force_markers()
{
	force_marker_.points.clear();
	force_goal_marker_.points.clear();
	force_goal2_marker_.points.clear();
	force_int_person_marker_.points.clear();
	force_int_robot_marker_.points.clear();
	force_obstacle_map_marker_.points.clear();
	force_obstacle_laser_marker_.points.clear();
}

void ForceRobotCompanionLearningAlgNode::vis_intentionality_prediction()
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


void ForceRobotCompanionLearningAlgNode::vis_destinations()
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
	dest_marker_.scale.z = 0.01;
    dest_marker_.pose.position.z = 0.0;
	for ( unsigned int i = 0; i < pred_.get_destinations().size(); ++i)
	{
		dest_marker_.pose.position.x = pred_.get_destinations()[i].x;
		dest_marker_.pose.position.y = pred_.get_destinations()[i].y;
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
		dest_marker_.scale.z = iit->get_best_dest().prob;
		dest_marker_.pose.position.z = 0.5*dest_marker_.scale.z;
		dest_marker_.pose.position.x = iit->get_best_dest().x;
		dest_marker_.pose.position.y = iit->get_best_dest().y;
		dest_marker_.id = cont;
		++cont;
		MarkerArray_destinations_msg_.markers.push_back( dest_marker_ );

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

void ForceRobotCompanionLearningAlgNode::vis_predictions()
{
	//a vector of pointers to the predicted trajectories to all destinations...
	 vector< vector<vector<Spose> >* > scene_pred = pred_.get_scene_motion_prediction();
	int cont = 0;

	//fill headers
	pred_marker_.header.frame_id = target_frame_;
	pred_marker_.header.stamp = now_;
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

void ForceRobotCompanionLearningAlgNode::dest_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{ 
  //ROS_INFO("ForceRobotCompanionLearningAlgNode::dest_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
  //this->dest_mutex_.enter(); 

  
  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  //this->dest_mutex_.exit(); 
}

void ForceRobotCompanionLearningAlgNode::tracks_callback(const iri_perception_msgs::peopleTrackingArray::ConstPtr& msg) 
{ 
  //ROS_INFO("tracks_callback: New Message Received"); 


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
            for(unsigned int i = 0; i < msg->peopleSet.size() ; ++i)
            {
                observation_point.point.x = msg->peopleSet[i].x;
                observation_point.point.y = msg->peopleSet[i].y;

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
			
		}
		else
		{
			ROS_ERROR("ForceRobotCompanionLearningAlgNode::No transform in tracks callback function");
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


void ForceRobotCompanionLearningAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  
  	ROS_INFO("         *******  algorithm config update  *******\n\n");
	using_prediction_ = config.using_prediction;
    learning_mode_ = (ForceRobotCompanionLearningAlgNode::learning_mode) config.learning_mode;
  results_folder_ = config.results_folder;
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
	if(config.follow_nearest)
	{
		ROS_INFO("Following nearest target (id: %d)", nearest_target_);
                pred_.set_target_person( nearest_target_ );
		config.target_person_id = nearest_target_;
	}
	force_map_path_ = config.force_map_path;
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


void ForceRobotCompanionLearningAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<ForceRobotCompanionLearningAlgNode>(argc, argv, "force_robot_companion_learning_alg_node");
}
