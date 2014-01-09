#include "force_navigation_learning_alg_node.h"

ForceNavigationLearningAlgNode::ForceNavigationLearningAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<ForceNavigationLearningAlgorithm>()
{
  this->loop_rate_ = 5;//in [Hz]
    this->public_node_handle_.getParam("reference_frame", target_frame_);
    this->public_node_handle_.getParam("robot_x_ini", robot_x_ini_);
    this->public_node_handle_.getParam("robot_y_ini", robot_y_ini_);
    this->public_node_handle_.getParam("force_map_path", force_map_path_);
    this->public_node_handle_.getParam("destination_map_path", destination_map_path_);

  // [init publishers]  
  this->learning_data_publisher_ = this->public_node_handle_.advertise<std_msgs::Float64MultiArray>("learning_data", 100);
  this->robot_simulated_pose_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("robot_simulated_pose", 100);
    this->trajectories_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/trajectories", 100);
    this->forces_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/forces", 100);
    this->destinations_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vis/destinations", 100);

  // [init subscribers]
    this->tracks_subscriber_ = this->public_node_handle_.subscribe("tracks", 100, &ForceNavigationLearningAlgNode::tracks_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
    init_node();
}

ForceNavigationLearningAlgNode::~ForceNavigationLearningAlgNode(void)
{
  // [free dynamic memory]
}

void ForceNavigationLearningAlgNode::mainNodeThread(void)
{
    this->alg_.lock();
    //robot_desired_position_ = force_navigation_.
  // [fill msg structures]
  //this->Float64_msg_.data = my_var;
  //this->PoseStamped_msg_.data = my_var;
    vis_trajectories();
    vis_destinations();

    vector<double> params = force_navigation_.get_force_nav_params();
    double work_robot, work_persons, work_total;

    switch( force_navigation_.get_robot_state() )
    {
        //let the navigsation algorithm work
        case Cforce_navigation::FREE_NAVIGATING :
        case Cforce_navigation::MID_ZONE_NAVIGATING :
        case Cforce_navigation::INNER_ZONE_NAVIGATING : 
            //ROS_INFO("robot state = %d" , force_navigation_.get_robot_state() );
            //ROS_INFO("params = ( %f , %f , %f )" , params[0] , params[1], params[2] );
            robot_desired_position_ = force_navigation_.robot_local_nav();
            PoseStamped_msg_.pose.position.x = robot_desired_position_.x;
            PoseStamped_msg_.pose.position.y = robot_desired_position_.y;
            PoseStamped_msg_.header.stamp = ros::Time::now();
            force_navigation_.update_robot( SdetectionObservation( 0, ros::Time::now().toSec(), 
                        robot_desired_position_.x, robot_desired_position_.y ,
                            0.0, 0.0	) );
            break;

        //new experiment is thrown and the previous is recorded
        case Cforce_navigation::IDLE :
            //save cost function and parameter values (publish topic), additionally set the next experiment parameters
            ROS_INFO("results for iteration %d" , force_navigation_.get_iteration() );
            //params = force_navigation_.learning_force_nav_params_iteration_MC( work_robot, work_persons, work_total );
            //params = force_navigation_.learning_force_nav_params_iteration_MCMC_MH( work_robot, work_persons, work_total );
            params = force_navigation_.learning_force_nav_params_iteration_MCMC_SA( work_robot, work_persons, work_total );
            Float64_msg_.data.clear();           
            Float64_msg_.data = params;
            Float64_msg_.data.push_back( work_robot );
            Float64_msg_.data.push_back( work_persons );
            Float64_msg_.data.push_back( work_total );
            this->learning_data_publisher_.publish(this->Float64_msg_);

            //set new trajectory
            force_navigation_.robot_global_planning( );
            break;

        case Cforce_navigation::UNKNOWN_ZONE : 
        default:
            break;
    }
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

    this->alg_.unlock();
  // [publish messages]
  this->robot_simulated_pose_publisher_.publish(this->PoseStamped_msg_);
    this->trajectories_publisher_.publish(this->MarkerArray_trajectories_msg_);
    this->forces_publisher_.publish(this->MarkerArray_forces_msg_);
    this->destinations_publisher_.publish(this->MarkerArray_destinations_msg_);
}

void ForceNavigationLearningAlgNode::init_node(void)
{

	//cout << "path = " << force_map_path_ << endl;
	if ( !force_navigation_.read_force_map(  force_map_path_.c_str() ) )
	{
		ROS_ERROR("Could not read map force file !!!");
	}
    else{
		ROS_WARN("read map force file : SUCCESS!!!");
	}


    //scene destinations: free environment
	//cout << "path = " << force_map_path_ << endl;
	if ( !force_navigation_.read_destination_map(  destination_map_path_.c_str() ) )
	{
		ROS_ERROR("Could not read map destinations file !!!");
	}
    else{
		ROS_WARN("read map destinations force file : SUCCESS!!!");
	}

	//robot initial update
    force_navigation_.set_dt( 1.0 / 5.0 );
    ros::Time now = ros::Time::now();
	Spose robot_current_position( robot_x_ini_,robot_y_ini_, 0.0, now.toSec() );
    force_navigation_.update_robot( SdetectionObservation( 0, robot_current_position.time_stamp, 
                                robot_current_position.x, robot_current_position.y ,
                                0.0, 0.0	) );
    
    //navigation initialization: important! after setting destinations	
    force_navigation_.stop_navigation();//set robot to IDLE state
    force_navigation_.robot_global_planning( ); //sets a paths a robot state to navigating


    //trajectory marker										   
    traj_marker_.ns = "trajectories";
    traj_marker_.header.frame_id = target_frame_;
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
    traj_robot_marker_.header.frame_id = target_frame_;
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
    robot_marker_.header.frame_id = target_frame_;
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
    target_marker_.header.frame_id = target_frame_;
    target_marker_.type = visualization_msgs::Marker::CYLINDER;
    target_marker_.action = visualization_msgs::Marker::ADD;
    target_marker_.lifetime = ros::Duration(1.0f);
    target_marker_.scale.x = 0.5;
    target_marker_.scale.y = 0.5;
	target_marker_.scale.z = 0.6;
	target_marker_.color.a = 0.4;
	target_marker_.color.r = 0.0;
	target_marker_.color.g = 0.8;
	target_marker_.color.b = 0.0;


    //text marker
    text_marker_.ns = "trajectories";
    text_marker_.header.frame_id = target_frame_;
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
    force_marker_.header.frame_id = target_frame_;
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

    //force markers: map obstacles interaction force (black)
    force_obstacle_map_marker_ = force_marker_;
    force_obstacle_map_marker_.color.r = 0.0;
    force_obstacle_map_marker_.color.g = 0.0;
    force_obstacle_map_marker_.color.b = 0.0;

    //destination marker
    dest_marker_.ns = "destinations";
	dest_marker_.header.frame_id = target_frame_;
    dest_marker_.type = visualization_msgs::Marker::CYLINDER;
    dest_marker_.action = visualization_msgs::Marker::ADD;
    dest_marker_.lifetime = ros::Duration(1.0f);
    dest_marker_.color.a = 0.8;

    //robot pose when simulating
    PoseStamped_msg_.header.frame_id = target_frame_;

}

void ForceNavigationLearningAlgNode::vis_trajectories()
{
    list<Cperson>& scene = force_navigation_.get_scene();
    int cont = 0, cont_f = 0;
    geometry_msgs::Point ros_point, ros_point_ini;

    //fill headers
    traj_marker_.header.stamp = ros::Time::now();
    traj_robot_marker_.header.stamp = traj_marker_.header.stamp;
    robot_marker_.header.stamp = traj_marker_.header.stamp;
    target_marker_.header.stamp = traj_marker_.header.stamp;
    text_marker_.header.stamp = traj_marker_.header.stamp;
    force_marker_.header.stamp = traj_marker_.header.stamp;
    force_goal_marker_.header.stamp = force_marker_.header.stamp;
    force_goal2_marker_.header.stamp = force_marker_.header.stamp;
    force_int_person_marker_.header.stamp = force_marker_.header.stamp;
    force_int_robot_marker_.header.stamp = force_marker_.header.stamp;
    force_obstacle_map_marker_.header.stamp = force_marker_.header.stamp;
    force_obstacle_laser_marker_.header.stamp = force_marker_.header.stamp;

    MarkerArray_trajectories_msg_.markers.clear();
    MarkerArray_forces_msg_.markers.clear();
	

    //drawing robot forces ---------------------------------------------------------------
    clear_force_markers();
    Sforce  force_to_goal, force_int_person , force_int_robot, force_obstacle, force_total;
    force_total = force_navigation_.get_robot().get_forces_person( force_to_goal, force_int_person , force_int_robot, force_obstacle );
    //ROS_INFO("robot force = (%f , %f) ", force_total.fx, force_total.fy );

    //initial point
    ros_point_ini.x = force_navigation_.get_robot().get_current_pose().x;
    ros_point_ini.y = force_navigation_.get_robot().get_current_pose().y;

    //drawing robot trajectory. As we are just plotting the target path, we will read the current pose
    if ( traj_robot_marker_.points.size() > 120)
    {
        vector<geometry_msgs::Point> temp_poses;
        temp_poses.reserve(100);
        for( unsigned int i = 21; i <= 100; ++i)
	        temp_poses.push_back(traj_robot_marker_.points[i]);
        traj_robot_marker_.points = temp_poses;
    }
    traj_robot_marker_.points.push_back(    ros_point_ini   );
    traj_robot_marker_.id = cont;
    ++cont;
    MarkerArray_trajectories_msg_.markers.push_back(  traj_robot_marker_  );
	

    //scaled force to goal: ( blue )
    force_goal_marker_.points.push_back(    ros_point_ini   );
    ros_point.x = ros_point_ini.x + force_to_goal.fx;
    ros_point.y = ros_point_ini.y + force_to_goal.fy;
    force_goal_marker_.points.push_back(    ros_point   );
    force_goal_marker_.id = cont;
    ++cont;
    MarkerArray_trajectories_msg_.markers.push_back(  force_goal_marker_  );

    //scaled person interaction force (green)
    force_int_person_marker_.points.push_back(    ros_point_ini   );
    ros_point.x = ros_point_ini.x + force_int_person.fx;
    ros_point.y = ros_point_ini.y + force_int_person.fy;
    force_int_person_marker_.points.push_back( ros_point);
    force_int_person_marker_.id = cont;
    ++cont;
    MarkerArray_trajectories_msg_.markers.push_back(  force_int_person_marker_  );

    //map obstacles interaction force (black)
    force_obstacle_map_marker_.points.push_back(    ros_point_ini   );
    ros_point.x = ros_point_ini.x + force_obstacle.fx;
    ros_point.y = ros_point_ini.y + force_obstacle.fy;
    force_obstacle_map_marker_.points.push_back( ros_point);
    force_obstacle_map_marker_.id = cont;
    ++cont;
    MarkerArray_trajectories_msg_.markers.push_back(  force_obstacle_map_marker_  );

    //weighted resultant force (red)
    force_marker_.points.push_back(    ros_point_ini   );
    ros_point.x = ros_point_ini.x + force_total.fx;
    ros_point.y = ros_point_ini.y + force_total.fy;
    force_marker_.points.push_back(  ros_point);
    force_marker_.id = cont;
    ++cont;
    MarkerArray_trajectories_msg_.markers.push_back(  force_marker_  );

    //Draw robot as a (pink) .stl mesh
    double robot_orientation = force_navigation_.get_robot().get_current_pose().theta - PI/2.0;
    robot_marker_.pose.position.x = ros_point_ini.x + 0.3*cos(robot_orientation);
    robot_marker_.pose.position.y = ros_point_ini.y + 0.3*sin(robot_orientation);
    robot_marker_.pose.position.z = 0.4;
    //geometry_msgs::Quaternion  tf::createQuaternionMsgFromRollPitchYaw(double roll,double pitch,double yaw)
    robot_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI/2.0,0.0, robot_orientation );
    robot_marker_.id = cont;
    ++cont;
    MarkerArray_trajectories_msg_.markers.push_back(  robot_marker_  );


    //draw pedestrians' trajectories ------------------------------------------------
    //ROS_INFO( "size of trajectories =%d" , scene.size()  );
    //Sforce  force_to_goal, force_int_person , force_int_robot, force_obstacle;
    for( list<Cperson>::iterator iit = scene.begin() ; iit != scene.end() ; iit++)
    {
        Spose target_pose = iit->get_current_pose();

        //drawing person forces ---------------------------------------------------------------
        clear_force_markers();
        force_total = iit->get_forces_person( force_to_goal, force_int_person , force_int_robot, force_obstacle );

        //initial point
        ros_point_ini.x = target_pose.x;
        ros_point_ini.y = target_pose.y;

        //scaled force to goal: ( blue )
        force_goal_marker_.points.push_back(    ros_point_ini   );
        ros_point.x = ros_point_ini.x + force_to_goal.fx;
        ros_point.y = ros_point_ini.y + force_to_goal.fy;
        force_goal_marker_.points.push_back(    ros_point   );
        force_goal_marker_.id = cont_f;
        ++cont_f;
        MarkerArray_forces_msg_.markers.push_back(  force_goal_marker_  );

        //scaled person interaction force (green)
        force_int_person_marker_.points.push_back(    ros_point_ini   );
        ros_point.x = ros_point_ini.x + force_int_person.fx;
        ros_point.y = ros_point_ini.y + force_int_person.fy;
        force_int_person_marker_.points.push_back( ros_point);
        force_int_person_marker_.id = cont_f;
        ++cont_f;
        MarkerArray_forces_msg_.markers.push_back(  force_int_person_marker_  );

        //scaled robot force (pink)
        force_int_robot_marker_.points.push_back(    ros_point_ini   );
        ros_point.x = ros_point_ini.x + force_int_robot.fx;
        ros_point.y = ros_point_ini.y + force_int_robot.fy;
        force_int_robot_marker_.points.push_back( ros_point);
        force_int_robot_marker_.id = cont_f;
        ++cont_f;
        MarkerArray_forces_msg_.markers.push_back(  force_int_robot_marker_  );

        //map obstacles interaction force (black)
        force_obstacle_map_marker_.points.push_back(    ros_point_ini   );
        ros_point.x = ros_point_ini.x + force_obstacle.fx;
        ros_point.y = ros_point_ini.y + force_obstacle.fy;
        force_obstacle_map_marker_.points.push_back( ros_point);
        force_obstacle_map_marker_.id = cont_f;
        ++cont_f;
        MarkerArray_forces_msg_.markers.push_back(  force_obstacle_map_marker_  );

        //weighted resultant force (red)
        force_marker_.points.push_back(    ros_point_ini   );
        ros_point.x = ros_point_ini.x + force_total.fx;
        ros_point.y = ros_point_ini.y + force_total.fy;
        force_marker_.points.push_back(  ros_point);
        force_marker_.id = cont_f;
        ++cont_f;
        MarkerArray_forces_msg_.markers.push_back(  force_marker_  );

		//draw targets as cylinders
		target_marker_.pose.position.x = target_pose.x;
		target_marker_.pose.position.y = target_pose.y;
		target_marker_.pose.position.z = 0.3;
		target_marker_.id = cont;
		++cont;
		MarkerArray_trajectories_msg_.markers.push_back(  target_marker_  );
		
		//draw target id
		text_marker_.pose.position.x = target_pose.x;
		text_marker_.pose.position.y = target_pose.y;
		text_marker_.id = cont_f;
		++cont_f;
		std::ostringstream target_id;
		target_id << iit->get_id();
		text_marker_.text = target_id.str();
		MarkerArray_forces_msg_.markers.push_back(  text_marker_  );
		++cont;
	}
	
}

void ForceNavigationLearningAlgNode::clear_force_markers()
{
	force_marker_.points.clear();
	force_goal_marker_.points.clear();
	force_goal2_marker_.points.clear();
	force_int_person_marker_.points.clear();
	force_int_robot_marker_.points.clear();
	force_obstacle_map_marker_.points.clear();
	force_obstacle_laser_marker_.points.clear();
}

void ForceNavigationLearningAlgNode::vis_destinations()
{
	MarkerArray_destinations_msg_.markers.clear();
	int cont = 0;
	
	//Print scene destinations
	dest_marker_.color.r = 0.0;
	dest_marker_.color.g = 0.4;
	dest_marker_.color.b = 1.0;
	dest_marker_.scale.x = 1;
	dest_marker_.scale.y = 1;
	dest_marker_.scale.z = 0.2;
    dest_marker_.pose.position.z = 0.1;
	for ( unsigned int i = 0; i < force_navigation_.get_destinations().size(); ++i)
	{
		dest_marker_.pose.position.x = force_navigation_.get_destinations()[i].x;
		dest_marker_.pose.position.y = force_navigation_.get_destinations()[i].y;
		dest_marker_.id = cont;
		++cont;
		MarkerArray_destinations_msg_.markers.push_back( dest_marker_ );
		//draw target id
		text_marker_.pose = dest_marker_.pose;
		text_marker_.id = cont;
		++cont;
		std::ostringstream target_id;
		target_id << force_navigation_.get_destinations()[i].id;
		text_marker_.text = target_id.str();
		MarkerArray_destinations_msg_.markers.push_back(  text_marker_  );
	}
	
	//print robot set of destinations
	dest_marker_.color.r = 1.0;
	dest_marker_.color.g = 0.6;
	dest_marker_.color.b = 0.0;
	dest_marker_.scale.x = 0.8;
	dest_marker_.scale.y = 0.8;
	dest_marker_.scale.z = 0.2;
    dest_marker_.pose.position.z = 0.3;
	for ( unsigned int i = 0; i < force_navigation_.get_global_planning_destinations().size(); ++i)
	{
		dest_marker_.pose.position.x = force_navigation_.get_global_planning_destinations()[i].x;
		dest_marker_.pose.position.y = force_navigation_.get_global_planning_destinations()[i].y;
		dest_marker_.id = cont;
		++cont;
		MarkerArray_destinations_msg_.markers.push_back( dest_marker_ );
	}
	
	//print current robot destination
    if( force_navigation_.get_global_planning_destinations().size() > 0)
    {
        dest_marker_.color.r = 0.0;
        dest_marker_.color.g = 0.9;
        dest_marker_.color.b = 0.9;
        dest_marker_.scale.x = 0.5;
        dest_marker_.scale.y = 0.5;
        dest_marker_.scale.z = 1.0;
        dest_marker_.pose.position.z = 0.5*dest_marker_.scale.z;
        dest_marker_.pose.position.x = force_navigation_.get_global_planning_destinations().back().x;
        dest_marker_.pose.position.y = force_navigation_.get_global_planning_destinations().back().y;
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

/*  [subscriber callbacks] */
void ForceNavigationLearningAlgNode::tracks_callback(const iri_perception_msgs::peopleTrackingArray::ConstPtr& msg) 
{ 
  //ROS_INFO("ForceNavigationLearningAlgNode::tracks_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->tracks_mutex_.enter(); 

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
            force_navigation_.update_scene( obs );
            this->alg_.unlock(); 
		}
		else
		{
			ROS_ERROR("ForceNavigationLearningAlgNode::No transform in tracks callback function");
		}
	}
    catch (tf::TransformException &ex)
	{
		ROS_ERROR("ForceNavigationLearningAlgNode:: %s",ex.what());
	}

  //unlock previously blocked shared variables 
  //this->tracks_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */


void ForceNavigationLearningAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  	ROS_INFO("         *******  algorithm config update  *******\n\n");
	target_frame_ = config.reference_frame;
	robot_frame_ = config.robot_frame;
	force_navigation_.set_v_max( config.v_max );
	force_navigation_.set_v_cruise( config.v_cruise );
    force_navigation_.set_time_horizon( config.time_horizon );

    robot_x_ini_ = config.robot_x_ini;
    robot_y_ini_ = config.robot_y_ini;

  this->alg_.unlock();
}

void ForceNavigationLearningAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<ForceNavigationLearningAlgNode>(argc, argv, "force_navigation_learning_alg_node");
}
