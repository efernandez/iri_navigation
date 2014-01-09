#include "localization3d_alg_node.h"

Localization3dAlgNode::Localization3dAlgNode(void)
{
	//sleep(1);//time to allow tf to be ready
		
	//init class attributes if necessary
	infoGain = 0;

	// [init publishers]
	this->tf_publisher_ = this->public_node_handle_.advertise<tf::tfMessage>("odom_to_map", 100);
	this->particleSet_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseArray>("particleSet", 100);
	this->position_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("position", 100);
	
	// [init subscribers]
	this->platformData_subscriber_ = this->public_node_handle_.subscribe("platform_data", 100, &Localization3dAlgNode::platformData_callback, this);
	this->platformOdometry_subscriber_ = this->public_node_handle_.subscribe("platform_odom", 100, &Localization3dAlgNode::platformOdometry_callback, this);
	this->laser0_subscriber_ = this->public_node_handle_.subscribe("laser0/scan", 100, &Localization3dAlgNode::laser0_callback, this);
	this->laser1_subscriber_ = this->public_node_handle_.subscribe("laser1/scan", 100, &Localization3dAlgNode::laser1_callback, this);
	this->laser2_subscriber_ = this->public_node_handle_.subscribe("laser2/scan", 100, &Localization3dAlgNode::laser2_callback, this);
	
	// [init services]
	
	// [init clients]
	
	// [init action servers]
	
	// [init action clients]
	
	//initializes odometry time stamp to now
	odometry.setTimeStamp(); 
		
	//gets user parameters
	//std::cout << "CONFIG = " << alg_.config_.num_particles << std::endl; //doesn't work beacuse config_update is not yet called
	double lp_rt; 
	this->public_node_handle_.getParam("filter_rate", lp_rt); 
	this->loop_rate_ = lp_rt;
	transform_tolerance_.fromSec(1.0/lp_rt+0.1);//duration considering loc tf as good: to allow tf extrapolation
	this->public_node_handle_.getParam("num_particles", numberOfParticles);
	this->public_node_handle_.getParam("resampling_style", resamplingStyle);
	this->public_node_handle_.getParam("mapFile", mapFileName);
	this->public_node_handle_.getParam("floorGridFile", gridFileName);
	this->public_node_handle_.getParam("initX", initX);
	this->public_node_handle_.getParam("initY", initY);
	this->public_node_handle_.getParam("initH", initH);
	
	this->public_node_handle_.getParam("odo_error_XY",odometry.error_factor_XY);
	this->public_node_handle_.getParam("odo_error_H",odometry.error_factor_H);
	this->public_node_handle_.getParam("odo_error_P",odometry.error_factor_P);
	this->public_node_handle_.getParam("odo_error_R",odometry.error_factor_R);
	
	this->public_node_handle_.getParam("sigma_heading",inclinometers.sigmaH);
	this->public_node_handle_.getParam("sigma_pitch_roll",inclinometers.sigmaP);
	this->public_node_handle_.getParam("sigma_pitch_roll",inclinometers.sigmaR);
	
	this->public_node_handle_.getParam("l0_frameName",rDevConfig[0].frameName);
	this->public_node_handle_.getParam("l0_typeId",rDevConfig[0].deviceType);
	if (rDevConfig[0].deviceType == 0)
	{
		this->public_node_handle_.getParam("l0_nRays",rDevConfig[0].nRays);
		this->public_node_handle_.getParam("l0_aperture",rDevConfig[0].aperture);
		this->public_node_handle_.getParam("l0_rMin",rDevConfig[0].rangeMin);
		this->public_node_handle_.getParam("l0_rMax",rDevConfig[0].rangeMax);
		this->public_node_handle_.getParam("l0_sigma_range",rDevConfig[0].stdDev);
	}
	this->public_node_handle_.getParam("l1_frameName",rDevConfig[1].frameName);
	this->public_node_handle_.getParam("l1_typeId",rDevConfig[1].deviceType);
	if (rDevConfig[1].deviceType == 0)
	{
		this->public_node_handle_.getParam("l1_nRays",rDevConfig[1].nRays);
		this->public_node_handle_.getParam("l1_aperture",rDevConfig[1].aperture);
		this->public_node_handle_.getParam("l1_rMin",rDevConfig[1].rangeMin);
		this->public_node_handle_.getParam("l1_rMax",rDevConfig[1].rangeMax);
		this->public_node_handle_.getParam("l1_sigma_range",rDevConfig[1].stdDev);
	}
	this->public_node_handle_.getParam("l2_frameName",rDevConfig[2].frameName);
	this->public_node_handle_.getParam("l2_typeId",rDevConfig[2].deviceType);
	if (rDevConfig[2].deviceType == 0)
	{
		this->public_node_handle_.getParam("l2_nRays",rDevConfig[2].nRays);
		this->public_node_handle_.getParam("l2_aperture",rDevConfig[2].aperture);
		this->public_node_handle_.getParam("l2_rMin",rDevConfig[2].rangeMin);
		this->public_node_handle_.getParam("l2_rMax",rDevConfig[2].rangeMax);
		this->public_node_handle_.getParam("l2_sigma_range",rDevConfig[2].stdDev);
	}

	//prints user's config
	printUserConfiguration();

	//init on-board laser mounting positions
	//sleep(1);
	initDevicePositions();
	
	//Allocates memory to run the filter 
	int debugMode;
	this->public_node_handle_.getParam("debug_mode",debugMode);
	//std::cout << "localization3d_alg_node.cpp:" << __LINE__ << ": debug_mode = "<< debugMode << std::endl;			
	pFilter = new CbasicPF(gridFileName, mapFileName, (bool)debugMode);
	for (unsigned int ii=0; ii<MAX_RANGE_DEVICES; ii++)
	{
		if(rDevConfig[ii].frameName != "" )//device has been configured
		{
			if(rDevConfig[ii].deviceType == 0 ) //device type not specified -> use paremetric description
			{
				pFilter->addRangeModel(2, rDevConfig[ii].nRays, rDevConfig[ii].aperture, rDevConfig[ii].aperture/(double)rDevConfig[ii].nRays,rDevConfig[ii].rangeMin,rDevConfig[ii].rangeMax,ii);
			}
			else //device type has been specified
			{
				pFilter->addRangeModel(rDevConfig[ii].deviceType,ii);
			}
		}
	}

	//initializes the filter with an initial estimate
	pFilter->trackingInit(numberOfParticles, initX, initY, initH);
	
}

Localization3dAlgNode::~Localization3dAlgNode(void)
{
	// [free dynamic memory]
	delete pFilter;
}

void Localization3dAlgNode::initDevicePositions()
{
	tf::TransformListener tfListener;
	tf::StampedTransform laserWRTbase;

//std::cout << "Localization3dAlgNode:" << __LINE__ << std::endl;
//std::string eMsg;

	//get device mounting point with respect to the platform (base link)
	for (unsigned int ii=0; ii<MAX_RANGE_DEVICES; ii++)
	{
		if(rDevConfig[ii].frameName != "" )
		{
			tfListener.waitForTransform("base_link", rDevConfig[ii].frameName, ros::Time(0), ros::Duration(10.0),ros::Duration(1.0));
			//tfListener.waitForTransform("base_link", rDevConfig[ii].frameName, ros::Time(0), ros::Duration(10.0),ros::Duration(1.0), & eMsg);
			//std::cout << "Localization3dAlgNode:" << __LINE__ << ": " << eMsg << std::endl;
			tfListener.lookupTransform("base_link", rDevConfig[ii].frameName, ros::Time(0), laserWRTbase);
			laserObs[ii].mountingPosition.setXYZ(laserWRTbase.getOrigin().x(),laserWRTbase.getOrigin().y(),laserWRTbase.getOrigin().z());
			laserObs[ii].mountingPosition.setQuaternion(laserWRTbase.getRotation().getW(),laserWRTbase.getRotation().getX(),laserWRTbase.getRotation().getY(),laserWRTbase.getRotation().getZ());
			laserObs[ii].mountingPosition.printPosition();
		}
	}
}

void Localization3dAlgNode::printUserConfiguration()
{
	std::cout << "********************** PARTICLE FILTER CONFIGURATION **************************" << std::endl;
	std::cout << "Number Of Particles = \t " << numberOfParticles << std::endl;
	//std::cout << "Filter Rate (wished) = \t " << this->loop_rate_ << std::endl;
	std::cout << "Init (X,Y,H) = \t" << "(" << initX << "," << initY << "," << initH << ")" << std::endl;
	std::cout << "Map File = \t" << mapFileName << std::endl;
	std::cout << "Grid File = \t" << gridFileName << std::endl;
	
	for (unsigned int ii=0; ii<MAX_RANGE_DEVICES; ii++)
	{
		if(rDevConfig[ii].frameName != "" )
		{
			std::cout << "RANGE DEVICE " << ii << std::endl;
			std::cout << "\t Frame Name = \t" << rDevConfig[ii].frameName << std::endl;
			std::cout << "\t Type = \t" << rDevConfig[ii].deviceType << std::endl;
			std::cout << "\t Num Rays = \t" << rDevConfig[ii].nRays << std::endl;
			std::cout << "\t Aperture = \t" << rDevConfig[ii].aperture << std::endl;
			std::cout << "\t Min Range = \t" << rDevConfig[ii].rangeMin << std::endl;
			std::cout << "\t Max Range = \t" << rDevConfig[ii].rangeMax << std::endl;
		}
	}
	std::cout << "*******************************************************************************" << std::endl;
}

void Localization3dAlgNode::mainNodeThread(void)
{
	double qReal, qi, qj, qk; //aux variables to fill messages
	unsigned int ii;
	Cparticle3d *particlePtr;
	tf::Quaternion quat;
	
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;	
	//locks config mutex during an iteration
	config_mutex.enter();
	
	//increments iteration index
	pFilter->incrementIterationId();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;
	
	//propagation
	this->platformOdometry_mutex_.enter();
 	//odometry.printObservation();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
	pFilter->propagatePset(odometry);
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
	//priorTime = ros::Time::now();
	priorTime = odoTime;
	this->platformOdometry_mutex_.exit(); 
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
	
	//sets estimate time stamp just after propagation
	locEstimate.setTimeStamp();
	//this->PoseWithCovarianceStamped_msg_.header.stamp = ros::Time::now();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		

	//3-axis compass (compass + inclinometer) correction
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
	this->platformData_mutex_.enter();
 	pFilter->correctPset(inclinometers);
	this->platformData_mutex_.exit(); 
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
	
	//laser correction
	for (ii=0; ii<MAX_RANGE_DEVICES; ii++)
	{	
		if(rDevConfig[ii].frameName != "" )//device model has been allocated
		{
			this->laser_mutex[ii].enter();
 			pFilter->correctPset(laserObs[ii], ii);
			this->laser_mutex[ii].exit(); 
		}
	}

	//normalize particle set
	pFilter->normalizePset();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		

	//computes Info gain for this iteration
	infoGain += pFilter->computeKLDivergence();
// std::cout << "IG = " << infoGain << std::endl;

	//set estimate
	pFilter->setEstimate(locEstimate);
	//locEstimate.position.printPosition();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;			

	//resampling
	pFilter->resamplingPset((unsigned int)resamplingStyle);
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
	
	//unlocks config mutex at the end of the filter iteration
	config_mutex.exit();

	
	// [fill msg structures]
	
	//fill particle cloud
	this->PoseArray_msg_.header.seq = pFilter->getIterationId();
	//this->PoseArray_msg_.header.stamp.sec = locEstimate.getTimeStampSeconds();
	//this->PoseArray_msg_.header.stamp.nsec = locEstimate.getTimeStampNanoSeconds();
	this->PoseArray_msg_.header.stamp = priorTime;
	this->PoseArray_msg_.header.frame_id = mapFrame;
	for(ii=0; ii<pFilter->getNumParticles(); ii++)
	{
		particlePtr = pFilter->getParticle(ii);
		PoseArray_msg_.poses.resize(ii+1);
		PoseArray_msg_.poses.at(ii).position.x = particlePtr->getX();
		PoseArray_msg_.poses.at(ii).position.y = particlePtr->getY();
		PoseArray_msg_.poses.at(ii).position.z = particlePtr->getZ()+0.1;//to avoid occlusion with model ground surface
// 		particlePtr->getQuaternion(qReal, qi, qj, qk);
// 		PoseArray_msg_.poses.at(ii).orientation.x = qi;
// 		PoseArray_msg_.poses.at(ii).orientation.y = qj;
// 		PoseArray_msg_.poses.at(ii).orientation.z = qk;
// 		PoseArray_msg_.poses.at(ii).orientation.w = qReal;
		quat = tf::createQuaternionFromRPY( locEstimate.position.getR(),locEstimate.position.getP(),locEstimate.position.getH() );
		PoseArray_msg_.poses.at(ii).orientation.x = quat.getX();
		PoseArray_msg_.poses.at(ii).orientation.y = quat.getY();
		PoseArray_msg_.poses.at(ii).orientation.z = quat.getZ();
		PoseArray_msg_.poses.at(ii).orientation.w = quat.getW();	
	}
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;

	//fill localization pose estimate
	this->PoseWithCovarianceStamped_msg_.header.seq = pFilter->getIterationId();
	//this->PoseWithCovarianceStamped_msg_.header.stamp.sec = locEstimate.getTimeStampSeconds();
	//this->PoseWithCovarianceStamped_msg_.header.stamp.nsec = locEstimate.getTimeStampNanoSeconds();
	this->PoseWithCovarianceStamped_msg_.header.stamp = priorTime;
	this->PoseWithCovarianceStamped_msg_.header.frame_id = mapFrame;
	this->PoseWithCovarianceStamped_msg_.pose.pose.position.x = locEstimate.position.getX();
	this->PoseWithCovarianceStamped_msg_.pose.pose.position.y = locEstimate.position.getY();
	this->PoseWithCovarianceStamped_msg_.pose.pose.position.z = locEstimate.position.getZ();
	
	locEstimate.position.getQuaternion(qReal, qi, qj, qk);
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.x = qi;
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.y = qj;
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.z = qk;
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.w = qReal;

// 	quat = tf::createQuaternionFromRPY( locEstimate.position.getR(),locEstimate.position.getP(),locEstimate.position.getH() );
// 	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.x = quat.getX();
// 	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.y = quat.getY();
// 	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.z = quat.getZ();
// 	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.w = quat.getW();
	
	this->PoseWithCovarianceStamped_msg_.pose.covariance[0] = locEstimate.cxx;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[1] = locEstimate.cxy;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[6] = locEstimate.cxy;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[7] = locEstimate.cyy;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[14] = locEstimate.czz;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[21] = locEstimate.chh;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[28] = locEstimate.cpp;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[35] = locEstimate.crr;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[33] = locEstimate.position.getP();
	this->PoseWithCovarianceStamped_msg_.pose.covariance[34] = locEstimate.position.getR();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;	
	
	// [fill srv structure and make request to the server]
	
	// [fill action structure and make request to the action server]

	// [publish messages]
	this->particleSet_publisher_.publish(this->PoseArray_msg_);
	this->position_publisher_.publish(this->PoseWithCovarianceStamped_msg_);
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;	
	
	//Publish provided tf Transforms: odom wrt map
	tf::Pose mapToBase;
	tf::poseMsgToTF(this->PoseWithCovarianceStamped_msg_.pose.pose, mapToBase);
	tf::Stamped<tf::Pose> baseToMap(mapToBase.inverse(),ros::Time(0),baseFrame);
	tf::Stamped<tf::Pose> odomToMap;
	this->tfListener.transformPose(odomFrame, baseToMap, odomToMap);
	this->tfBroadcaster.sendTransform(tf::StampedTransform(odomToMap.inverse(),priorTime+transform_tolerance_,mapFrame,odomFrame));

// std::cout << "mainNodeThread(): END OF ITERATION" << std::endl << std::endl;	

}

/*  [subscriber callbacks] */
void Localization3dAlgNode::platformData_callback(const iri_segway_rmp_msgs::SegwayRMP200Status::ConstPtr& msg) 
{ 
// 	ROS_INFO("Localization3dAlgNode::platformData_callback: New Message Received"); 
	
	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->platformData_mutex_.enter(); 
	

	//std::cout << msg->data << std::endl; 
	inclinometers.markAsNew();
	inclinometers.setTimeStamp();//to do: it would be better to get ts from the message
	//inclinometers.setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec);
	inclinometers.pitch = msg->pitch_angle;
	inclinometers.roll = msg->roll_angle;
	inclinometers.setMagneticDistortion(); //absolute heading is not provided by segway, so we indicate it by alarming 
	inclinometers.markAsCorrect(); //to do: we should ckeck for correctness before mark the observation as correct
		
	//the 4 lines below should be at odometry callback, but they are here just for debugging because old bags did not provide rotation rates in the odometry message
// 	this->platformOdometry_mutex_.enter(); 
// 	odometry.accumDeltaP(dT*msg->pitch_rate); //accumulates pitch rotation
// 	odometry.accumDeltaR(dT*msg->roll_rate); //accumulates roll rotation
// 	this->platformOdometry_mutex_.exit(); 
	
	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->platformData_mutex_.exit(); 
	
	//ROS_INFO("Localization3dAlgNode::platformData_callback: New Message Received 2"); 
}
void Localization3dAlgNode::platformOdometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{ 
	double vx,vy,vz,vTrans;
	double tLast; //dT;
		
	//ROS_INFO("Localization3dAlgNode::platformOdometry_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->platformOdometry_mutex_.enter(); 

	//std::cout << msg->data << std::endl; 
	odometry.markAsNew();
	tLast = odometry.getTimeStamp();
	odometry.setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec); //get ts from the message
	//tLast = odometry.getTimeStamp(); //gets last time stamp
	//odometry.setTimeStamp(); //
	dT = odometry.getTimeStamp() - tLast; //computes elapsed time between consecutive readings
//std::cout << "dT = " << dT << std::endl;
	if (fabs(dT)>1)
	{
		std::cout << "WARNING! Odometry Integration time >1s !" << std::endl;
		dT = 0;
	}
	vx = msg->twist.twist.linear.x; 
	vy = msg->twist.twist.linear.y;
	vz = msg->twist.twist.linear.z;
	vTrans = sqrt(vx*vx+vy*vy+vz*vz);  //odometry observation considers only forward velocity
	odometry.accumDeltaTrans(dT*vTrans); //accumulates translational displacement
	odometry.accumDeltaH(dT*msg->twist.twist.angular.z); //accumulates heading rotation
	odometry.accumDeltaP(dT*msg->twist.twist.angular.y); //accumulates pitch rotation
	odometry.accumDeltaR(dT*msg->twist.twist.angular.x); //accumulates roll rotation
	odometry.markAsCorrect();//to do: we should ckeck for correctness before mark the observation as correct
	//tf::poseMsgToTF(msg->pose.pose, odoPose);//keeps odoPose for tf broadcaster
	odoTime = msg->header.stamp;
	//odoTime = ros::Time::now();

	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->platformOdometry_mutex_.exit(); 
	
	//ROS_INFO("Localization3dAlgNode::platformOdometry_callback: New Message Received 2"); 
}
void Localization3dAlgNode::laser0_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
	unsigned int ii;

	//ROS_INFO("Localization3dAlgNode::laser0_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->laser_mutex[0].enter(); 

	//std::cout << msg->data << std::endl; 
	laserObs[0].markAsNew();
	laserObs[0].setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec);//laserObs[1].setTimeStamp();
	laserObs[0].numPoints = msg->ranges.size();
	laserObs[0].aperture = fabs(msg->angle_max - msg->angle_min);
	laserObs[0].rmin = msg->range_min;
	laserObs[0].rmax = msg->range_max;
	laserObs[0].sigmaRange = rDevConfig[0].stdDev; //fixed as an user param, not in the message
	laserObs[0].ranges.clear(); //erase previous range data 
	laserObs[0].ranges.resize(laserObs[0].numPoints);
	if (msg->angle_min > msg->angle_max) //angles are provided clockwise from an overhead point of view
	{
// 		ROS_INFO("Localization3dAlgNode::laser0_callback: Clockwise"); 
		for(ii=0; ii<laserObs[0].numPoints; ii++) laserObs[0].ranges.at(ii) = msg->ranges.at(ii);
	}
	else //angles are provided counterclockwise from an overhead point of view
	{
// 		ROS_INFO("Localization3dAlgNode::laser0_callback: CounterClockwise"); 
		for(ii=0; ii<laserObs[0].numPoints; ii++) laserObs[0].ranges.at(ii) = msg->ranges.at(laserObs[0].numPoints-1-ii);
	}
	laserObs[0].markAsCorrect();//to do: we should ckeck for correctness before mark the observation as correct
	//laserObs[1].printObservation();//debug: check received data
	
	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->laser_mutex[0].exit(); 
	
	//ROS_INFO("Localization3dAlgNode::laser0_callback: New Message Received 2"); 
}
void Localization3dAlgNode::laser1_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
	unsigned int ii;

	//ROS_INFO("Localization3dAlgNode::laser1_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->laser_mutex[1].enter(); 

	//std::cout << msg->data << std::endl; 
	laserObs[1].markAsNew();
	laserObs[1].setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec);//laserObs[1].setTimeStamp();
	laserObs[1].numPoints = msg->ranges.size();
	laserObs[1].aperture = fabs(msg->angle_max - msg->angle_min);
	laserObs[1].rmin = msg->range_min;
	laserObs[1].rmax = msg->range_max;
	laserObs[1].sigmaRange = rDevConfig[1].stdDev; //fixed as an user param, not in the message
	laserObs[1].ranges.clear(); //erase previous range data 
	laserObs[1].ranges.resize(laserObs[1].numPoints);
	if (msg->angle_min > msg->angle_max) //angles are provided clockwise from an overhead point of view
	{
// 		ROS_INFO("Localization3dAlgNode::laser1_callback: Clockwise"); 
		for(ii=0; ii<laserObs[1].numPoints; ii++) laserObs[1].ranges.at(ii) = msg->ranges.at(ii);
	}
	else //angles are provided counterclockwise from an overhead point of view
	{
// 		ROS_INFO("Localization3dAlgNode::laser1_callback: CounterClockwise"); 
		for(ii=0; ii<laserObs[1].numPoints; ii++) laserObs[1].ranges.at(ii) = msg->ranges.at(laserObs[1].numPoints-1-ii);
 	}
	laserObs[1].markAsCorrect();//to do: we should ckeck for correctness before mark the observation as correct
	//laserObs[1].printObservation();//debug: check received data
	
	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->laser_mutex[1].exit(); 
	
	//ROS_INFO("Localization3dAlgNode::laser1_callback: New Message Received 2"); 	
}
void Localization3dAlgNode::laser2_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
	unsigned int ii;
	
	//ROS_INFO("Localization3dAlgNode::laser2_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->laser_mutex[2].enter(); 

	//std::cout << msg->data << std::endl; 
	laserObs[2].markAsNew();
	laserObs[2].setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec);//laserObs[1].setTimeStamp();
	laserObs[2].numPoints = msg->ranges.size();
	laserObs[2].aperture = fabs(msg->angle_max - msg->angle_min);
	laserObs[2].rmin = msg->range_min;
	laserObs[2].rmax = msg->range_max;
	laserObs[2].sigmaRange = rDevConfig[2].stdDev; //fixed as an user param, not in the message
	laserObs[2].ranges.clear(); //erase previous range data 
	laserObs[2].ranges.resize(laserObs[2].numPoints);
	if (msg->angle_min > msg->angle_max) //angles are provided clockwise from an overhead point of view
	{
// 		ROS_INFO("Localization3dAlgNode::laser2_callback: Clockwise"); 
		for(ii=0; ii<laserObs[2].numPoints; ii++) laserObs[2].ranges.at(ii) = msg->ranges.at(ii);
	}
	else //angles are provided counterclockwise from an overhead point of view
	{
// 		ROS_INFO("Localization3dAlgNode::laser2_callback: CounterClockwise"); 
		for(ii=0; ii<laserObs[2].numPoints; ii++) laserObs[2].ranges.at(ii) = msg->ranges.at(laserObs[2].numPoints-1-ii);
	}
	laserObs[2].markAsCorrect();//to do: we should ckeck for correctness before mark the observation as correct
	//laserObs[2].printObservation();//debug: check received data
	
	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->laser_mutex[2].exit(); 

	//ROS_INFO("Localization3dAlgNode::laser2_callback: New Message Received 2"); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */


void Localization3dAlgNode::node_config_update(Config &config, uint32_t level)
{
	config_mutex.enter();
	this->public_node_handle_.getParam("num_particles", numberOfParticles);
	pFilter->setNumParticles( (unsigned int)numberOfParticles );
	config_mutex.exit();
}

void Localization3dAlgNode::addNodeDiagnostics(void)
{
}

// void Localization3dAlgNode::setLaserObservation(const sensor_msgs::LaserScan::ConstPtr& msg, ClaserObservation & laserData)
// {
// 	laserData.
// }

/* main function */
int main(int argc,char *argv[])
{
	//initialize the filter (to do)
	
	//initialize glut (faramotics objects)
	glutInit(&argc, argv);

	//run the thread
	return algorithm_base::main<Localization3dAlgNode>(argc, argv, "localization3d_alg_node");
}
