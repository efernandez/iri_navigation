#include "laser_localisation_alg_node.h"

LaserLocalisationAlgNode::LaserLocalisationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<LaserLocalisationAlgorithm>(),
  received_ref_(false),
  received_sens_(false),
  received_pose_ref_(false)
{
  ROS_INFO("Laser Localisation: Warming up..");
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  pose_publisher_ =
    public_node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 100);

   // [init subscribers]
  scan_ref_subscriber_ = public_node_handle_.subscribe("scan_ref", 100,
                            &LaserLocalisationAlgNode::scan_ref_callback, this);
  scan_sens_subscriber_ = public_node_handle_.subscribe("scan_sens", 100,
                           &LaserLocalisationAlgNode::scan_sens_callback, this);
  pose_ref_subscriber_ = public_node_handle_.subscribe("pose_ref", 100,
                            &LaserLocalisationAlgNode::pose_ref_callback, this);

  // [init services]
  estimate_server_ = public_node_handle_.advertiseService("estimate",
                             &LaserLocalisationAlgNode::estimateCallback, this);
  localise_server_ = public_node_handle_.advertiseService("localise",
                             &LaserLocalisationAlgNode::localiseCallback, this);

  // [init clients]
  get_relative_pose_client_ =
    public_node_handle_.serviceClient<iri_laser_icp::GetRelativePose>("/iri_laser_icp/get_relative_pose");

  // [init action servers]

  // [init action clients]
  T_mapa_odom_.setOrigin( tf::Vector3(0, 0, 0.0) );
  T_mapa_odom_.setRotation( tf::createIdentityQuaternion() );

  //tfb_.sendTransform(tf::StampedTransform(T_mapa_odom_, ros::Time::now(), alg_.config_.map_frame,alg_.config_.odom_frame)); //"/map", "/odom")); //

  ROS_DEBUG("Laser Localisation: T map odom broadcasted");
}

LaserLocalisationAlgNode::~LaserLocalisationAlgNode(void)
{
  // [free dynamic memory]
}

void LaserLocalisationAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->PoseWithCovarianceStamped_msg_.data = my_var;

  if(received_sens_ && received_ref_ & received_pose_ref_ && alg_.config_.continuous)
  {
    ROS_INFO("mainnodethread");
    calc_T_mapa_odom_(scan_ref_,scan_sens_,pose_ref_);
    tfb_.sendTransform(tf::StampedTransform(T_mapa_odom_, ros::Time::now(), alg_.config_.map_frame, alg_.config_.odom_frame));
    received_sens_ = false;
  }else{
   tfb_.sendTransform(tf::StampedTransform(T_mapa_odom_, ros::Time::now(), alg_.config_.map_frame, alg_.config_.odom_frame));
  }
}

/*  [subscriber callbacks] */
void LaserLocalisationAlgNode::scan_ref_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("LaserNavigationAlgNode::scan_callback: New Message Received");
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->scan_ref_mutex_.enter();
  scan_ref_ = *msg;
  received_ref_ = true;
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->scan_ref_mutex_.exit();
}

void LaserLocalisationAlgNode::scan_sens_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("LaserNavigationAlgNode::scan_callback: New Message Received");
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->scan_sens_mutex_.enter();
  scan_sens_ = *msg;
  received_sens_ = true;
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->scan_sens_mutex_.exit();
}

void LaserLocalisationAlgNode::pose_ref_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("LaserNavigationAlgNode::scan_callback: New Message Received");
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->pose_ref_mutex_.enter();
  pose_ref_ = *msg;
  received_pose_ref_ = true;
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->pose_ref_mutex_.exit();
}

/*  [service callbacks] */
bool LaserLocalisationAlgNode::estimateCallback(iri_laser_localisation::DoEstimation::Request &req, iri_laser_localisation::DoEstimation::Response &res)
{
  ROS_INFO("LaserLocalisationAlgNode::estimateCallback: New Request Received!");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->estimate_mutex_.enter();

  T_mapa_odom_init_est_.setOrigin( tf::Vector3(req.pose_est.pose.position.x, req.pose_est.pose.position.y, req.pose_est.pose.position.z) );

  tf::Quaternion q;
  //ROS_INFO("QUATERNION MSG: %f %f %f %f",req.pose_est.pose.orientation.x, req.pose_est.pose.orientation.y, req.pose_est.pose.orientation.z, req.pose_est.pose.orientation.w);
  tf::quaternionMsgToTF (req.pose_est.pose.orientation, q);
  //ROS_ERROR("QUATERNION MSG: %f %f %f %f" req.pose_est.pose.orientation.x, req.pose_est.pose.orientation.y, req.pose_est.pose.orientation.z, req.pose_est.pose.orientation.w);
  //q.normalized();

  T_mapa_odom_init_est_.setRotation(q.normalized());

  T_mapa_odom_ = T_mapa_odom_init_est_;

  tfb_.sendTransform(tf::StampedTransform(T_mapa_odom_, ros::Time::now(), alg_.config_.map_frame, alg_.config_.odom_frame));

  ROS_INFO_XYR("T_mapa_odom_ ESTIMACIO INICIAL",T_mapa_odom_);

  res.ok = true;

  ROS_INFO("Laser Localisation: T map odom estimation broadcasted");

  //if(this->alg_.isRunning())
  //{
    //ROS_INFO("LaserLocalisationAlgNode::estimateCallback: Processin New Request!");
    //do operations with req and output on res
    //res.data2 = req.data1 + my_var;
  //}
  //else
  //{
    //ROS_INFO("LaserLocalisationAlgNode::estimateCallback: ERROR: alg is not on run mode yet.");
  //}

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->estimate_mutex_.exit();

  return true;
}

bool LaserLocalisationAlgNode::localiseCallback(iri_laser_localisation::DoLocalisation::Request &req, iri_laser_localisation::DoLocalisation::Response &res)
{
  ROS_DEBUG("Laser Localisation: New Request");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->localise_mutex_.enter();
  pose_ref_ = req.pose_ref;
  received_pose_ref_ = true;
  calc_T_mapa_odom_(req.scan_ref,req.scan_sens,pose_ref_);

  tf::poseTFToMsg(T_mapa_odom_,res.pose.pose.pose);

  //broadcast Tf
  ros::Duration tolerance(0.2);
  tfb_.sendTransform(tf::StampedTransform(T_mapa_odom_, ros::Time::now()+tolerance, alg_.config_.map_frame, alg_.config_.odom_frame));
  ROS_DEBUG("Laser Localisation: Tf Broadcasted");

    //this->pose_publisher_.publish(this->PoseWithCovarianceStamped_msg_);

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->localise_mutex_.exit();


  return true;
}

/*  [action callbacks] */

/*  [action requests] */

void LaserLocalisationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void LaserLocalisationAlgNode::addNodeDiagnostics(void)
{
}

/// ============================================================================

void LaserLocalisationAlgNode::calc_T_mapa_odom_(sensor_msgs::LaserScan const& scan_ref,
                                                sensor_msgs::LaserScan const& scan_sens,
                                                geometry_msgs::Pose const& pose_ref)
{

  get_relative_pose_srv_.request.scan_ref  = scan_ref;
  get_relative_pose_srv_.request.scan_sens = scan_sens;

  //do operations with req and output on res
  ROS_DEBUG("Laser Localisation: Sending New Request");
  if (get_relative_pose_client_.call(get_relative_pose_srv_))
  {

    /** T_path_base_ *//* Es calcula amb ICP
    */
    T_path_base_.setOrigin( tf::Vector3(
                  get_relative_pose_srv_.response.pose_rel.pose.pose.position.x,
                  get_relative_pose_srv_.response.pose_rel.pose.pose.position.y,
                  0.0) );
    tf::Quaternion path_base_q;
    tf::quaternionMsgToTF (get_relative_pose_srv_.response.pose_rel.pose.pose.orientation,
                           path_base_q);
    T_path_base_.setRotation(path_base_q.normalized());
    ROS_INFO_XYR("T_path_base_",T_path_base_);
    //ROS_INFO_XYR("T_path_base_ inverse",T_path_base_.inverse());
    //T_path_base_=T_path_base_.inverse();

    /** T_base_odom_ *//* Es mira via TF
    */
    tf::StampedTransform T_base_odom_stamped;
    try{
      tfl_.lookupTransform(alg_.config_.base_frame, alg_.config_.odom_frame,
                           ros::Time(0), T_base_odom_stamped);
      T_base_odom_ = T_base_odom_stamped;
      ROS_INFO_XYR("T_base_odom_",T_base_odom_);
      //ROS_INFO_XYR("T_base_odom_ inverse",T_base_odom_.inverse());
      //T_base_odom_=T_base_odom_.inverse();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    /** T_mapa_path_ *//* Ve donada per la pose del mapa
    */
    tf::poseMsgToTF(pose_ref, T_mapa_path_);
    ROS_INFO_XYR("T_mapa_path_",T_mapa_path_);
    //T_mapa_path_ = T_mapa_path_ * T_mapa_odom_init_est_;
    //ROS_INFO_XYR("T_mapa_path_ - init_est_",T_mapa_path_* T_mapa_odom_init_est_);

    /** T_mapa_odom_ *//* Càlcul final
    */
    T_mapa_odom_ = T_mapa_path_ * T_path_base_ * T_base_odom_;

    ROS_INFO_XYR("T_mapa_odom_",T_mapa_odom_);

  }
  else
  {
    ROS_ERROR("Laser Localisation: Failed to Call Server on topic get_relative_pose ");
  }
}

void LaserLocalisationAlgNode::ROS_INFO_XYR(const std::string & str,const tf::Transform & tftrans)
{
  geometry_msgs::Transform gtrans;
  tf::transformTFToMsg(tftrans,gtrans);
  ROS_INFO("Laser Localisation: %s",str.c_str());
  ROS_INFO("\033[31mx:\033[0m%f \033[32my:\033[0m%f \033[34mth:\033[0m%f ",
           gtrans.translation.x,gtrans.translation.y,tf::getYaw(gtrans.rotation));
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<LaserLocalisationAlgNode>(argc, argv, "laser_localisation_alg_node");
}
