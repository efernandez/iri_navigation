#include "laser_localisation_alg_node.h"

LaserLocalisationAlgNode::LaserLocalisationAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<LaserLocalisationAlgorithm>()
{
  ROS_INFO("Laser Localisation: Warming up..");
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->pose_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 100);

  // [init services]
  this->localise_server_ = this->public_node_handle_.advertiseService("localise", &LaserLocalisationAlgNode::localiseCallback, this);

  // [init clients]
  get_relative_pose_client_ = this->public_node_handle_.serviceClient<iri_laser_icp::GetRelativePose>("/iri_laser_icp/get_relative_pose");

  // [init action servers]

  // [init action clients]

  T_mapa_odom_.setOrigin( tf::Vector3(0, 0, 0.0) );
  T_mapa_odom_.setRotation( tf::createIdentityQuaternion() );

  tfb_.sendTransform(tf::StampedTransform(T_mapa_odom_, ros::Time::now(), alg_.config_.map_frame, alg_.config_.odom_frame));

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

  tfb_.sendTransform(tf::StampedTransform(T_mapa_odom_, ros::Time::now(), alg_.config_.map_frame, alg_.config_.odom_frame));

}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool LaserLocalisationAlgNode::localiseCallback(iri_laser_localisation::DoLocalisation::Request &req, iri_laser_localisation::DoLocalisation::Response &res)
{
  ROS_DEBUG("Laser Localisation: New Request");
  tf::Quaternion q;

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->localise_mutex_.enter();

//   if(this->alg_.isRunning())
//   {
  //ROS_INFO("LaserLocalisationAlgNode::localiseCallback: Processing New Request!");
  get_relative_pose_srv_.request.scan_ref  = req.scan_map;
  get_relative_pose_srv_.request.scan_sens = req.scan_sens;

  //do operations with req and output on res
  ROS_DEBUG("Laser Localisation: Sending New Request");
  if (get_relative_pose_client_.call(get_relative_pose_srv_))
  {
    // get transform from icp
    T_path_base_.setOrigin( tf::Vector3(
      get_relative_pose_srv_.response.pose_rel.pose.pose.position.x,
      get_relative_pose_srv_.response.pose_rel.pose.pose.position.y, 0.0) );

    tf::StampedTransform t;
    try{
      tfl_.lookupTransform(alg_.config_.map_frame, alg_.config_.odom_frame,ros::Time(0), t);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    /// test odom
//     geometry_msgs::Transform tm;
//     get_relative_pose_srv_.response.pose_rel.pose.pose.position.x+=tm.translation.x;
//     get_relative_pose_srv_.response.pose_rel.pose.pose.position.y+=tm.translation.y;
//     tf::Transform tp;
//     tf::transformTFToMsg(t,tm);
//     tf::Quaternion qp;
//     tf::quaternionMsgToTF(get_relative_pose_srv_.response.pose_rel.pose.pose.orientation,qp);
//     tf::Quaternion q = t.getRotation() * qp.inverse();
//     tf::quaternionTFToMsg(q,get_relative_pose_srv_.response.pose_rel.pose.pose.orientation);
//     this->pose_publisher_.publish(get_relative_pose_srv_.response.pose_rel);

    tf::quaternionMsgToTF (get_relative_pose_srv_.response.pose_rel.pose.pose.orientation, q);
    T_path_base_.setRotation(q);
    geometry_msgs::Transform tm;
    tf::transformTFToMsg(T_path_base_,tm);
    // update map to odom transform
    tf::transformTFToMsg(T_mapa_odom_,tm);
    ROS_INFO_XYR("T_mapa_odom_ prior",tm.translation.x,tm.translation.y,tm.rotation);               //ROS_INFO("Laser Localisation: T_mapa_odom_ prior %f %f %f",tm.translation.x,tm.translation.y,tf::getYaw(tm.rotation));
    T_mapa_odom_.setOrigin(T_mapa_odom_.getOrigin()+T_path_base_.getOrigin());
    T_mapa_odom_.setRotation(T_mapa_odom_.getRotation()*T_path_base_.getRotation().inverse());
    tf::transformTFToMsg(T_mapa_odom_,tm);
    ROS_INFO_XYR("T_mapa_odom_ postr",tm.translation.x,tm.translation.y,tm.rotation);

    tf::poseTFToMsg(T_mapa_odom_,res.pose.pose.pose);

    //broadcast Tf
    ros::Duration tolerance(0.2);
    tfb_.sendTransform(tf::StampedTransform(T_mapa_odom_, ros::Time::now()+tolerance, alg_.config_.map_frame, alg_.config_.odom_frame));
    ROS_DEBUG("Laser Localisation: Tf Broadcasted");
    /// ------------------------------------- AMCL ---------------------------------
    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    //   ros::Time transform_expiration = (laser_scan->header.stamp +
    //                                     transform_tolerance_);
    //   tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
    //                                       transform_expiration,
    //                                       global_frame_id_, odom_frame_id_);
    //   this->tfb_->sendTransform(tmp_tf_stamped);
    //   sent_first_transform_ = true;
    /// ----------------------------------------------------------------------------

    //this->pose_publisher_.publish(this->PoseWithCovarianceStamped_msg_);
  }
  else
  {
    ROS_ERROR("Laser Localisation: Failed to Call Server on topic get_relative_pose ");
  }
//  }
//   else
//   {
//     ROS_INFO("LaserLocalisationAlgNode::localiseCallback: ERROR: alg is not on run mode yet.");
//   }

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

// =============================

void LaserLocalisationAlgNode::ROS_INFO_XYR(const std::string & str,const float & x,const float & y,const geometry_msgs::Quaternion & r)
{
  ROS_INFO("Laser Localisation: %s",str.c_str());
  ROS_INFO("\033[31mx:\033[0m%f \033[32my:\033[0m%f \033[34mth:\033[0m%f ",
           x,y,tf::getYaw(r));
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<LaserLocalisationAlgNode>(argc, argv, "laser_localisation_alg_node");
}
