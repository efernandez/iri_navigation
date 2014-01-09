#include "leuze_speed_limit_alg_node.h"

LeuzeSpeedLimitAlgNode::LeuzeSpeedLimitAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<LeuzeSpeedLimitAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 0.2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  this->leuze_status_subscriber_ = this->public_node_handle_.subscribe("leuze_status", 10, &LeuzeSpeedLimitAlgNode::leuze_status_callback, this);
  
  // [init services]
  
  // [init clients]
  change_speed_client_ = this->public_node_handle_.serviceClient<iri_diff_local_planner::change_max_vel>("change_speed");
  change_zone_client_ = this->public_node_handle_.serviceClient<iri_common_drivers_msgs::change_wp_pair>("change_zone");
  
  // [init action servers]
  
  // [init action clients]

  this->error=false;
  this->warning=false;
  this->current_pair=(pairs_t)-1;
  this->new_pair=(pairs_t)-1;
  this->new_status=false;

  this->zone_speeds.resize(4);
}

LeuzeSpeedLimitAlgNode::~LeuzeSpeedLimitAlgNode(void)
{
  // [free dynamic memory]
}

void LeuzeSpeedLimitAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  this->alg_.lock();

  if(this->new_status)
  {
    if(this->error || this->warning)
    {
      switch(this->current_pair)
      {
        case LEUZE_FP1: if(this->error)
                        {
                          /* set the speed to 0 */
                        }
                        break;
        case LEUZE_FP2: /* change the current zone */
                        this->new_pair=LEUZE_FP1;
                        break;
        case LEUZE_FP3: /* change the current zone */
                        this->new_pair=LEUZE_FP2;
                        break;
        case LEUZE_FP4: /* change the current zone */
                        this->new_pair=LEUZE_FP3;
                        break;
      }
    }
    else
    {
      switch(this->current_pair)
      {
        case LEUZE_FP1: /* change the current zone */
                        this->new_pair=LEUZE_FP2; 
                        break;
        case LEUZE_FP2: /* change the current zone */
                        this->new_pair=LEUZE_FP3;
                        break;
        case LEUZE_FP3: /* change the current zone */
                        this->new_pair=LEUZE_FP4;
                        break;
        case LEUZE_FP4: /* do nothing */
                        break;
      }
    }
    if(this->new_pair!=this->current_pair)
    {
      /* change the maximum speed of the robot */
      /* change the working zone of the leuze laser */
      change_zone_srv_.request.new_pair = this->new_pair; 
      ROS_INFO("LeuzeSpeedLimitAlgNode:: Sending New Request with new pair %d",this->new_pair); 
      if (change_zone_client_.call(change_zone_srv_)) 
      { 
        // send the appropiate maximum speed 
        change_speed_srv_.request.new_max_vel = this->zone_speeds[(int)this->new_pair]; 
        //ROS_INFO("LeuzeSpeedLimitAlgNode:: Sending New Request!"); 
        if (change_speed_client_.call(change_speed_srv_)) 
        { 
          this->new_status=false;
        } 
        else 
        { 
          ROS_INFO("LeuzeSpeedLimitAlgNode:: Failed to Call Server on topic change_speed "); 
        }
      } 
      else 
      { 
        ROS_INFO("LeuzeSpeedLimitAlgNode:: Failed to Call Server on topic change_zone "); 
      }
    }
    else
      this->new_status=false;
  }

  this->alg_.unlock();
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void LeuzeSpeedLimitAlgNode::leuze_status_callback(const iri_common_drivers_msgs::leuze_status::ConstPtr& msg) 
{ 
  static bool first=true;
 
  //ROS_INFO("LeuzeSpeedLimitAlgNode::leuze_status_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  this->alg_.lock(); 
  //this->leuze_status_mutex_.enter(); 

  if(first)
  {
    first=false;
    this->new_pair=(pairs_t)msg->current_pair;
  }

  this->current_pair=(pairs_t)msg->current_pair;
  this->warning=msg->alarm1;
  this->error=msg->ossd1 | msg->ossd2;
  this->new_status=true;
  ROS_INFO("Current pair %d",this->current_pair);

  //unlock previously blocked shared variables 
  this->alg_.unlock(); 
  //this->leuze_status_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void LeuzeSpeedLimitAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->zone_speeds[0]=config.zone1_max_speed;
  this->zone_speeds[1]=config.zone1_max_speed;
  this->zone_speeds[2]=config.zone1_max_speed;
  this->zone_speeds[3]=config.zone1_max_speed;

  this->alg_.unlock();
}

void LeuzeSpeedLimitAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<LeuzeSpeedLimitAlgNode>(argc, argv, "leuze_speed_limit_alg_node");
}
