#include "people_follower_client_alg.h"

#include <vector>

PeopleFollowerClientAlgorithm::PeopleFollowerClientAlgorithm(void)
{
}

PeopleFollowerClientAlgorithm::~PeopleFollowerClientAlgorithm(void)
{
}

void PeopleFollowerClientAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// PeopleFollowerClientAlgorithm Public API
bool PeopleFollowerClientAlgorithm::isSomeoneStanding(const iri_people_tracking::peopleTrackingArray::ConstPtr& msg, 
                                                      unsigned int & target_index)
{
  bool ret = false;

  ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: msg->peopleSet.size=%d", msg->peopleSet.size());

  this->lock();
  std::vector<TargetPeople> vCurrentTargets;

  //for all tracked persons
  for(unsigned int ii=0; ii<msg->peopleSet.size(); ii++)
  {
    unsigned int target_id = (unsigned int)abs(msg->peopleSet[ii].targetId);

    //compute current velocity
    float mod_vel = sqrt(msg->peopleSet[ii].vx*msg->peopleSet[ii].vx + 
                         msg->peopleSet[ii].vy*msg->peopleSet[ii].vy);

    //if current velocity is below threshold
    //person is stopped
    if( mod_vel < people_stand_max_vel_ )
    {
      //check if this person was already tracked
      bool found = false;
      unsigned int jj;
      for(jj=0; jj<vTargets_.size(); jj++)
      {
        if( target_id == vTargets_[jj].target_id )
        {
          ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: target_id=%d FOUND in jj=%d!", target_id, jj);
          found = true;
          break;
        }
      }
    
      ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: target_id=%d STANDING!", target_id);

      //if target was already tracked
      if(found)
        vCurrentTargets.push_back( TargetPeople(target_id, ++vTargets_[jj].stop_iter) );
      //if first time
      else
        vCurrentTargets.push_back( TargetPeople(target_id) );

      ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: [%d][%d] target_id=%d stop_iter=%d", ii, jj, target_id, vCurrentTargets[jj].stop_iter);
    }
    //person is moving
    else
      ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: targetId=%d MOVING", target_id);
  }

  //for all current tracked targets
  target_index = msg->peopleSet.size()+1;
  float min_dist = 1000000000.f;
  for(unsigned int ii=0; ii<vCurrentTargets.size(); ii++)
  {
    //if number of stopped iterations greater than threshold
    if( vCurrentTargets[ii].stop_iter > min_stopped_iters_)
    {
      float dist = sqrt(msg->peopleSet[ii].x*msg->peopleSet[ii].x + msg->peopleSet[ii].y*msg->peopleSet[ii].y);
      ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: \ttargetId=%d pose=(%f, %f) dist=%f", 
               msg->peopleSet[ii].targetId, msg->peopleSet[ii].x, msg->peopleSet[ii].y, dist);
      
      //check if it is the current minimum distance
      if( dist < min_dist )
      {
        target_index = ii;
        min_dist     = dist;
      }
    }
    else
      ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: \ttargetId=%d NOT stopped for enough time: %d", msg->peopleSet[ii].targetId, vCurrentTargets[ii].stop_iter);
  }

  //save current targets
  vTargets_ = vCurrentTargets;
  if(target_index != msg->peopleSet.size()+1)
  {
    ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: Final Candidate: target_id=%d", msg->peopleSet[target_index].targetId);
    ret = true;
  }
  else
    ROS_DEBUG("PeopleFollowerClientAlgorithm::isSomeoneStanding: NO Final Candidate");
  
  this->unlock();

  return ret;
}
