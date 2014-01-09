#include "force_robot_companion_alg.h"

ForceRobotCompanionAlgorithm::ForceRobotCompanionAlgorithm(void)
{
}

ForceRobotCompanionAlgorithm::~ForceRobotCompanionAlgorithm(void)
{
}

void ForceRobotCompanionAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// ForceRobotCompanionAlgorithm Public API