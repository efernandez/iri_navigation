#include "force_robot_companion_learning_alg.h"

ForceRobotCompanionLearningAlgorithm::ForceRobotCompanionLearningAlgorithm(void)
{
}

ForceRobotCompanionLearningAlgorithm::~ForceRobotCompanionLearningAlgorithm(void)
{
}

void ForceRobotCompanionLearningAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// ForceRobotCompanionLearningAlgorithm Public API