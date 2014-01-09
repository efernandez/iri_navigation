#include "force_navigation_learning_alg.h"

ForceNavigationLearningAlgorithm::ForceNavigationLearningAlgorithm(void)
{
}

ForceNavigationLearningAlgorithm::~ForceNavigationLearningAlgorithm(void)
{
}

void ForceNavigationLearningAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// ForceNavigationLearningAlgorithm Public API