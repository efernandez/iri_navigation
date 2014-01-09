#include "force_navigation_alg.h"

ForceNavigationAlgorithm::ForceNavigationAlgorithm(void)
{
}

ForceNavigationAlgorithm::~ForceNavigationAlgorithm(void)
{
}

void ForceNavigationAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// ForceNavigationAlgorithm Public API