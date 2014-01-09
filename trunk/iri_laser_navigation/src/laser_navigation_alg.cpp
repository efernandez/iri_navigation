#include "laser_navigation_alg.h"

LaserNavigationAlgorithm::LaserNavigationAlgorithm(void)
{
}

LaserNavigationAlgorithm::~LaserNavigationAlgorithm(void)
{
}

void LaserNavigationAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// LaserNavigationAlgorithm Public API