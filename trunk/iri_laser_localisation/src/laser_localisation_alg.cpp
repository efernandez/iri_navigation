#include "laser_localisation_alg.h"

LaserLocalisationAlgorithm::LaserLocalisationAlgorithm(void)
{
}

LaserLocalisationAlgorithm::~LaserLocalisationAlgorithm(void)
{
}

void LaserLocalisationAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// LaserLocalisationAlgorithm Public API