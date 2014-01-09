#include "dynamics_objects_localization_alg.h"

DynamicsObjectsLocalizationAlgorithm::DynamicsObjectsLocalizationAlgorithm(void)
{
}

DynamicsObjectsLocalizationAlgorithm::~DynamicsObjectsLocalizationAlgorithm(void)
{
}

void DynamicsObjectsLocalizationAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// DynamicsObjectsLocalizationAlgorithm Public API
