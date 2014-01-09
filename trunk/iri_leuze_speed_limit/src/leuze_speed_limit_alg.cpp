#include "leuze_speed_limit_alg.h"

LeuzeSpeedLimitAlgorithm::LeuzeSpeedLimitAlgorithm(void)
{
}

LeuzeSpeedLimitAlgorithm::~LeuzeSpeedLimitAlgorithm(void)
{
}

void LeuzeSpeedLimitAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// LeuzeSpeedLimitAlgorithm Public API