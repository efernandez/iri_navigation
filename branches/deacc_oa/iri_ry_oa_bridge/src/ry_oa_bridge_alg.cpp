#include "ry_oa_bridge_alg.h"

RyOaBridgeAlgorithm::RyOaBridgeAlgorithm(void)
{
}

RyOaBridgeAlgorithm::~RyOaBridgeAlgorithm(void)
{
}

void RyOaBridgeAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// RyOaBridgeAlgorithm Public API