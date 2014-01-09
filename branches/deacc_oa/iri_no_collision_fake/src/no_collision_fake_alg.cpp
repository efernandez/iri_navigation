#include "no_collision_fake_alg.h"

NoCollisionFakeAlgorithm::NoCollisionFakeAlgorithm(void)
{
}

NoCollisionFakeAlgorithm::~NoCollisionFakeAlgorithm(void)
{
}

void NoCollisionFakeAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// NoCollisionFakeAlgorithm Public API