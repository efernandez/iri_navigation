#include "my_map_checker_alg.h"

MyMapCheckerAlgorithm::MyMapCheckerAlgorithm(void)
{
}

MyMapCheckerAlgorithm::~MyMapCheckerAlgorithm(void)
{
}

void MyMapCheckerAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// MyMapCheckerAlgorithm Public API