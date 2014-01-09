#include "trajectory_2_markers_alg.h"

Trajectory2MarkersAlgorithm::Trajectory2MarkersAlgorithm(void)
{
}

Trajectory2MarkersAlgorithm::~Trajectory2MarkersAlgorithm(void)
{
}

void Trajectory2MarkersAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  
  this->unlock(); 
}