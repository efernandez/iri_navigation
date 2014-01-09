#include "trajectory_broadcaster_alg.h"

TrajectoryBroadcasterAlgorithm::TrajectoryBroadcasterAlgorithm(void)
{
}

TrajectoryBroadcasterAlgorithm::~TrajectoryBroadcasterAlgorithm(void)
{
}

void TrajectoryBroadcasterAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  this->unlock();
}

// TrajectoryBroadcasterAlgorithm Public API