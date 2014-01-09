#include "localization3d_alg.h"

Localization3dAlgorithm::Localization3dAlgorithm()
{
}

Localization3dAlgorithm::~Localization3dAlgorithm()
{
}

void Localization3dAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  // save the current configuration
  this->config_=new_cfg;
}

// Localization3dAlgorithm Public API