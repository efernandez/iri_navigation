#include "iri_oa_client_alg.h"

IriOaClientAlgorithm::IriOaClientAlgorithm()
{
}

IriOaClientAlgorithm::~IriOaClientAlgorithm()
{
}

void IriOaClientAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// IriOaClientAlgorithm Public API