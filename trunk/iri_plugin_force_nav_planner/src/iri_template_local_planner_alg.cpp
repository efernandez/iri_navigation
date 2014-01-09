#include "iri_template_local_planner_alg.h"

IriTemplateLocalPlannerAlgorithm::IriTemplateLocalPlannerAlgorithm(void)
{
}

IriTemplateLocalPlannerAlgorithm::~IriTemplateLocalPlannerAlgorithm(void)
{
}

void IriTemplateLocalPlannerAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  
  this->unlock();
}

// IriTemplateLocalPlannerAlgorithm Public API