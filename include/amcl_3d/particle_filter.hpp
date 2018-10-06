#pragma once
#include "amcl_3d/particle_filter_interface.hpp"
namespace amcl_3d
{
class ParticleFilter : public ParticleFilterInterface
{
public:
  ParticleFilter();
  virtual ~ParticleFilter(){};
  bool init() override;
  bool resample() override;
  bool predict() override;
  bool measure() override;

private:
};
}