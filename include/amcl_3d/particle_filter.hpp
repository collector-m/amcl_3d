#pragma once
#include "amcl_3d/particle_filter_interface.hpp"
#include "amcl_3d/xorshift128.hpp"

namespace amcl_3d
{
class ParticleFilter : public ParticleFilterInterface
{
public:
  ParticleFilter();
  virtual ~ParticleFilter(){};
  bool init(const Position &position, const Quat &quat, const PoseCovariance &covariance, const size_t particle_num) override;
  bool resample() override;
  bool predict(std::function<bool()> func) override;
  bool measure(std::function<bool()> func) override;
private:
};
}