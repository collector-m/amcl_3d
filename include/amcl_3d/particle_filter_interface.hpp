#pragma once
#include <memory> 

namespace amcl_3d
{

struct Position
{
  double x;
  double y;
  double z;
};
struct Quat
{
  double x;
  double y;
  double z;
  double w;
};
struct State
{
  Position position;
  Quat quat;
};

class ParticleFilterInterface
{
public:
  virtual bool init() = 0;
  virtual bool resample() = 0;
  virtual bool predict() = 0;
  virtual bool measure() = 0;

protected:
  std::shared_ptr<State> particles_ptr_;
};

} // namespace amcl_3d