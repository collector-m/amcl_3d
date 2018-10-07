#pragma once
#include <memory>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Matrix6d PoseCovariance;
typedef Eigen::Matrix3d PosisitonCovariance;
typedef Eigen::Vector3d Position;
typedef Eigen::Vector4d Quat;

namespace amcl_3d
{
struct State
{
  Position position; // x,y,z
  Quat quat;     // x,y,z,w
};

class ParticleFilterInterface
{
public:
  enum class MeasurementType
  {
    IMU,
    ODOMETRY,
    PC2,
    VELOCITY,
    ACCEL,
    POSITION,
    QUATERNION
  };
  virtual bool init(const Position &position, const Quat &quat, const PoseCovariance &covariance, const size_t particle_num) = 0;
  virtual bool resample() = 0;
  virtual bool predict(std::function<bool()> func) = 0;
  virtual bool measure(std::function<bool()> func) = 0;

protected:
  std::shared_ptr<State> particles_ptr_;
};

} // namespace amcl_3d