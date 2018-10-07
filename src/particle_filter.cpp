#include "amcl_3d/particle_filter.hpp"

namespace amcl_3d
{
ParticleFilter::ParticleFilter(){};
bool ParticleFiltler::init(const Position &position, const Quat &quat, const PoseCovariance &covariance, const size_t particle_num)
{
    particles_ptr_.reset();
    for (size_t i = 0; i < particle_num; ++i)
    {

    }
    return true;
};
bool ParticleFiltler::resample(){};
bool ParticleFiltler::predict(std::function<bool()> func){};
bool ParticleFiltler::measure(std::function<bool()> func){};

} // namespace amcl_3d