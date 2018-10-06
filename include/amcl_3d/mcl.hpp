#pragma once
#include "amcl_3d/particle_filter_interface.hpp"
#include "amcl_3d/particle_filter.hpp"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <memory> 

namespace amcl_3d
{
class Amcl
{
  public:
    Amcl();
    ~Amcl(){};
    bool setKdMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map);
    bool measureIMU(const sensor_msgs::Imu::ConstPtr &input_imu_msg){
    }

  private:
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_map_ptr_;
    std::shared_ptr<ParticleFilter> pf_ptr_;
};
} // namespace amcl_3d