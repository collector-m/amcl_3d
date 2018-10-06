#include "amcl_3d/mcl.hpp"
namespace amcl_3d
{
Amcl::Amcl() {}
bool Amcl::setKdMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map)
{
    kd_map_ptr_->setInputCloud(map);
return true;
}
} // namespace amcl_3d