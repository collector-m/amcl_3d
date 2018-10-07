#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <memory>
#include "amcl_3d/mcl.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

namespace amcl_3d
{

class Amcl3dNode
{
public:
  Amcl3dNode();
  ~Amcl3dNode(){};

private: // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pf_pub_;
  ros::Subscriber pc2_map_sub_; // pointcloud2 map data
  ros::Subscriber odom_sub_;    // odometry
  ros::Subscriber imu_sub_;     // imu
  ros::Subscriber pc2_sub_;     // pointcloud2 measurement data
  ros::Timer timer_;            // publish timer
  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &input_map_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &input_odom_msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &input_imu_msg);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_init_pose_msg);
  void pc2Callback(const sensor_msgs::PointCloud2::ConstPtr &input_pc2_msg);
  void timerCallback(const ros::TimerEvent &e);

private:
  Amcl amcl_;
  std::string map_frame_id_;
};
Amcl3dNode::Amcl3dNode() : nh_(""), pnh_("~")
{
  pf_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particles", 1, true);
  pc2_map_sub_ = nh_.subscribe("map", 10, &Amcl3dNode::mapCallback, this);
  odom_sub_ = nh_.subscribe("odom", 100, &Amcl3dNode::odomCallback, this);
  imu_sub_ = nh_.subscribe("imu", 100, &Amcl3dNode::imuCallback, this);
  imu_sub_ = nh_.subscribe("initialpose", 100, &Amcl3dNode::initialPoseCallback, this);
  pc2_sub_ = nh_.subscribe("pc2", 1, &Amcl3dNode::pc2Callback, this);
  timer_ = nh_.createTimer(ros::Duration(0.01), &Amcl3dNode::timerCallback, this);
}
void Amcl3dNode::mapCallback(const sensor_msgs::PointCloud2::ConstPtr &input_map_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_map_msg, *pc_map);
  amcl_.setKdMap(pc_map);
}
void Amcl3dNode::odomCallback(const nav_msgs::Odometry::ConstPtr &input_odom_msg)
{
  //   static geometry_msgs::Pose test_pose;
  //   geometry_msgs::PoseArray output_msg;

  //   output_msg.header = input_odom_msg.header;
  // output_msg.poses.push_back(test_pose);
  // pf_pub_.publish(output_msg);
}
void Amcl3dNode::imuCallback(const sensor_msgs::Imu::ConstPtr &input_imu_msg) {}
void Amcl3dNode::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_init_pose_msg)
{
  const geometry_msgs::Point &ros_position = input_init_pose_msg->pose.pose.position;
  const geometry_msgs::Quaternion &ros_quat = input_init_pose_msg->pose.pose.orientation;
  Position position(ros_position.x, ros_position.y, ros_position.z);
  Quat quat(ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w);
  PoseCovariance covariance;
  if (!amcl_.setInitialPose(position, quat, covariance))
    ROS_ERROR("Failed set initial pose");
}
void Amcl3dNode::pc2Callback(const sensor_msgs::PointCloud2::ConstPtr &input_pc2_msg) {}
void Amcl3dNode::timerCallback(const ros::TimerEvent &e)
{
}

} // namespace amcl_3d

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "amcl_3d");

  amcl_3d::Amcl3dNode node();
  ros::spin();

  return 0;
}