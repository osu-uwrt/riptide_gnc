#ifndef TRANS_EKF_COMBINATOR
#define TRANS_EKF_COMBINATOR

#include "ros/ros.h"
#include <tf/tf.h>
#include "riptide_msgs/Depth.h"
#include "nortek_dvl/Dvl.h"
#include "auv_msgs/SixDoF.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include "math.h"
#include "auv_core/auv_core_headers.hpp"


namespace riptide_gnc
{
class TransEKFCombinator
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber depth_sub_, imu_sub_, dvl_sub_, odometrySub;
  ros::Publisher six_dof_pub_;
  auv_msgs::SixDoF six_dof_msg_;
  Eigen::Quaterniond quatNED_, quatENU_, quatBodyFixedENU2NED_;
  int cb_counter_; // Counts the number of callbacks entered within each spin
/*
  void depthCB(const riptide_msgs::Depth::ConstPtr &depth);
  
  void dvlCB(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &dvl);
*/
  void imuCB(const sensor_msgs::Imu::ConstPtr &imu);
  void odometryCB(const nav_msgs::Odometry::ConstPtr &odom);

public:
  TransEKFCombinator(ros::NodeHandle nh);
  int getCBCounter();
  void publishMsg();
};
} // namespace riptide_gnc

#endif
