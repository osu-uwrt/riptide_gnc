#ifndef TRANS_EKF_COMBINATOR
#define TRANS_EKF_COMBINATOR

#include "ros/ros.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "nortek_dvl/Dvl.h"
#include "auv_msgs/SixDoF.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include <algorithm>
#include "math.h"

namespace riptide_gnc
{
class TransEKFCombinator
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber depth_sub_, imu_sub_, dvl_sub_;
  ros::Publisher six_dof_pub_;
  auv_msgs::SixDoF six_dof_msg_;
  int cb_counter_; // Counts the number of callbacks entered within each spin

  void depthCB(const riptide_msgs::Depth::ConstPtr &depth);
  void imuCB(const sensor_msgs::Imu::ConstPtr &imu);
  void dvlCB(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &dvl);

public:
  TransEKFCombinator(ros::NodeHandle nh);
  int getCBCounter();
  void publishMsg();
};
} // namespace riptide_gnc

#endif
