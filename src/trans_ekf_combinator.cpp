#include "riptide_gnc/trans_ekf_combinator.hpp"

namespace riptide_gnc
{
TransEKFCombinator::TransEKFCombinator(ros::NodeHandle nh)
{
   nh_ = nh;

   std::string trans_ekf_sub_topic;
   nh_.param<std::string>("auv_gnc/trans_ekf/subscriber_topic", trans_ekf_sub_topic, std::string("/puddles/auv_gnc/trans_ekf/input_data"));

   depth_sub_ = nh_.subscribe<riptide_msgs::Depth>("/puddles/depth/raw", 1, &TransEKFCombinator::depthCB, this);
   imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/puddles/imu/data", 1, &TransEKFCombinator::imuCB, this);
   dvl_sub_ = nh_.subscribe<geometry_msgs::TwistWithCovarianceStamped>("/puddles/dvl_twist", 1, &TransEKFCombinator::dvlCB, this);

   six_dof_pub_ = nh_.advertise<auv_msgs::SixDoF>("/puddles/auv_gnc/trans_ekf/input_data", 1);
}

int TransEKFCombinator::getCBCounter()
{
   return cb_counter_;
}

void TransEKFCombinator::publishMsg()
{
   six_dof_pub_.publish(six_dof_msg_);
   cb_counter_ = 0;
}

void TransEKFCombinator::depthCB(const riptide_msgs::Depth::ConstPtr &depth)
{
 
   double time1 = six_dof_msg_.header.stamp.toSec(); // Current stamp
   double time2 = depth->header.stamp.toSec();       // Depth stamp
   if (time2 > time1)
      six_dof_msg_.header.stamp = depth->header.stamp;

   six_dof_msg_.pose.position.z = -(depth->depth);
   cb_counter_++;
}

void TransEKFCombinator::imuCB(const sensor_msgs::Imu::ConstPtr &imu)
{
   double time1 = six_dof_msg_.header.stamp.toSec(); // Current stamp
   double time2 = imu->header.stamp.toSec();         // IMU stamp
   if (time2 > time1)
      six_dof_msg_.header.stamp = imu->header.stamp;

   double x;
   double y;
   double z;
   
   tf2::Quaternion quat;

   tf2::convert(imu->orientation, quat);

   tf2::Matrix3x3(quat).getRPY(x,y,z);

   quat.setRPY(y,x,-z);

   six_dof_msg_.pose.orientation = tf2::toMsg(quat);

   //six_dof_msg_.pose.orientation.x = imu->orientation.x;
   //six_dof_msg_.pose.orientation.y = imu->orientation.y;
   //six_dof_msg_.pose.orientation.z = imu->orientation.z;
   //six_dof_msg_.pose.orientation.w = imu->orientation.w;

   six_dof_msg_.velocity.angular.x = imu->angular_velocity.y;
   six_dof_msg_.velocity.angular.y = imu->angular_velocity.x;
   six_dof_msg_.velocity.angular.z = -(imu->angular_velocity.z);

   six_dof_msg_.linear_accel.x = imu->linear_acceleration.y;
   six_dof_msg_.linear_accel.y = imu->linear_acceleration.x;
   six_dof_msg_.linear_accel.z = -(imu->linear_acceleration.z);
   cb_counter_++;
}

void TransEKFCombinator::dvlCB(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &dvl)
{
   double time1 = six_dof_msg_.header.stamp.toSec(); // Current stamp
   double time2 = dvl->header.stamp.toSec();         // DVL stamp
   if (time2 > time1)
      six_dof_msg_.header.stamp = dvl->header.stamp;

   if (!isnan(dvl->twist.twist.linear.x)) // There is no easy way to handle the DVL outputting nan
   {
      //ENU to NED
      six_dof_msg_.velocity.linear.x = dvl->twist.twist.linear.y;
      six_dof_msg_.velocity.linear.y = dvl->twist.twist.linear.x;
      six_dof_msg_.velocity.linear.z = -(dvl->twist.twist.linear.z);
   }
   cb_counter_++;


}
} // namespace riptide_gnc
