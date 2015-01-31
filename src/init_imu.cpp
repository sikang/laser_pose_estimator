#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/OutputData.h>

ros::Publisher imu_pub;
ros::Publisher quad_pub;
static bool first_msg_ = true;
static double init_yaw_ = 0;

void quadCallback(const quadrotor_msgs::OutputData::ConstPtr& quad)
{
  if(first_msg_)
  {
    first_msg_ = false;
    init_yaw_ = tf::getYaw(quad->orientation);
    return;
  }

  double q0 = quad->orientation.w;
  double q1 = quad->orientation.x;
  double q2 = quad->orientation.y;
  double q3 = quad->orientation.z;
  double y = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
  double p = asin(2*(q0*q2 - q3*q1));
  double r = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));

  quadrotor_msgs::OutputData new_quad = *quad;
  new_quad.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y - init_yaw_);

  quad_pub.publish(new_quad);
}


void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  if(first_msg_)
  {
    first_msg_ = false;
    init_yaw_ = tf::getYaw(imu->orientation);
    return;
  }

  double q0 = imu->orientation.w;
  double q1 = imu->orientation.x;
  double q2 = imu->orientation.y;
  double q3 = imu->orientation.z;
  double y = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
  double p = asin(2*(q0*q2 - q3*q1));
  double r = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));

  sensor_msgs::Imu new_imu = *imu;
  new_imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y - init_yaw_);

  imu_pub.publish(new_imu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "init_imu");
  ros::NodeHandle n("~");

  ros::Subscriber sub1 = n.subscribe("imu_in", 10, imuCallback);
  ros::Subscriber sub2 = n.subscribe("quad_in", 10, quadCallback);
  imu_pub = n.advertise<sensor_msgs::Imu>("imu_out", 10);
  quad_pub = n.advertise<quadrotor_msgs::OutputData>("quad_out", 10);

  ros::spin();

  return 0;

}
