#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <quadrotor_msgs/OutputData.h>
#include "slam_2d.h"
#include "laser_height_estimator.h"
#include <std_msgs/Empty.h>

#include <csm_utils/scan_utils.h>

// ROS Variables
ros::Publisher pubOdom;
ros::Publisher pubHeight;
ros::Publisher pubScan;
ros::Publisher pubSubmap;

// Scan data 
arma::mat                  scan;
sensor_msgs::LaserScan  scanOut;
ros::Time               tScan;
bool                    isScan   = false;
ros::Time               tHeight;
bool                    isHeight = false;
// Laser Height Estimator 
LaserHeightEstimator laserHeightEstimator;
// 2D SLAM 
SLAM2D SLAM2D;

laser_slam::ScanUtils scan_utils;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Height scan
  laserHeightEstimator.ProcessScan(*msg);
  tHeight = msg->header.stamp;

  // Horizontal scan
  scan = preprocess_scan(laserHeightEstimator.GetHeight(), *msg, SLAM2D.get_resolution(), laserHeightEstimator.GetImuOrientation(), scanOut);
  tScan = msg->header.stamp;
  if (scan.n_cols > 0)
    isScan = true;

  // CSM
  if(!SLAM2D.use_csm_)
    return;

  double laser_height, laser_height_cov;

  sensor_msgs::LaserScan scan_out = scan_utils.scan_filter(*msg, laser_height, laser_height_cov);
  if(laser_height_cov > 0.1)
    ROS_WARN_THROTTLE(1, "Laser height = %f, cov = %f", laser_height, laser_height_cov);

  sensor_msgs::PointCloud cloud = scan_utils.scan_to_cloud(scan_out);
  cloud.header = msg->header;

  Eigen::Matrix3d R;
  arma::colvec ypr = laserHeightEstimator.GetImuOrientation();
  R(0,0) = cos(ypr(1));
  R(0,1) = sin(ypr(2))*sin(ypr(1));
  R(0,2) = cos(ypr(2))*sin(ypr(1));
  R(1,0) = 0;
  R(1,1) = cos(ypr(2));
  R(1,2) = -sin(ypr(2));
  R(2,0) = -sin(ypr(1));
  R(2,1) = sin(ypr(2))*cos(ypr(1));
  R(2,2) = cos(ypr(2))*cos(ypr(1));


  sensor_msgs::PointCloud cloud2d = scan_utils.project_cloud(R, cloud);
  sensor_msgs::PointCloud curr_cloud = scan_utils.down_sample_cloud(cloud2d);
  SLAM2D.set_ldp(curr_cloud);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  laserHeightEstimator.setHeight(msg->pose.pose.position.z);
}

void quad_callback(const quadrotor_msgs::OutputData::ConstPtr& msg)
{
  laserHeightEstimator.ProcessQuad(*msg);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  laserHeightEstimator.ProcessImu(*msg);
}

void publish_pose(string frame_id)
{
  // Assemble 6-DOF pose
  arma::colvec pose2d = SLAM2D.get_pose();
  arma::mat    cov    = SLAM2D.get_covariance();
  arma::colvec pose(6);
  pose(0)  = pose2d(0);
  pose(1)  = pose2d(1);
  pose(2)  = laserHeightEstimator.GetHeight();
  pose(3)  = pose2d(2);
  pose(4)  = laserHeightEstimator.GetImuOrientation()(1);
  pose(5)  = laserHeightEstimator.GetImuOrientation()(2);
  arma::colvec q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  // Assemble ROS odom message
  nav_msgs::Odometry odom;
  odom.header.stamp            = tScan;
  odom.header.frame_id         = frame_id;
  odom.pose.pose.position.x    = pose(0);
  odom.pose.pose.position.y    = pose(1);
  odom.pose.pose.position.z    = pose(2);
  odom.pose.pose.orientation.w = q(0);
  odom.pose.pose.orientation.x = q(1);
  odom.pose.pose.orientation.y = q(2);
  odom.pose.pose.orientation.z = q(3);
  for (int j = 0; j < 2; j++)
    for (int i = 0; i < 2; i++)
      odom.pose.covariance[i+j*6] = cov(i,j);
  odom.pose.covariance[0+3*6] = cov(0,2);
  odom.pose.covariance[1+3*6] = cov(1,2);
  odom.pose.covariance[3+0*6] = cov(2,0);
  odom.pose.covariance[3+1*6] = cov(2,1);
  odom.pose.covariance[3+3*6] = cov(2,2);
  odom.pose.covariance[2+2*6] = 0.01*0.01;
  odom.pose.covariance[4+4*6] = 0.03*0.03;
  odom.pose.covariance[5+5*6] = 0.03*0.03;
  pubOdom.publish(odom);

  // Also publish scan, Encode pose information in intensities
  scanOut.header.frame_id = string("/llaser");
  scanOut.intensities.clear();
  scanOut.intensities.push_back(pose(0));                                // 0
  scanOut.intensities.push_back(pose(1));                                // 1
  scanOut.intensities.push_back(pose(2));                                // 2
  scanOut.intensities.push_back(pose(3));                                // 3
  scanOut.intensities.push_back(pose(4));                                // 4
  scanOut.intensities.push_back(pose(5));                                // 5
  scanOut.intensities.push_back(0);                                      // 6
  scanOut.intensities.push_back(laserHeightEstimator.GetFloor());        // 7
  scanOut.intensities.push_back(laserHeightEstimator.GetLaserHeight());  // 8
  scanOut.intensities.push_back(0);                                      // 9
  scanOut.header.stamp = tScan;
  pubScan.publish(scanOut);
 }

void publish_submap(string frame_id)
{
  nav_msgs::OccupancyGrid submap;
  MapInfo submapInfo = SLAM2D.get_submap(submap.data);
  if (submapInfo.update)
  {
    arma::colvec pose2d       = SLAM2D.get_pose();
    arma::colvec submapOrigin = arma::zeros<arma::colvec>(3);
    submapOrigin(0) = submapInfo.originX;
    submapOrigin(1) = submapInfo.originY;
    submapOrigin(2) = 0.0;
    arma::colvec submapOriginBody = pose_update_2d(pose_inverse_2d(pose2d), submapOrigin);
    submap.header.stamp            = tScan;
    submap.header.frame_id         = frame_id;
    submap.info.resolution         = submapInfo.resolution;
    submap.info.width              = submapInfo.mapX;
    submap.info.height             = submapInfo.mapY;
    submap.info.origin.position.x  = submapOriginBody(0);
    submap.info.origin.position.y  = submapOriginBody(1);
    submap.info.origin.position.z  = 0;
    submap.info.origin.orientation = tf::createQuaternionMsgFromYaw(submapOriginBody(2));
    submap.info.map_load_time      = tScan;
    pubSubmap.publish(submap);  
  }
}


void save_callback(const std_msgs::Empty::ConstPtr& msg)
{
  // Save state to parameter server
  ros::NodeHandle pnh("~");
  arma::colvec pose2d = SLAM2D.get_pose();
  pnh.setParam("x", pose2d(0));
  pnh.setParam("y", pose2d(1));
  pnh.setParam("yaw", pose2d(2));
  ROS_INFO("Saved (x, y, yaw)=(%.2f, %.2f, %.2f) to param server",
           pose2d(0), pose2d(1), pose2d(2));
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_2d");
  ros::NodeHandle n("~");
  double x, y, yaw;
  bool valid = true;
  valid &= n.getParam("x", x);
  valid &= n.getParam("y", y);
  valid &= n.getParam("yaw", yaw);
  if (!valid) {
    ROS_INFO("NOT loading x, y, theta");
  } else {
    SLAM2D.set_pose(x, y, yaw);
    ROS_INFO("Loaded (x, y, yaw)=(%.2f, %.2f, %.2f)", x, y, yaw);
  }
  ros::Subscriber save_sub = n.subscribe("save_callback", 1, save_callback);

  SLAM2D.init_scan_matcher(n);

  laser_slam::ScanUtils::ScanInfo scan_info;

  n.param("idx_width", scan_info.idx_width, 38);
  n.param("idx_middle", scan_info.idx_middle, 968);
  n.param("height_idx_low", scan_info.height_idx_low, 0);
  n.param("height_idx_up", scan_info.height_idx_up, 10);
  n.param("min_ang_idx", scan_info.min_ang_idx, 0);
  n.param("shift_yaw", scan_info.shift_yaw, M_PI/4);
  n.param("resolution", scan_info.res, 0.05);

  scan_utils.init_scan_utils(scan_info);


  SLAM2D.set_resolution(scan_info.res);
  SLAM2D.set_loc_map_ratio(4);
  // Input
  ros::Subscriber sub1 = n.subscribe("scan_in", 10, scan_callback, ros::TransportHints().udp());
  ros::Subscriber sub2 = n.subscribe("quad_in", 10, quad_callback, ros::TransportHints().udp());
  ros::Subscriber sub3 = n.subscribe("imu_in", 10, imu_callback, ros::TransportHints().udp());
  ros::Subscriber sub4 = n.subscribe("sim_odom_in", 10, odom_callback, ros::TransportHints().udp());
  // Output
  pubOdom   = n.advertise<nav_msgs::Odometry>(     "odom" ,  10);
  pubHeight = n.advertise<nav_msgs::Odometry>(     "height", 10);
  pubScan   = n.advertise<sensor_msgs::LaserScan>( "scan" , 100);
  pubSubmap = n.advertise<nav_msgs::OccupancyGrid>("dmap" ,  10);

  ros::Rate r(1000.0);
  while(n.ok())
  {
    ros::spinOnce();
    // Update SLAM
    if (isScan)
    {
      isScan = false;
      // Odom prediction, right now just IMU yaw
      static double prevYaw = laserHeightEstimator.GetImuOrientation()(0);
      static double currYaw = laserHeightEstimator.GetImuOrientation()(0);
      currYaw = laserHeightEstimator.GetImuOrientation()(0);
      arma::colvec diffOdom = arma::zeros<arma::colvec>(3);
      diffOdom(2) = currYaw - prevYaw;
      prevYaw = currYaw;
      // SLAM
      if (SLAM2D.update_slam(scan, diffOdom)) // SLAM success, publish 6-DOF pose
      {
        publish_pose(string("/map"));
        publish_submap(string("/map"));
      }
      else                                    // SLAM failed, publish null for pose odom
      {
        publish_pose(string("null"));
        publish_submap(string("null"));
      }
    }
    // Publish height
    if (isHeight)
    {
      isHeight = false;
      nav_msgs::Odometry height;
      height.header.frame_id = (laserHeightEstimator.IsLaserHeight())?string("height"):string("null");
      height.header.stamp    = tHeight;
      height.pose.pose.position.z    = laserHeightEstimator.GetHeight();
      height.twist.twist.linear.z    = laserHeightEstimator.GetLaserHeight();
      height.pose.covariance[2+2*6]  = 0.05*0.05;
      height.twist.covariance[2+2*6] = 0.05*0.05;
      arma::colvec q = R_to_quaternion(ypr_to_R(laserHeightEstimator.GetImuOrientation()));
      height.pose.pose.orientation.w = q(0);
      height.pose.pose.orientation.x = q(1);
      height.pose.pose.orientation.y = q(2);
      height.pose.pose.orientation.z = q(3);
      pubHeight.publish(height);
    }
    r.sleep();
  }

  return 0;
}


