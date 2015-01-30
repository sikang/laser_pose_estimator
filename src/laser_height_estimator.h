#ifndef LASER_HEIGHT_ESTIMATION_H
#define LASER_HEIGHT_ESTIMATION_H

#include <iostream>
#include <ros/ros.h>
#include <armadillo>
#include <sensor_msgs/LaserScan.h>
#include <quadrotor_msgs/OutputData.h>
#include <sensor_msgs/Imu.h>
#include "pose_utils.h"

#define CONT_FLOOR_CNT_THR 60
#define FLOOR_THR          0.1
#define DIFF_FLOOR_THR     1.0
#define COMBINE_FLOOR_THR  0.6

class LaserHeightEstimator
{
  private:

    // Imu
    arma::colvec ypr;
    // Laser Measurements
    double prev_laser_h;
    double curr_laser_h;
    int  laser_h_invalid_cnt;
    bool laser_h_valid_flag;
    // Current Floor Level
    double curr_floor;
    // Pressure Altitude, in case laser scan fails constantly
    double pressure_h;
    double pressure_dh;
    double pressure_h_offset;
    bool   pressure_h_offset_set;
    // Floor level to be published, for create multi-floor map in SLAM
    int cont_floor_cnt;
    double to_publish_floor;
    vector<double> to_publish_floor_hist;
    // Kalman Filter
    bool kalman_init_flag;
    bool floor_init_flag;
    // State
    arma::colvec X;
    // Covariance
    arma::mat Q;		// Process noise, from IMU
    arma::mat R;		// Measurement noise, from Laser
    arma::mat P;		// Overall covariance

    // Flag
    bool init;
    bool isSim;
    double global_z_;

  public:

    LaserHeightEstimator();
    ~LaserHeightEstimator();

    bool   Initialized();
    double GetHeight();
    double GetVelocityZ();
    double GetFloor();
    double GetLaserHeight();
    bool   IsLaserHeight();
    void   setHeight(double z);
    arma::colvec GetImuOrientation();

    void ProcessScan(sensor_msgs::LaserScan scan);
    void ProcessQuad(quadrotor_msgs::OutputData quad);
    void ProcessImu(sensor_msgs::Imu quad);
};

#endif
