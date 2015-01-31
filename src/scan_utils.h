#ifndef SCAN_UTILS_H
#define SCAN_UTILS_H

#include <iostream>
#include <ros/ros.h>
#include <armadillo>
#include "pose_utils.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud.h>

// Cut the 'mirror' part of laser scan, downsample to meet resolution, project to common ground frame, also return the cutted and downsampled scan for republish
arma::mat preprocess_scan(double height, const sensor_msgs::LaserScan& scan, double resolution, arma::colvec ypr, sensor_msgs::LaserScan& lscan);

// Compute scan covariance based on fisher's imformation matrix
arma::mat cov_fisher(const arma::mat& scan, bool& isCovValid);

#endif
