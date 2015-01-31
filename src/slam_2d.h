#ifndef SLAM_2D_H
#define SLAM_2D_H
#include <ros/ros.h>
#include <iostream>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "scan_utils.h"
#include "pose_utils.h"
#include <csm_utils/canonical_scan.h>

struct MapInfo
{
  int mapX;
  int mapY;
  double originX;
  double originY;
  double originZ;
  double resolution;
  bool update;
};

class SLAM2D
{
  private:
    // Instant Sensor data and state
    arma::mat       currScan;
    arma::colvec    diffOdom;
    arma::colvec    prevPose;
    arma::colvec    currPose;
    // Current covariance matrix
    arma::mat cov;

    // Status flags
    bool updateMapFlag;
    bool initMapFlag;
    unsigned long long int loc_map_cnt;

    // Map
    IplImage* map;		    // Occupancy grid map in Log scale
    arma::colvec offsetMap;
    IplImage* submap;     // Incremental Map
    arma::colvec offsetSubmap;

    // Parameters
    double res;
    int loc_map_ratio;

    // Main sequence
    void prediction();
    bool localization();
    void mapping();

    laser_slam::CanonicalScan scan_matcher;
    LDP curr_ldp;
    LDP prev_ldp;
    bool first_match_;

  public:
    SLAM2D();
    ~SLAM2D();
    void init_scan_matcher(ros::NodeHandle& n);
    void set_ldp(const sensor_msgs::PointCloud& cloud);

    bool update_slam(const arma::mat& scan, const arma::colvec& odom);								        // Main update
    void set_resolution(double _res);
    void set_pose(double x, double y, double z);
    void set_loc_map_ratio(int _loc_map_ratio);
    double        get_resolution();
    const arma::colvec  get_pose();					                                        		// Get current pose in 6*1 vector format
    const arma::mat     get_covariance();					                                      // Get current covariance matrix (6*6)
    const MapInfo get_submap(vector<signed char>& probMap);                       // Return Submap in ROS(?) Format 
    bool use_csm_;
};

#endif

