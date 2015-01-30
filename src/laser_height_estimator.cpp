#include "laser_height_estimator.h"

LaserHeightEstimator::LaserHeightEstimator()
{
  // Imu
  ypr = arma::zeros<arma::colvec>(3);
  // Laser Measurements
  prev_laser_h  = 0;
  curr_laser_h  = 0;
  laser_h_invalid_cnt = 0;
  laser_h_valid_flag = false;
  // Current Floor Level
  curr_floor = 0.0;
  // Pressure Altitude, in case laser scan fails constantly
  pressure_h  = 0;
  pressure_dh = 0;
  pressure_h_offset = 0;
  pressure_h_offset_set = false;
  // Floor level to be published, for create multi-floor map in SLAM
  cont_floor_cnt = 0;
  to_publish_floor = 0;
  to_publish_floor_hist.clear();
  // Kalman Filter
  kalman_init_flag = false;
  floor_init_flag  = false;
  // State
  X = arma::zeros<arma::colvec>(3);
  // Covariance
  Q = arma::zeros<arma::mat>(1,1);	
  R = arma::zeros<arma::mat>(1,1);
  P = arma::zeros<arma::mat>(3,3);		
  Q(0,0) = 2*2;			      
  R(0,0) = 0.02*0.02;			
  P(0,0) = 0.02;			    
  P(1,1) = 0.02;
  P(2,2) = 0.0001;
  // Flag
  init = false;
  isSim = false;
  global_z_ = 0;
}

LaserHeightEstimator::~LaserHeightEstimator() { }

bool   LaserHeightEstimator::Initialized()       { return init;                                   }
double LaserHeightEstimator::GetHeight()  
{ 
  if(!isSim) 
    return X(0);        
  else
    return global_z_;
}
double LaserHeightEstimator::GetVelocityZ()      { return X(1);                                   }
double LaserHeightEstimator::GetFloor()          { return to_publish_floor;                       }
double LaserHeightEstimator::GetLaserHeight()    { return curr_laser_h;                           }
bool   LaserHeightEstimator::IsLaserHeight()     { return laser_h_valid_flag;                     }
arma::colvec LaserHeightEstimator::GetImuOrientation() { return ypr;                                    }

void LaserHeightEstimator::setHeight(double z)
{
  isSim = true;
  global_z_ = z;
}

void LaserHeightEstimator::ProcessScan(sensor_msgs::LaserScan scan)
{
  if (!kalman_init_flag)
    return;
  // Get laser height scan
 // double height_angle_min = 1.9373;
 // double height_angle_max = 1.9896;
  int height_idx_min = 972; //(height_angle_min - scan.angle_min) / scan.angle_increment;
  int height_idx_max = 982; //(height_angle_max - scan.angle_min) / scan.angle_increment;


  arma::colvec height_array(height_idx_max - height_idx_min + 1);
  int    height_cnt = 0;
  for (unsigned int k = 0; k < height_array.n_elem; k++)
  {
    if (!isnan(scan.ranges[height_idx_min + k]) && !isinf(scan.ranges[height_idx_min + k]))
    {
      height_array(k) = scan.ranges[height_idx_min + k];
      height_cnt++;
    }
  }
  double height_scan = median(height_array) - 0.045;  //Minus the distance from scan to mirror
  if (height_cnt == height_idx_max - height_idx_min + 1 && height_scan > 0.05  && height_scan < 10.0)
    laser_h_valid_flag = true;
  else
    laser_h_valid_flag = false;
  // Kalman Filter, Measurement Update
  if (laser_h_valid_flag)
  {
    // Reset invalid counter
    laser_h_invalid_cnt  = 0;
    pressure_h_offset = 0;
    pressure_h_offset_set = false;
    //Transform height scan to horizontal frame
    arma::colvec prh = ypr;
    prh(0) = 0;
    arma::mat Rh = ypr_to_R(prh);
    arma::colvec h(3);
    h(0) = -0.02;
    h(1) = 0.07;
    h(2) = -(height_scan - 0.23);//-(height_scan - 0.128);
    arma::colvec hGlobal = Rh * h;
    curr_laser_h = -hGlobal(2);
    // Handle floor levels
    if (!floor_init_flag)
    {
      floor_init_flag = true;
      X(0) = curr_laser_h + curr_floor;
      to_publish_floor_hist.push_back(to_publish_floor);
    }
    if (fabs(X(0) - (curr_laser_h + curr_floor)) >= FLOOR_THR)
    {
      //ROS_WARN(" ----- Floor Level Changed ----- ");
      curr_floor = X(0) - curr_laser_h;
      cont_floor_cnt = 0;  
    }
    else
    {
      cont_floor_cnt++;
      if (cont_floor_cnt == CONT_FLOOR_CNT_THR && fabs(curr_floor-to_publish_floor) > DIFF_FLOOR_THR)
      {
        // Try combine floor levels
        double min_dist = NUM_INF;
        double min_idx  = NUM_INF;
        for (unsigned int k = 0; k < to_publish_floor_hist.size(); k++)
        {
          double dist = fabs(to_publish_floor_hist[k] - curr_floor);
          if (dist < min_dist)
          {
            min_dist = dist;
            min_idx = k;
          }
        }
        if (min_dist < COMBINE_FLOOR_THR)
        {
          to_publish_floor = to_publish_floor_hist[min_idx];
          curr_floor = to_publish_floor;
          X(0) = curr_laser_h + curr_floor;
        }
        else
        {
          to_publish_floor = curr_floor;
          to_publish_floor_hist.push_back(to_publish_floor);
        }
      }
    }
    X(2) = curr_floor;
    arma::mat C(1,3);
    C(0,0) = 1; C(0,1) = 0; C(0,2) = -1;
    arma::colvec K = P*trans(C) * inv(C*P*trans(C) + R);
    X = X + K * (curr_laser_h - C*X);
    P = P - K*C*P;
    curr_floor   = X(2); 
  }
  else
  {
    laser_h_invalid_cnt++;
  }
  if (laser_h_invalid_cnt > 15)
  {
//    ROS_ERROR(" --- No Laser Height Measurement, Use Pressure Altitude --- ");
    if (!pressure_h_offset_set)
    {
      pressure_h_offset_set = true;
      pressure_h_offset = X(0) - pressure_h;
    }
    X(0) = pressure_h + pressure_h_offset;
    X(1) = pressure_dh;
    X(2) = X(2);
  }
}
void LaserHeightEstimator::ProcessImu(sensor_msgs::Imu quad)
{
  // Get acceleration
  arma::colvec a(3);
  a(0) = quad.linear_acceleration.x;
  a(1) = quad.linear_acceleration.y;
  a(2) = quad.linear_acceleration.z;
  // Calibration, estimate gravity
  static int calLimit = 100;
  static int calCnt   = 0;
  static arma::colvec g = arma::zeros<arma::colvec>(3);
  if (calCnt < calLimit)       
  {
    calCnt++;
    g += a;
    return;
  }
  else if (calCnt == calLimit) 
  {
    calCnt++;
    g /= calLimit;
    g(0) = 0;
    g(1) = 0;
    return;
  }
  // Init process update time
  static ros::Time prev_update;
  static ros::Time curr_update;
  if (!kalman_init_flag)
  {
    kalman_init_flag = true;
    prev_update = quad.header.stamp;	
    curr_update = quad.header.stamp;
    return;
  }
  // Transform acceleration to global frame
  arma::colvec q(4);
  q(0) = quad.orientation.w; 
  q(1) = quad.orientation.x;
  q(2) = quad.orientation.y; 
  q(3) = quad.orientation.z;
  ypr = R_to_ypr(quaternion_to_R(q));
  arma::colvec prh = ypr;
  prh(0) = 0;
  arma::colvec ag = ypr_to_R(prh) * a - g;  // Substract gravity force
  double az = ag(2);	
  // Get update time interval
  curr_update = quad.header.stamp;
  double dt = (curr_update - prev_update).toSec();
  prev_update = curr_update;
  // Process Update
  arma::mat A = arma::eye<arma::mat>(3,3);
  A(0,1) = dt;
  arma::mat B(3,1);
  B(0,0) = dt*dt/2;
  B(1,0) = dt;
  B(2,0) = 0;
  X = A*X + B*az;
  P = A*P*trans(A) + B*Q*trans(B);
  init = true;
}
void LaserHeightEstimator::ProcessQuad(quadrotor_msgs::OutputData quad)
{
  // Get pressure 
  pressure_h  = quad.pressure_height;
  pressure_dh = quad.pressure_dheight;
  // Get acceleration
  arma::colvec a(3);
  a(0) = quad.linear_acceleration.x;
  a(1) = quad.linear_acceleration.y;
  a(2) = quad.linear_acceleration.z;
  // Calibration, estimate gravity
  static int calLimit = 100;
  static int calCnt   = 0;
  static arma::colvec g = arma::zeros<arma::colvec>(3);
  if (calCnt < calLimit)       
  {
    calCnt++;
    g += a;
    return;
  }
  else if (calCnt == calLimit) 
  {
    calCnt++;
    g /= calLimit;
    g(0) = 0;
    g(1) = 0;
    return;
  }
  // Init process update time
  static ros::Time prev_update;
  static ros::Time curr_update;
  if (!kalman_init_flag)
  {
    kalman_init_flag = true;
    prev_update = quad.header.stamp;	
    curr_update = quad.header.stamp;
    return;
  }
  // Transform acceleration to global frame
  arma::colvec q(4);
  q(0) = quad.orientation.w; 
  q(1) = quad.orientation.x;
  q(2) = quad.orientation.y; 
  q(3) = quad.orientation.z;
  ypr = R_to_ypr(quaternion_to_R(q));
  arma::colvec prh = ypr;
  prh(0) = 0;
  arma::colvec ag = ypr_to_R(prh) * a - g;  // Substract gravity force
  double az = ag(2);	
  // Get update time interval
  curr_update = quad.header.stamp;
  double dt = (curr_update - prev_update).toSec();
  prev_update = curr_update;
  // Process Update
  arma::mat A = arma::eye<arma::mat>(3,3);
  A(0,1) = dt;
  arma::mat B(3,1);
  B(0,0) = dt*dt/2;
  B(1,0) = dt;
  B(2,0) = 0;
  X = A*X + B*az;
  P = A*P*trans(A) + B*Q*trans(B);
  init = true;
}
