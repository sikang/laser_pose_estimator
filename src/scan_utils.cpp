#include "scan_utils.h"

arma::mat preprocess_scan(double height, const sensor_msgs::LaserScan& scan, double resolution, arma::colvec ypr, sensor_msgs::LaserScan& lscan, sensor_msgs::PointCloud& rcloud)
{
  // Cut laser scan
  lscan = scan;
  lscan.angle_min = -2.2689; 
  lscan.angle_max =  1.5800;
  lscan.range_max =  30;
  lscan.ranges.clear();
  arma::colvec h(3);
  arma::mat R(3,3);
  h(0) = 0;
  h(1) = 0;
  h(2) = height;
  //Z1Y2X3 Yaw = 0
  R(0,0) = cos(ypr(1));
  R(0,1) = sin(ypr(2))*sin(ypr(1));
  R(0,2) = cos(ypr(2))*sin(ypr(1));
  R(1,0) = 0;
  R(1,1) = cos(ypr(2));
  R(1,2) = -sin(ypr(2));
  R(2,0) = -sin(ypr(1));
  R(2,1) = sin(ypr(2))*cos(ypr(1));
  R(2,2) = cos(ypr(2))*cos(ypr(1));


  int curr_idx = -round((scan.angle_min - lscan.angle_min) / scan.angle_increment);

  double curr_angle = scan.angle_min + curr_idx * scan.angle_increment;
  while (curr_angle <= lscan.angle_max)
  {
    if(scan.ranges[curr_idx] > 0 ){
      arma::colvec p(3), pw(3);
      p(0) = scan.ranges[curr_idx]*cos(curr_angle + M_PI/4);
      p(1) = scan.ranges[curr_idx]*sin(curr_angle + M_PI/4);
      p(2) = 0;
      pw = R*p + h;
      if( pw(2) > -0.01 ){
        geometry_msgs::Point32 point;
        point.x = p(0);//pw(0);
        point.y = p(1);//pw(1);
        point.z = p(2);//pw(2);
        rcloud.points.push_back(point);

        lscan.ranges.push_back((scan.ranges[curr_idx] < lscan.range_max)?scan.ranges[curr_idx]:lscan.range_max);
      }
      else
        lscan.ranges.push_back(lscan.range_max);

   }
    else
      lscan.ranges.push_back(lscan.range_max);
 
   //    lscan.ranges.push_back((scan.ranges[curr_idx] < lscan.range_max)?scan.ranges[curr_idx]:lscan.range_max);
    curr_idx++;
    curr_angle = scan.angle_min + curr_idx * scan.angle_increment;
  }
  // Transform 45 degree, to IMU frame
  lscan.angle_min += 45 * PI/180;
  lscan.angle_max += 45 * PI/180;
  // Downsample
  arma::mat lscan_mat = arma::zeros<arma::mat>(2,lscan.ranges.size());
  int cnt = 0;
  double prevx = NUM_INF;
  double prevy = NUM_INF;
  for (unsigned int k = 0; k < lscan.ranges.size(); k++)
  {
    if (lscan.ranges[k] < lscan.range_max && lscan.ranges[k] >= 0 && !isnan(lscan.ranges[k]) && !isinf(lscan.ranges[k]))
    {
      double theta = lscan.angle_min + k*lscan.angle_increment;
      double x = lscan.ranges[k] * cos(theta);
      double y = lscan.ranges[k] * sin(theta);
      double dist = (x-prevx)*(x-prevx) + (y-prevy)*(y-prevy); 
      if (dist > resolution*resolution)
      { 
        lscan_mat(0,cnt) = x;
        lscan_mat(1,cnt) = y;
        prevx = x;
        prevy = y;
        cnt++;
      }
      else
        lscan.ranges[k] = lscan.range_max;
    }
  }
  if (cnt > 0)
    lscan_mat = lscan_mat.cols(0,cnt-1);
  else
    lscan_mat = arma::zeros<arma::mat>(2, cnt);
  // Projection
  ypr(0) = 0;
  arma::mat Rpr = ypr_to_R(ypr);
  Rpr = Rpr.submat(0,0,1,1);
  arma::mat lscan_mat_proj = Rpr * lscan_mat;
  return lscan_mat_proj;
}

// For compute fisher matrix
struct scanData
{
  double r;
  double theta;
  double alpha;
  double beta;
  double x;
  double y;
};

arma::mat cov_fisher(const arma::mat& scan, bool& isCovValid)
{

  double minDist = 0.10;

  // Convert data format
  vector<scanData> data;
  data.clear();
  data.reserve(scan.n_cols);
  for (unsigned int k = 0; k < scan.n_cols; k++)
  {
    double x = scan(0,k);
    double y = scan(1,k);
    scanData _data;
    _data.x = x;
    _data.y = y;
    _data.r = hypot(x,y);
    _data.theta = atan2(y,x);
    _data.alpha = 100;
    _data.beta = 0;
    data.push_back(_data);    
  }
  //Subsample
  int i = 0;		
  while (i < (int)(data.size())-1)
  {
    double d = hypot(data[i].x-data[i+1].x, data[i].y-data[i+1].y);
    if (d >= minDist)
      i++;
    else
      data.erase(data.begin()+i+1);      
  }
  //Calculate normal orientation
  for (unsigned int k = 0; k < data.size(); k++)	
  {
    if (k > 0 && k < data.size() - 1)
    {
      double tx = 0;
      double ty = 0;
      bool invalid_flag = false;
      double d1 = hypot(data[k-1].x-data[k].x, data[k-1].y-data[k].y);
      double d2 = hypot(data[k].x-data[k+1].x, data[k].y-data[k+1].y);
      if (d1 <= minDist * 2)
      {
        tx += (data[k-1].x-data[k].x) / d1;
        ty += (data[k-1].y-data[k].y) / d1;
      }
      else
        invalid_flag = true;
      if (d2 <= minDist * 2)
      {
        tx += (data[k].x-data[k+1].x) / d2;
        ty += (data[k].y-data[k+1].y) / d2;
      }
      else
        invalid_flag = true;
      if (!invalid_flag)
      {
        double nx = -ty;
        double ny =  tx;
        data[k].alpha = atan2(ny, nx);
        data[k].beta = data[k].alpha - data[k].theta;
      }
    }
  }
  //Remove points that do not have normal orientation
  unsigned int j = 0;	
  while (j < data.size())
  {
    if (data[j].alpha != 100)
      j++;
    else
      data.erase(data.begin()+j);
  }
  // Compute FIM based on normal
  arma::mat FIM(3,3);
  FIM.zeros();
  arma::mat f(3,3);
  f.zeros();
  for (unsigned int k = 0; k < data.size(); k++)
  {
    f.zeros();
    double r = data[k].r;
    double c = cos(data[k].alpha);
    double s = sin(data[k].alpha);
    double b = cos(data[k].beta);
    double t = tan(data[k].beta);
    // Upper left 2*2 block
    f(0,0) = c*c/(b*b);
    f(1,0) = c*s/(b*b);
    f(0,1) = c*s/(b*b);
    f(1,1) = s*s/(b*b);
    // Two symmetric block
    f(0,2) = r*t*c/b;
    f(1,2) = r*t*s/b;
    f(2,0) = r*t*c/b;
    f(2,1) = r*t*s/b;
    // Lower right
    f(2,2) = (r*t)*(r*t);
    // Add
    FIM = FIM + f; 
  }
  // Ray-tracing range measurement covariance
  double sigma = 0.2;
  FIM = FIM / (sigma*sigma);	
  // Deal with singular/non-positive-definite case
  arma::colvec eigFIM = eig_sym(FIM);
  arma::mat C = arma::eye<arma::mat>(3,3); 
  static bool isCovValidPrev = false;
  if (eigFIM(0) > 10 && !isCovValidPrev)
    isCovValid = true;
  else if (eigFIM(0) < 10e-10 && isCovValidPrev)
    isCovValid = false;
  else
    isCovValid = isCovValidPrev;
  isCovValidPrev = isCovValid;
  if (isCovValid)
    C = inv(FIM);

  return C;
}

