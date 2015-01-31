#include "slam_2d.h"

/* ******************** Constructor ******************** */
SLAM2D::SLAM2D()
{
  // Parameters
  res = 0.05;
  loc_map_ratio = 5;
  // Status flags
  updateMapFlag = false;	
  initMapFlag = false;
  loc_map_cnt = 0;
  // Instant Sensor data and state
  currScan = arma::zeros<arma::mat>(2,100);			
  diffOdom = arma::zeros<arma::colvec>(3);	
  currPose = arma::zeros<arma::colvec>(3);
  // Covariance Matrix
  cov = arma::eye<arma::mat>(3,3);
  cov(0,0) = 2*2;
  cov(1,1) = 2*2;
  cov(2,2) = 45*PI/180*45*PI/180;
  // Map
  map = cvCreateImage(cvSize(100,100), IPL_DEPTH_8S, 1);
  map->origin = 0;
  cvSetZero(map);
  offsetMap = arma::zeros<arma::colvec>(2);
  submap = cvCreateImage(cvSize(100,100), IPL_DEPTH_8S, 1);
  submap->origin = 0;
  cvSetZero(submap);
  offsetSubmap = arma::zeros<arma::colvec>(2);
  first_match_ = true;
  
  cvNamedWindow("map window");
}

/* ******************** Destructor ******************** */
SLAM2D::~SLAM2D()
{
  cvReleaseImage(&map);
}


void SLAM2D::init_scan_matcher(ros::NodeHandle& n)
{
  n.param("use_csm", use_csm_, false);
  if(use_csm_)
  {
    ROS_WARN("CSM_ENABLED!");
    scan_matcher.initParams(n);
  }
}

void SLAM2D::set_ldp(const sensor_msgs::PointCloud& cloud)
{
  scan_matcher.pointCloudToLDP(cloud, curr_ldp);
}

/* ******************** Set Parameters ******************** */
void SLAM2D::set_resolution(double _res)
{
  res = _res;
}

void SLAM2D::set_pose(double x, double y, double yaw)
{
  currPose(0) = x;
  currPose(1) = y;
  currPose(2) = yaw;
}


void SLAM2D::set_loc_map_ratio(int _loc_map_ratio)
{
  loc_map_ratio = _loc_map_ratio;
}

/* ******************** SLAM Update ******************** */
bool SLAM2D::update_slam(const arma::mat& scan, const arma::colvec& odom)
{
  // Update sensor data
  currScan = scan;
  diffOdom = odom;
  if (!(loc_map_cnt % loc_map_ratio))
    updateMapFlag = true;
  else
    updateMapFlag = false;
  loc_map_cnt++;

  // Prediction
  prediction();

  bool isLocalizationValid = false;
  // Localization
  if (initMapFlag)
  {
    isLocalizationValid = localization();
  }

  // Mapping
  if (updateMapFlag)
  {
    mapping();
    initMapFlag = true;
  }

  return isLocalizationValid;
}

/* ********************* Return Resolution ********************* */
double SLAM2D::get_resolution()
{
  return res;
}

/* ******************** Return Current Pose ******************** */
const arma::colvec SLAM2D::get_pose()
{
  return currPose;
}

/* ************* Return Current Covariance Matrix ************** */
const arma::mat SLAM2D::get_covariance()
{
  return cov;
}

/* ******************** Return Submap in ROS(?) Format ******************** */
const MapInfo SLAM2D::get_submap(vector<signed char>& probMap)
{
  MapInfo mapInfo;
  if (updateMapFlag)
  {
    probMap.resize(submap->width * submap->height);
    probMap.assign(submap->imageData, submap->imageData + submap->width * submap->height);
    mapInfo.mapX = submap->width;
    mapInfo.mapY = submap->height;
    mapInfo.originX = -offsetSubmap(0)*res;
    mapInfo.originY = -offsetSubmap(1)*res;
    mapInfo.originZ = 0;
    mapInfo.resolution = res;
    mapInfo.update = true;
  }
  else
    mapInfo.update = false;
  return mapInfo;
}

/* ******************** Prediction ******************** */
void SLAM2D::prediction()
{
  // Prediction
  arma::colvec x = arma::zeros<arma::colvec>(6);
  arma::colvec u = arma::zeros<arma::colvec>(6);

  prevPose = currPose;
  x(0) = currPose(0);
  x(1) = currPose(1);
  x(3) = currPose(2);
  u(0) = diffOdom(0);
  u(1) = diffOdom(1);
  u(3) = diffOdom(2); 
  x = pose_update(x, u);
  currPose(0) = x(0);
  currPose(1) = x(1);
  currPose(2) = x(3);
}

/* ******************** Localization ******************** */
bool SLAM2D::localization()
{
  // Compute likehood for each pose sample, use float to speed up?
  float bestP = 0;
  float bestX = 0;
  float bestY = 0;
  float bestA = 0;
  int scanCnt = currScan.n_cols;   
  for (float a = -1.0*PI/180; a <= 1.0*PI/180+0.1*PI/180; a += 0.4*PI/180)  // Small window due to gyro prediction
  {
    float pt = currPose(2) + a;
    float c = cos(pt);
    float s = sin(pt);	
    arma::mat R(2,2);
    R(0,0) =  c;
    R(0,1) = -s;
    R(1,0) =  s;
    R(1,1) =  c;  
    arma::mat scanR = R * currScan;
    for (float x = -0.1; x <= 0.1+0.025; x += 0.05)  // Maxi speed = 2m/s
    {
      for (float y = -0.1; y <= 0.1+0.025; y += 0.05)
      {
        float p = 0.0;
        float px = currPose(0) + x;
        float py = currPose(1) + y;
        for (int i = 0; i < scanCnt; i+=2)
        {
          // Transform scan to global coordinate
          float sx = scanR(0,i) + px;
          float sy = scanR(1,i) + py;
          // Transform scan to map coordinate
          int sxMap = floor(sx / res) + offsetMap(0);
          int syMap = floor(sy / res) + offsetMap(1);
          if ( !(sxMap < 0 || sxMap >= map->width || syMap < 0 || syMap >= map->height) && 
               *(signed char*)(map->imageData + (map->widthStep*syMap + sxMap)) > 0      )
            p++;
        }
        if (p > bestP)
        {
          bestP = p;
          bestX = x;
          bestY = y;
          bestA = a;
        }
      }
    }
  }
  currPose(0) += bestX;
  currPose(1) += bestY;
  currPose(2) += bestA;


  if(use_csm_){
    if(first_match_){
      first_match_ = false;
    }
    else{
      gtsam::Pose2 prev_pose(prevPose(0), prevPose(1), prevPose(2));
      gtsam::Pose2 curr_pose(currPose(0), currPose(1), currPose(2));

      gtsam::Pose2 predicted_pose = prev_pose.between(curr_pose);

      gtsam::Pose2 pose_diff;
      gtsam::Matrix noise_matrix;
      scan_matcher.processScan2D(curr_ldp, prev_ldp, predicted_pose, pose_diff, noise_matrix);
      curr_pose = prev_pose.compose(pose_diff);

      currPose(0) = curr_pose.x();
      currPose(1) = curr_pose.y();
      currPose(2) = curr_pose.theta();
    }
    prev_ldp = curr_ldp;
  }
  // Pick maximum likehood

  // Determine scans that hitting obstacle in the map
  vector<arma::colvec> scanObsVect;
  double c = cos(currPose(2));
  double s = sin(currPose(2));	
  arma::mat R(2,2);
  R(0,0) =  c;
  R(0,1) = -s;
  R(1,0) =  s;
  R(1,1) =  c;  
  arma::mat scanR = R * currScan;
  for (int i = 0; i < scanCnt; i++)
  {
    // Transform scan to global coordinate
    float sx = scanR(0,i) + currPose(0);
    float sy = scanR(1,i) + currPose(1);
    // Transform scan to map coordinate
    int sxMap = floor(sx / res) + offsetMap(0);
    int syMap = floor(sy / res) + offsetMap(1);
    if ( !(sxMap < 0 || sxMap >= map->width || syMap < 0 || syMap >= map->height) && 
         *(signed char*)(map->imageData + (map->widthStep*syMap + sxMap)) > 0      )
    {
      arma::colvec xy = scanR.col(i);
      scanObsVect.push_back(xy);
    }
  }
  arma::mat scanObs(2,scanObsVect.size());
  for (unsigned int i = 0; i < scanObsVect.size(); i++)
    scanObs.col(i) = scanObsVect[i];
  // Compute localization covariance
  bool isCovValid = false;
  cov = cov_fisher(scanObs, isCovValid);
  if (isCovValid)
    return true;
  else
    return false;
}

/* ******************** Mapping ******************** */
void SLAM2D::mapping()
{
  // Mapping parameters
  int expandStep = 200;
  int lOcc       = 2;
  int lFree      = -1;
  // Determine map boundary
  bool mapResetFlag = false;
  int mapX = map->width;
  int mapY = map->height;
  int offX = offsetMap(0);
  int offY = offsetMap(1);
  arma::mat R2D(2,2);
  double c = cos(currPose(2));
  double s = sin(currPose(2));
  R2D(0,0) = c; R2D(0,1) = -s;
  R2D(1,0) = s; R2D(1,1) =  c;
  arma::colvec P2D = currPose.rows(0,1);
  arma::mat scanTmp = R2D * currScan + P2D * arma::ones<arma::rowvec>(currScan.n_cols);
  scanTmp = join_rows(scanTmp,currPose.rows(0,1));
  int mapXprev = mapX;
  int mapYprev = mapY;
  int offXprev = offX;
  int offYprev = offY;
  arma::colvec offsetTmp(2);
  offsetTmp(0) = offX;
  offsetTmp(1) = offY;
  arma::mat scanGridTmp = scanTmp / res + offsetTmp * arma::ones<arma::rowvec>(scanTmp.n_cols);
  int minX = floor(min(scanGridTmp.row(0)));
  int minY = floor(min(scanGridTmp.row(1)));
  while(minX < 0)
  {
    offX += expandStep;
    mapX += expandStep;
    minX += expandStep;
    mapResetFlag = true;
  }
  while(minY < 0)
  {
    offY += expandStep;
    mapY += expandStep;
    minY += expandStep;
    mapResetFlag = true;
  }
  offsetTmp(0) = offX;
  offsetTmp(1) = offY;
  scanGridTmp = scanTmp / res + offsetTmp * arma::ones<arma::rowvec>(scanTmp.n_cols);
  int maxX = floor(max(scanGridTmp.row(0)));
  int maxY = floor(max(scanGridTmp.row(1)));
  while(maxX > mapX-1)
  {
    mapX += expandStep;
    mapResetFlag = true;
  }
  while(maxY > mapY-1)
  {
    mapY += expandStep;
    mapResetFlag = true;
  }
  // Resize current map
  if (mapResetFlag)
  {
    IplImage* mapPrev = cvCloneImage(map);
    cvReleaseImage(&map);
    map = cvCreateImage(cvSize(mapX,mapY), IPL_DEPTH_8S, 1);
    map->origin = 0;
    cvSetZero(map);
    int doffX = offX - offXprev;
    int doffY = offY - offYprev;
    cvSetImageROI(map, cvRect(doffX, doffY, mapXprev, mapYprev));
    cvCopy(mapPrev, map);
    cvResetImageROI(map);
    cvReleaseImage(&mapPrev);
  }
  offsetMap(0) = offX;
  offsetMap(1) = offY;
  // Create Occupancy Grid 
  arma::mat scanGrid  = scanTmp / res + offsetMap*arma::ones<arma::rowvec>(scanTmp.n_cols);
  arma::colvec center = P2D / res + offsetMap;
  int scanCnt = scanGrid.n_cols;  
  double centerX = floor(center(0));
  double centerY = floor(center(1));
  // Get Min and Max
  double subminX = floor(min(scanGrid.row(0)));
  double subminY = floor(min(scanGrid.row(1)));
  double submaxX = ceil(max(scanGrid.row(0)));
  double submaxY = ceil(max(scanGrid.row(1)));
  // Get offset and map size
  int suboffX = subminX;
  int suboffY = subminY;
  int submapX = submaxX-subminX+1;
  int submapY = submaxY-subminY+1;
  // Create Occupancy Grid Map
  cvReleaseImage(&submap);
  submap = cvCreateImage(cvSize(submapX,submapY), IPL_DEPTH_8S, 1);  
  submap->origin = 0;
  cvSetZero(submap);
  // Lines, represent free space
  CvPoint p1 = cvPoint(centerX-suboffX , centerY-suboffY);
  for (int k = 0; k < scanCnt-1; k++)
  {
    CvPoint p2 = cvPoint(floor(scanGrid(0,k))-suboffX , floor(scanGrid(1,k))-suboffY);    
    cvLine(submap, p1, p2, cvScalar(lFree,0,0,0));
  }
  // Scan endpoints
  for (int k = 0; k < scanCnt-1; k++)
  {
    cvSetReal2D(submap, floor(scanGrid(1,k))-suboffY, floor(scanGrid(0,k))-suboffX, lOcc);
  }
  // Make the padded elements zero
  for (int y = 0; y < submapY; y++)
    for (int d = submap->width; d < submap->widthStep; d++)
      submap->imageData[y*submap->widthStep+d] = 0;
  // Add into current map, REALLY UGLY...
  int mapStep = map->widthStep;
  int submapStep = submap->widthStep;
  for (int x = 0; x < submapX; x++)
  {
    for (int y = 0; y < submapY; y++)
    {   
      int m = (int)(map->imageData[(y+suboffY)*mapStep+(x+suboffX)]) + (int)(submap->imageData[y*submapStep+x]);
      if (m <= 127 && m > -5)
      //if (m <= 127 && m > -127)
        map->imageData[(y+suboffY)*mapStep+(x+suboffX)] += submap->imageData[y*submapStep+x];
    }
  }
  // OpenCV use 4 alignment... so change submapX accordingly
  submap->width = submap->widthStep;
  offsetSubmap(0) = offsetMap(0) - suboffX;
  offsetSubmap(1) = offsetMap(1) - suboffY;
 
 // cvShowImage("map window", map);
 // cvWaitKey(5);
}

								
