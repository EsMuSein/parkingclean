#include "datasave.h"



dataSave::dataSave():cmd_(new geometry_msgs::Twist())
{

}
int dataSave::getRobotSpeed( geometry_msgs::TwistPtr &speed){

  boost::unique_lock<boost::mutex> lock(mMutexRobotSpeed);
// boost::unique_lock<boost::recursive_mutex> lock(mMutexRobotSpeed);
   speed->linear.x = cmd_->linear.x;
   speed->linear.y = cmd_->linear.y;
   speed->linear.z = cmd_->linear.z;
   speed->angular.x = cmd_->angular.x;
   speed->angular.y = cmd_->angular.y;
   speed->angular.z = cmd_->angular.z;
  return 0;
}

int dataSave::setRobotSpeed(const geometry_msgs::TwistPtr &speed){

  boost::unique_lock<boost::mutex> lock(mMutexRobotSpeed);
    cmd_->linear.x = speed->linear.x;
    cmd_->linear.y = speed->linear.y;
    cmd_->linear.z = speed->linear.z;
    cmd_->angular.x = speed->angular.x;
    cmd_->angular.y = speed->angular.y;
    cmd_->angular.z = speed->angular.z;
  return 0;
}

int dataSave::getRobotPose( CPose2D &robotPose){

  boost::unique_lock<boost::mutex> lock(mMutexRobotPose);
    robotPose = mRobotPose;
  return 0;
}
int dataSave::setRobotPose(const CPose2D &robotPose){

  boost::unique_lock<boost::mutex> lock(mMutexRobotPose);
    mRobotPose = robotPose;
  return 0;
}

int dataSave::getRobotPoseMap( CPose2D &robotPoseMap){

  boost::unique_lock<boost::mutex> lock(mMutexRobotPose);
    robotPoseMap = mRobotPoseMap;
  return 0;
}
int dataSave::setRobotPoseMap(const CPose2D &robotPoseMap){

  boost::unique_lock<boost::mutex> lock(mMutexRobotPose);
    mRobotPoseMap = robotPoseMap;
  return 0;
}

int dataSave::getLaserData(std::vector<double> &vlaserData){

  boost::unique_lock<boost::mutex> lock(mMutexLaserData);
  vlaserData.clear();
  if(mLaserData.size()<1){
    return -1;
  }
  vlaserData.resize(mLaserData.size());
  std::copy(mLaserData.begin(),mLaserData.end(),vlaserData.begin());
  return 0;
}

int dataSave::setLaserData(const std::vector<double> &vlaserData){

  boost::unique_lock<boost::mutex> lock(mMutexLaserData);
  mLaserData.clear();
  mLaserData.resize(vlaserData.size());
  std::copy(vlaserData.begin(),vlaserData.end(),mLaserData.begin());
  return 0;
}

int dataSave::getThreeDirectDist(Struct::sDists &threeDirectDist){

  boost::unique_lock<boost::mutex> lock(mMutexThreeDirectDist);
  threeDirectDist.F = msThreeDirectDist.F;
  threeDirectDist.L = msThreeDirectDist.L;
  threeDirectDist.R = msThreeDirectDist.R;
  return 0;
}

int dataSave::setThreeDirectDist(const Struct::sDists &threeDirectDist){

  boost::unique_lock<boost::mutex> lock(mMutexThreeDirectDist);
  msThreeDirectDist.F = threeDirectDist.F;
  msThreeDirectDist.L = threeDirectDist.L;
  msThreeDirectDist.R = threeDirectDist.R;
  return 0;
}

int dataSave::getSystermMode(Enum::modes &modes){
  boost::unique_lock<boost::mutex> lock(mMutexSystermMode);
  modes = mModes;
  return 0;
}

int dataSave::setSystermMode(const Enum::modes &modes){

  boost::unique_lock<boost::mutex> lock(mMutexSystermMode);
  mModes = modes;
  return 0;
}

int dataSave::getRobotHandleControl(Enum::rControls &rControls){

  boost::unique_lock<boost::mutex> lock(mMutexRobotHandleControl);
  rControls = mRControls;
  return 0;
}

int dataSave::setRobotHandleControl(const Enum::rControls &rControls){

  boost::unique_lock<boost::mutex> lock(mMutexRobotHandleControl);
  mRControls = rControls;
  return 0;
}
int dataSave::getAllRobotPoseInMap(std::vector<cv::Point2d> &mVRobotPose){
  boost::unique_lock<boost::mutex> lock(mMutexAllRobotPose);
//  std::cout<<"mvRobotPose.size(): "<< mvRobotPose.size()<<std::endl;
  mVRobotPose.clear();
  for(int i=0;i<mvRobotPose.size();i++){
    mVRobotPose.push_back(coordinateFromRobotToMap(mvRobotPose[i]));
  }
  return 0;
}

int dataSave::getAllRobotPose(std::vector<cv::Point2d>& mVRobotPose){
  boost::unique_lock<boost::mutex> lock(mMutexAllRobotPose);
  mVRobotPose.clear();
  mVRobotPose.resize(mvRobotPose.size());
  std::copy(mvRobotPose.begin(),mvRobotPose.end(),mVRobotPose.begin());;
  return 0;
}
int dataSave::clearAllRobotPose(){
  boost::unique_lock<boost::mutex> lock(mMutexAllRobotPose);
  mvRobotPose.clear();
  return 0;
}
int dataSave::setAllRobotPose(const cv::Point2d &MRobotPose){
  boost::unique_lock<boost::mutex> lock(mMutexAllRobotPose);
  mvRobotPose.push_back(MRobotPose);
  return 0;
}

int dataSave::getMotionRobotPoint(std::vector<cv::Point2d>& mVRobotPose){
  boost::unique_lock<boost::mutex> lock(mMutexMotionPoint);
//  std::cout<<"mvRobotPose.size():"<< mvRobotPose.size() <<std::endl;
  mVRobotPose.clear();
  mVRobotPose.resize(mVMotionPoint.size());
  std::copy(mVMotionPoint.begin(),mVMotionPoint.end(),mVRobotPose.begin());
//  std::cout<<"mVRobotPose.size():"<< mVRobotPose.size() <<std::endl;
  return 0;
}

int dataSave::setMotionRobotPoint(const std::vector<cv::Point2d>& MVRobotPose){
  boost::unique_lock<boost::mutex> lock(mMutexMotionPoint);
  mVMotionPoint.clear();
  mVMotionPoint.resize(MVRobotPose.size());
  std::copy(MVRobotPose.begin(),MVRobotPose.end(),mVMotionPoint.begin());
  return 0;
}


int dataSave::getGlobalMapMsg(cv::Mat &map){
  boost::unique_lock<boost::mutex> lock(mMutexGridMapMsg);

  if(mGlobalGridMap.empty()){
    std::cerr<<"get global gridmap error as globalmap is empty!"<<std::endl;
    return -1;
  }
  map = mGlobalGridMap.clone();
  return 0;
}

int dataSave::setGlobalMapMsg(const cv::Mat &map){
  boost::unique_lock<boost::mutex> lock(mMutexGridMapMsg);
  if(map.empty()){
    std::cerr<<"set global gridmap error as globalmap is empty!"<<std::endl;
    return -1;
  }
  map.copyTo(mGlobalGridMap);
  return 0;
}

int dataSave::getSubMapMsg(cv::Mat &map, CPose2D &subGridMapPose){
  boost::unique_lock<boost::mutex> lock(mmutexSubMapMsg);
  if(mGlobalGridMap.empty()){
    std::cerr<<"get subgridmap error as subgridmap is empty!"<<std::endl;
    return -1;
  }
  map = msubGridMap.clone();
  subGridMapPose = mRP_SubGridMap;
  return 0;
}

int dataSave::setSubMapMsg(const cv::Mat &map, const CPose2D &subGridMapPose){
  boost::unique_lock<boost::mutex> lock(mmutexSubMapMsg);
  if(map.empty()){
    std::cerr<<"set subgridmap error as subgridmap is empty!"<<std::endl;
    return -1;
  }
  msubGridMap = map.clone();
  mRP_SubGridMap = subGridMapPose;
  return 0;
}

int dataSave::getMotionRobotPose(std::vector<CPose2D> &vMotionRobotPose){
  boost::unique_lock<boost::mutex> lock(mMutexMotionRobotPose);
  vMotionRobotPose.clear();
  vMotionRobotPose.resize(mvMotionRobotPose.size());
  std::copy(mvMotionRobotPose.begin(),mvMotionRobotPose.end(),vMotionRobotPose.begin());
  return 0;
}
int dataSave::setMotionRobotPose(const std::vector<CPose2D> &vMotionRobotPose){
  boost::unique_lock<boost::mutex> lock(mMutexMotionRobotPose);
  mvMotionRobotPose.clear();
  mvMotionRobotPose.resize(vMotionRobotPose.size());
  std::copy(vMotionRobotPose.begin(),vMotionRobotPose.end(),mvMotionRobotPose.begin());
  return 0;
}

int dataSave::getMapOriginParm(cv::Point2d &MapOriginPram){

  boost::unique_lock<boost::mutex>lock(mMutexMapOriginPram);
  if(mMapOriginPram.x==0&&mMapOriginPram.y==0){
    std::cerr<<"mMapOriginPram is no parm!"<<std::endl;
    return -1;
  }
  MapOriginPram = mMapOriginPram;
  return 0;
}
int dataSave::setMapOriginParm(cv::Point2d &MapOriginPram){

  boost::unique_lock<boost::mutex>lock(mMutexMapOriginPram);
  mMapOriginPram = MapOriginPram;
  return 0;
}

int dataSave::getRobotIfArriveDestPose(bool &robotifArriveDestPose){
  boost::unique_lock<boost::mutex> lock(mMutexIsRobotArriveDestPose);
  robotifArriveDestPose = misRobotArriveDestPose;
  return 0;
}
int dataSave::setRobotIfArriveDestPose(const bool &robotifArriveDestPose){
  boost::unique_lock<boost::mutex> lock(mMutexIsRobotArriveDestPose);
  misRobotArriveDestPose = robotifArriveDestPose;
  return 0;
}

int dataSave::getNowMotionPose(CPose2D &NowMotionPose){
    boost::unique_lock<boost::mutex> lock(mMutexNowMotionPose);
    NowMotionPose = mNowMotionPose;
    return 0;
}

int dataSave::setNowMotionPose(const CPose2D &NowMotionPose){
    boost::unique_lock<boost::mutex> lock(mMutexNowMotionPose);
    mNowMotionPose = NowMotionPose;
    return 0;
}


double dataSave::coordinateFromMapToRobot(const double &Angle){
  double angle = - (Angle -90);
  if(angle>180) angle -=360;
  if(angle<=-180) angle +=360;
  return angle;
}
double dataSave::coordinateFromRobotToMap(const double &Angle){
 double angle = - (Angle -90);
 if(angle>180) angle -=360;
 if(angle<=-180) angle +=360;
 return angle;
}

CPose2D dataSave::coordinateFromRobotToMap(const CPose2D &RobotPose){
  CPose2D robotPose;
  cv::Point2d MapOriginPram;
  getMapOriginParm(MapOriginPram);
  robotPose.x(round(RobotPose.x()/mdResolution)+MapOriginPram.x);
  robotPose.y(round(-RobotPose.y()/mdResolution)+MapOriginPram.y);
  robotPose.phi_angle(coordinateFromRobotToMap(RobotPose.phi_angle()));
  return robotPose;
}

CPose2D dataSave::coordinateFromMapToRobot(const CPose2D &RobotPose){
  CPose2D robotPose;
  cv::Point2d MapOriginPram;
  getMapOriginParm(MapOriginPram);
  robotPose.x((RobotPose.x()-MapOriginPram.x)*mdResolution);
  robotPose.y(-(RobotPose.y()-MapOriginPram.y)*mdResolution);
  robotPose.phi_angle(coordinateFromRobotToMap(RobotPose.phi_angle()));
  return robotPose;
}

cv::Point2d dataSave::coordinateFromRobotToMap(const cv::Point2d &RobotPose){
  cv::Point2d robotPose;
  cv::Point2d MapOriginPram;
  getMapOriginParm(MapOriginPram);
  robotPose.x = round(RobotPose.x/mdResolution)+MapOriginPram.x;
  robotPose.y = round(-RobotPose.y/mdResolution)+MapOriginPram.y;
  return robotPose;
}

cv::Point2d dataSave::coordinateFromMapToRobot(const cv::Point2d &RobotPose){
  cv::Point2d robotPose;
  cv::Point2d MapOriginPram;
  getMapOriginParm(MapOriginPram);
  robotPose.x = (RobotPose.x-MapOriginPram.x)*mdResolution;
  robotPose.y = -(RobotPose.y-MapOriginPram.y)*mdResolution;
  return robotPose;
}
