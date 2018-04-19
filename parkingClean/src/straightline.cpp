#include "straightline.h"
#include <iostream>

StraightLine::StraightLine():mLaserResolutionParam(1)
{

}
StraightLine::StraightLine(double laserResolution):mLaserResolutionParam(laserResolution)
{

}

StraightLine::~StraightLine(){

}

  /******将激光数据转换为点云数据
   * @brief 将激光长度数据在当前坐标系下投影成点云数据
   * @param vLaserLong 原始激光的长度信息
   * @param vLaserPoint 投影后的激光点云数据
   * return 0 正常
   */
int StraightLine::mLaserDataTransformPointCloudData( const std::vector<double> &vLaserLong,
                                                    std::vector<cv::Point2d> &vLaserPoint){
    if(vLaserLong.size()<=0){
      std::cerr<<"laser is empty"<<std::endl;
      return -1;
    }
    int len = vLaserLong.size();
    std::cout<<"laser data size:"<<len<<std::endl;
    vLaserPoint.clear();
    for(int i=0;i<len;i++){
      vLaserPoint[i].x = vLaserLong[i]*sin((-(len-1)/2+i)*mLaserResolutionParam);
      vLaserPoint[i].y = vLaserLong[i]*cos((-(len-1)/2+i)*mLaserResolutionParam);
    }

  return 0;
}
  /***点云数据处理***
  * @brief将激光点云数据根据激光数据的疏密程度分成多组点云数据
  * @param vLaserLong 一组激光点云数据
  * @param vLaserPoint 处理后的多组激光点云数据
  * return 0 正常
  */
int StraightLine::mPointCloudDataTreat(const cv::Point2d&laserPoint,const std::vector<cv::Point2d> &vLaserPoint,
                                       std::vector<std::vector<cv::Point2d> > &vvLaserPoint){


  double laserDataDensity;   /*激光数据的 疏密Density 程度*/

  if(vLaserPoint.size()<=0){
    std::cerr<<"laser is empty"<<std::endl;
    return -1;
  }

  vvLaserPoint[0].push_back(vLaserPoint[0]);
  for(int i=1,j=0;i<vLaserPoint.size();i++){
    /*激光数据的疏密程度 = 自适应系数 * 激光到该点的距离 * 激光的分辨率  *****/
    laserDataDensity = mLineFitAdaptParam * getTwoPointDis(laserPoint,vLaserPoint[i-1]) * mLaserResolutionParam;

    if(getTwoPointDis(vLaserPoint[i-1],vLaserPoint[i])<laserDataDensity){
      vvLaserPoint[j].push_back(vLaserPoint[i]);
    }else{
      j++;
      vvLaserPoint[j].push_back(vLaserPoint[i]);
    }
  }

  /* test */
  {
  int numLaser = 0;
  for(int i=0;i<vvLaserPoint.size();i++){
     std::cout<<"vvLaserPoint["<<i<<"]:"<<vvLaserPoint[i].size()<<std::endl;
     numLaser +=vvLaserPoint[i].size();
  }
  std::cout<<"numLaser:"<<numLaser<<std::endl;
  }

  return 0;
}
  /*点云数据提取*/
int StraightLine::mPointCloudDataExtract(std::vector<std::vector<cv::Point2d>>&vvLaserPoint){

  std::vector<std::vector<cv::Point2d>>  vvLaser;
  for(int i=0,k=0;i<vvLaserPoint.size();i++){
     if(vvLaserPoint[i].size()>=2){
        if(vvLaserPoint[i].size()>2){
          cv::Point2d start = vvLaserPoint[i][0];
          cv::Point2d end = vvLaserPoint[i][vvLaserPoint[i].size()-1];
          double distMaxPTL = 0;
          int distMaxNum=0;
          for(int i=1;i<vvLaserPoint[k].size()-1;i++){
            double dist = mPointToLineDist(start,end,vvLaserPoint[k][i])>distMaxPTL;
            if(distMaxPTL>dist){
                distMaxPTL = dist;
                distMaxNum = i+1;
            }
          }
          if(distMaxNum>mLineExtractMaxParm){

          }
        }else{
          vvLaser[k].push_back(vvLaserPoint[i][0]);
          vvLaser[k].push_back(vvLaserPoint[i][1]);
          k++;
        }
     }
  }

  return 0;
}





/***计算点到直线的距离***
* @brief两点确定一条直线，计算另一个点到直线的距离
* @param start 直线起点
* @param end 直线中点
* @param Point 到直线的某个点
* return 点到直线的距离
*/
double StraightLine::mPointToLineDist(const cv::Point2d &start, const cv::Point2d &end, const cv::Point2d &Point){


  double delta;
  double angleSTP,angleSTE;
  double distSTP;

  angleSTP = atan2(Point.y-start.y,Point.x-start.x);
  distSTP = getTwoPointDis(start,Point);
  angleSTE = atan2(end.y-start.y,end.x-start.x);
  delta = fabs(angleSTP-angleSTE);


  std::cout<<"angleSTP:"<<angleSTP*180/3.14<<std::endl;
  std::cout<<"angleSTE:"<<angleSTE*180/3.14<<std::endl;
  std::cout<<"delta:"<<delta*180/3.14<<std::endl;
  std::cout<<"dist:"<<distSTP*sin(delta)<<std::endl;


  return distSTP*sin(delta);
}

  /*分界点的求取
   * @brief根据误差向求某一组点云数据
   * @param 输入点云数据
   * @param 可接受的误差项
   * @param 输出分界点索引
   * return 0 正常
   */
int StraightLine::mDemarcationPointGet(const std::vector<cv::Point2d> &vPoints, const double& AcceptError, std::vector<int> &vDemarcationIndex){
  if(vPoints.size()<2){
    std::cerr<<"input data is error"<<std::endl;
    return -1;
  }
  int maxIndex;
  double dist_PL;
  if(vPoints.size()>2){

    if(!mGetMidToStartEndLineMaxDist(vPoints,maxIndex,dist_PL)){
      if(dist_PL>AcceptError){
         vDemarcationIndex.push_back(maxIndex);
//         std::copy(vPoints,vPoints+maxIndex,);
      }
    }else {
      std::cerr<<"get max dist error"<<std::endl;
      return -1;
    }


  }else {
    vDemarcationIndex.push_back(0);
  }
  return 0;
}


/*求中间各点到起始点的最大距离
 * @brief 起点和终点连成直线求中间点到该直线的最大距离
 * @param 输入点云数据
 * @param 最大点索引   //第几个点最大
 * @param 最大距离
 * return 0 ok
 */
int StraightLine::mGetMidToStartEndLineMaxDist(const std::vector<cv::Point2d>&vPoints,int&MaxIndex,double&MaxDist){

  if(vPoints.size()<2){
    std::cerr<<"input data is error"<<std::endl;
    return -1;
  }

  MaxDist =0;
  double dist_PL;
  if(vPoints.size()>2){
    for(int i=1;i<vPoints.size()-1;i++){

     dist_PL =mPointToLineDist(vPoints.front(),vPoints.back(),vPoints[i]);

     if(dist_PL>MaxDist){
        MaxDist = dist_PL;
        MaxIndex = i+1;
     }

    }
  }else{

    MaxIndex = 0;
    MaxDist = 0;
  }


}
