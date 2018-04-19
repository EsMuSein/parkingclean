#ifndef STRAIGHTLINE_H
#define STRAIGHTLINE_H

#include <opencv.hpp>
#include <cpose2d.h>


class StraightLine
{
public:
  StraightLine();
  StraightLine(double laserResolution);

  ~StraightLine();


  /**Straight line detection algorithm**/
  int getStraightLine;

private:

  /*激光数据直线拟合算法*/
  /*将激光数据转换为点云数据*/
  int mLaserDataTransformPointCloudData(const std::vector<double>&vLaserLong,std::vector<cv::Point2d>&vLaserPoint);
  /*点云数据处理*/
  int mPointCloudDataTreat(const cv::Point2d& laserPoint,const std::vector<cv::Point2d>&vLaserPoint,std::vector<std::vector<cv::Point2d>>&vvLaserPoint);
  /*点云数据提取*/
  int mPointCloudDataExtract(std::vector<std::vector<cv::Point2d>>&vvLaserPoint);
  /*点云数据拟合直线*/
  int mPointCloudDataFitStraightLine(std::vector<std::vector<cv::Point2d>>&vvLaserPoint,std::vector<StraightLine>&vStraightLine);
  /*直线拟合误差*/
  int mFittingStraightLineError();
  /*直线合并*/
  int mStraightLineMerger();


  /*求取点到直线的距离*/
  double mPointToLineDist(const cv::Point2d& start,const cv::Point2d& end,const cv::Point2d& Point);

public:

//  /*点到直线的信息*/
//  int mPointToLineMsgs(const cv::Point2d &start, const cv::Point2d &end, const cv::Point2d &Point,double&dist,double&angle);


  /*分界点的求取*/
  int mDemarcationPointGet(const std::vector<cv::Point2d>&vPoints,const double& AcceptError,std::vector<int>&vDemarcationIndex);
  /*求中间各点到起始点的最大距离*/
  int mGetMidToStartEndLineMaxDist(const std::vector<cv::Point2d>&vPoints,int&MaxIndex,double&MaxDist);

private:


  double mLaserResolutionParam = 0.5;   //激光分辨率
  double mLineFitAdaptParam = 0.05;   //直线拟合自适应调整参数

  double mLineExtractMaxParm = 0.5;



public:
  double theata_l;  //直线法向角
  double theata_sl; //直线起点角度
  double theata_el; //直线终点角度
  double dist_l;   //激光原点到线段的最近距离
  double length_l;    //线段的长度
  double k_l;     //直线斜率
  double b_l;     //直线截距

  cv::Point2d start_l;    //起点
  cv::Point2d end_l;    //终点



};

#endif // STRAIGHTLINE_H
