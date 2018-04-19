#ifndef LASERDATADEAL_H
#define LASERDATADEAL_H

#include <cpose2d.h>
#include <vector>
#include <opencv.hpp>
#include <highgui.h>
#include <math.h>
#include <ceres/ceres.h>
#include <chrono>
#include <algorithm>

class laserDataDeal
{

public:
  laserDataDeal();
  ~laserDataDeal();

  /* 画箭头函数  地图  该点位姿  箭头长度*/
  void drawArrow(cv::Mat &,const CPose2D & ,int);
  /* 起点在障碍里时 ，将起点移出来   */
  int startPointInobstacle(cv::Mat &gridMap,cv::Point& start);
  /* 判断两点之间有障碍  地图 起点 终点*/
  bool isHaveObstacleBetweenPoints(cv::Mat &,const cv::Point&,const cv::Point&);
  /* 点和点之间的连线画在地图上  地图 点坐标*/
  int pointAndPointLinePaintInMap(cv::Mat& , std::vector<cv::Point2d>);
  /*从栅格地图中得到子图 地图 子地图 全局位姿 局部位姿 子地图的长度*/
  int getSubMapFromGridMap(const cv::Mat&src,cv::Mat&dst,const CPose2D&globalPose,CPose2D&subPose,int len);


  /*********     以下为仿真所用函数         ****************/
  /* 得到激光数据  地图 当前位姿 最小角 最大角 保存激光数据 保存障碍点*/
  int getLasterData(cv::Mat &Map, const CPose2D &RobotPose, const int &MinAngle, const int &MaxAngle,std::vector<double>&Laster,std::vector<CPose2D>&lasterPose);
  /* 得到激光数据的最大 长度、点  地图 当前位姿 最小角 最大角 最大长度 最大长度的点*/
  int getLasterDataMax( std::vector<double> &Laster, std::vector<CPose2D> &LasterPose,double&MaxLong,CPose2D&MaxPose);
  /* 得到激光数据的最小 长度、点  地图 当前位姿 最小角 最大角 最小长度 最小长度的点*/
  int getLasterDataMin( std::vector<double> &Laster, std::vector<CPose2D> &LasterPose,double&MinLong,CPose2D&MinPose);

  /* 得到激光三个方向距离障碍物的距离  地图 当前位姿 前方 左方 右方*/
  int getThreeDirectorDis( cv::Mat&Map,const CPose2D &RobotPose, double &Front, double &Left, double &Right);
  /* 得到激光三个方向距离障碍物的点  地图  当前位姿  前方  左方  右方 */
  int getThreeDirectorPoint( cv::Mat&Map,const CPose2D &RobotPose, CPose2D &FrontPose, CPose2D &LeftPose, CPose2D &RightPose);
  /* 根据激光数据以及 激光最大最小角度 得出 theta角度的长度*/
  int getLenAsLaserAngle(const std::vector<double>&Laser, const double &MinAngle, const double &MaxAngle,const double&Angle,double&Len);
  /* 机器人前方障碍点  地图  机器人位姿*/
  CPose2D extendUntilHaveObstacle(const cv::Mat&,const CPose2D &);

  /* 得到机器人下一个点位姿正前方  地图  机器人位姿  前进长度*/
  CPose2D PoseInFrontOfRobot(const cv::Mat& Map,const CPose2D &RobotPose,const double & Len);
  /* 得到机器人上一个点位姿正后方 地图  机器人位姿  前进长度*/
  CPose2D PoseInRearOfRobot(const cv::Mat& Map,const CPose2D &RobotPose,const double & Len);
  /* 得到机器人下一个点位姿  地图  机器人位姿  角度*/
  CPose2D nextCPose2DRo(const CPose2D &RobotPose,const double & Angle);

  /*沿着某一方向寻找可同行的点*/
  int FindPassablePoint( cv::Mat&Map,const double angle,const cv::Point2d&RobotPose,cv::Point2d&nextPint);

  /* 膨胀障碍距离  地图  膨胀长度*/
  int expendGridMapRelyNum(cv::Mat &gridMap,cv::Mat& expendGridMap,int expendGridMapNum);

  /*点 线 关系*/
  /* 得到线段上所有的点  线段的起始点  存放所有点的容器  该直线的倾角*/
  bool getLineAllPoint(const cv::Vec4i&, std::vector<cv::Point2d>&,double&);
  /* 判断一个点是否在一个线段上*/
  bool onePointIfInOneLine(const cv::Point2d &start, const cv::Point2d &end,const cv::Point2d &point);
  /* 一个点到容器中所有点的最近距离  起点 存放所有点的容器  最近点的坐标*/
  double onePointToVectPointNearPoint(const cv::Point2d&, std::vector<cv::Point2d>&,cv::Point2d&);
  /* 得到某点到多条线段最近点的位姿  多条线段 起点  输出点*/
  bool getPose2dFromVet4iNearPoint(std::vector<cv::Vec4i>&,const CPose2D &, CPose2D &);

  /*x*/
  /**两张图片的异或 地图1  地图2  目的地图  程度 **/
  int twoMaps_xor(const cv::Mat&map1,const cv::Mat&map2,cv::Mat&dstMap,const int&level);
  /**一张图的取反**/

  /*求取点到直线的距离*/
  int getPointToLineMsgs(const CPose2D& LinePose,const cv::Point2d& Point,bool&thisInPointReal);
  int getPointToLineMsgs(const cv::Point2d &start, const cv::Point2d &end, const cv::Point2d &Point,double&dist,double&angle);
  /* 判断直线与线段的焦点是否在另一条线段上   与坐标系无关*/
  bool twoLinesIfHaveSamePointInSecondLines(const cv::Point2d &start1, const cv::Point2d &end1,
  /***两条直线的相似度***/                                    const cv::Point2d &start2, const cv::Point2d &end2,cv::Point2d&dstPoint2d);
  double twoLinesSimilarity(const cv::Point2d &start1, const cv::Point2d &end1,const cv::Point2d &start2, const cv::Point2d &end2);

  /**将激光数据投影成点**/
  int  laserDataTransformPointCloudData(  const double&minAngle,const double&maxAngle,const std::vector<double> &vLaserLong,std::vector<cv::Point2d> &vLaserPoint);

    /**最小二乘法求取一个均值**/
  int  getLinesPoint2dSlope(std::vector<cv::Point2d>&vLinePoint2d,double& k,double& b);
    /**最小二乘法求取一个均值**/
  int  getANumaverage(std::vector<double>&vValue,double& average);

  /*********     霍夫变换用      ****************/
  /* 将激光数据投影到栅格图上  初始地图  投影地图  机器人位姿  激光数据  激光起始角度*/
  void getMapByLasterDataMappingToMap(const cv::Mat &, cv::Mat& ,const CPose2D & , std::vector<double>&,const int&);
  /* 霍夫变换得到多条直线  地图  线段存储器*/
  void thoughHoughLineGetLinePoint(cv::Mat&,std::vector<cv::Vec4i>&);
  void getMinNearLine(const CPose2D&,std::vector<double>&,const int&);


private:

  cv::Mat m_lacalMap;

  /*判断某个栅格是障碍和非障碍的参数*/
  int  mJudgeObstacleParm = 150;



};

#endif // LASERDATADEAL_H
