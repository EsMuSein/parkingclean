#ifndef DATASAVE_H
#define DATASAVE_H

#include <boost/thread.hpp>

#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <algorithm>
#include <cpose2d.h>
#include <opencv.hpp>

class dataSave
{
public:
 dataSave();

 int getRobotSpeed(geometry_msgs::TwistPtr& speed);
 int setRobotSpeed(const geometry_msgs::TwistPtr& speed);

 int getRobotPose(CPose2D& robotPose);
 int setRobotPose(const CPose2D& robotPose);

 int getRobotPoseMap(CPose2D& robotPoseMap);
 int setRobotPoseMap(const CPose2D& robotPoseMap);

 int getLaserData(std::vector<double>& vlaserData);
 int setLaserData(const std::vector<double>& vlaserData);

 int getThreeDirectDist(Struct::sDists& threeDirectDist);
 int setThreeDirectDist(const Struct::sDists& threeDirectDist);

 int getSystermMode(Enum::modes& modes);
 int setSystermMode(const Enum::modes& modes);

 int getRobotHandleControl(Enum::rControls& rControls);
 int setRobotHandleControl(const Enum::rControls& rControls);

 int getAllRobotPoseInMap(std::vector<cv::Point2d>& mVRobotPose);
 int getAllRobotPose(std::vector<cv::Point2d>& mVRobotPose);
 int clearAllRobotPose();
 int setAllRobotPose(const cv::Point2d& MRobotPose);

 int getMotionRobotPoint(std::vector<cv::Point2d>& mVRobotPose);
 int setMotionRobotPoint(const std::vector<cv::Point2d>& MVRobotPose);

 int getGlobalMapMsg(cv::Mat& map);
 int setGlobalMapMsg(const cv::Mat& map);

 int getSubMapMsg(cv::Mat& map,CPose2D&subGridMapPose);
 int setSubMapMsg(const cv::Mat& map,const CPose2D&subGridMapPose);

 int getMotionRobotPose(std::vector<CPose2D>& vMotionRobotPose);
 int setMotionRobotPose(const std::vector<CPose2D>& vMotionRobotPose);

 int getMapOriginParm(cv::Point2d&MapOriginPram);
 int setMapOriginParm(cv::Point2d&MapOriginPram);

 int getRobotIfArriveDestPose(bool&robotifArriveDestPose);
 int setRobotIfArriveDestPose(const bool&robotifArriveDestPose);

 int getNowMotionPose(CPose2D& NowMotionPose);
 int setNowMotionPose(const CPose2D& NowMotionPose);

private:
 /*互斥操作 速度命令互斥锁*/
boost::mutex mMutexRobotSpeed;
/*互斥操作 机器人位姿互斥锁*/
boost::mutex mMutexRobotPose;
/*互斥操作 机器人位姿互斥锁*/
boost::mutex mMutexRobotPoseMap;
/*互斥操作 激光数据*/
boost::mutex mMutexLaserData;
/*互斥操作 机器人三个方向距离*/
boost::mutex mMutexThreeDirectDist;

/*互斥操作 按键模式和机器人控制*/
boost::mutex mMutexSystermMode;
boost::mutex mMutexRobotHandleControl;

/*互斥操作 机器人位姿*/
boost::mutex mMutexAllRobotPose;

/*互斥操作 导航包机器人运动点*/
boost::mutex mMutexMotionRobotPose;

/*互斥操作 全局地图和局部地图*/
boost::mutex mMutexGridMapMsg;
boost::mutex mmutexSubMapMsg;

 /*机器人运行时的路径点*/
boost::mutex mMutexMotionPoint;

 /*保存地图变化参数*/
boost::mutex mMutexMapOriginPram;
/*机器人是否到达目标点*/
boost::mutex mMutexIsRobotArriveDestPose;

/**机器人当前运行目标点**/
boost::mutex mMutexNowMotionPose;
private:
/*机器人是否到达目标点*/
 bool misRobotArriveDestPose = false;

  /* 机器人当前的速度 */
 geometry_msgs::TwistPtr cmd_;

 /*机器人当前位姿   与起始点的实际位姿  unit:m*/
 CPose2D mRobotPose;

 /*机器人当前位姿   图像坐标系  unit:grid*/
 CPose2D mRobotPoseMap;

 /*机器人运行时的路径点*/
 std::vector<cv::Point2d> mVMotionPoint;    //演变行走时的路径

 /*机器人全局地图  unit:grid  resolusion:0.05  */
 cv::Mat mGlobalGridMap;

 /**机器人导航运动时的目标点**/
 std::vector<CPose2D> mvMotionRobotPose;


 CPose2D mRP_SubGridMap;
 cv::Mat msubGridMap;

 /*所有的机器人位姿*/
 std::vector<cv::Point2d> mvRobotPose;      //该位姿是接听到的机器人的所有位姿

 /*保存地图变化参数*/
 cv::Point2d mMapOriginPram;      //栅格坐标  表示

 /*当前激光数据*/
 std::vector<double> mLaserData;
 /*当前激光三个方向距离*/
 Struct::sDists msThreeDirectDist;
 /*机器人模式和控制*/
 Enum::modes mModes;
 Enum::rControls mRControls = Enum::rStop;


 CPose2D mNowMotionPose;


public:
 /**坐标系间的转换**/
 double coordinateFromMapToRobot(const double &Angle);
 double coordinateFromRobotToMap(const double &Angle);
 CPose2D coordinateFromMapToRobot(const CPose2D &RobotPose);
 CPose2D coordinateFromRobotToMap(const CPose2D &RobotPose);
 cv::Point2d coordinateFromMapToRobot(const cv::Point2d &RobotPose);
 cv::Point2d coordinateFromRobotToMap(const cv::Point2d &RobotPose);
private:
 double mdResolution = 0.05;
};

#endif // DATASAVE_H
