#ifndef PARKINGCLEAN_H
#define PARKINGCLEAN_H
#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <ecl/threads.hpp>

#include <opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <cpose2d.h>
#include <laserdatadeal.h>
#include <datasave.h>
#include <straightline.h>
#include <line.h>

#include <message_filters/subscriber.h>     //
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <tf/tf.h>          //tf转换
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <client/myMsg.h>       //按键

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <std_msgs/String.h>

#include <X11/Xlib.h>
#include <pthread.h>
#include <boost/filesystem.hpp>
#include <boost/bind/bind.hpp>
#include <rosconsole/macros_generated.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base/MoveBaseConfig.h>
#include <move_base/move_base.h>

//#define TEST_ROBOT_CONTROL
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class parkingClean
{
public:

  parkingClean();
  ~parkingClean();

  /*****************
   ** init
   ** *************/
  bool init();
  /*****************
   ** Runtime
   ** ***********ee**/
  void spin();

private:
  /* 发送 速度话题   状态话题*/
  ros::Publisher velocity_pub, message_pub;
  bool mlastZeroVelSend;
  /* 接收 激光话题   按键话题*/
  ros::Subscriber laser_sub,client_sub,imu_sub,msgs_String_sub;
  ros::Subscriber globalMap_sub;
  MoveBaseClient* ac;

  /*类 激光数据处理指针*/
  laserDataDeal *p_laserDataDeal;
  /*类 数据保存*/
  dataSave *p_dataSave;
  /*类 直线求取*/
  StraightLine *p_straightLine;

  /*该节点的指针*/
  std::string name_;


  /*系统所处的     模式 状态*/
  Enum::modes mModes = Enum::emIdle;    //模式
  Enum::states mStates = Enum::esWalkNearWall;  //状态
  Enum::sstates mSStates = Enum::essStop;   //状态下的小状态
  Enum::rControls mRControls = Enum::rStop;  //机器人的控制命令
  bool isFirstIntoMode = true;
  bool isFirstIntoState = true;
  bool isFirstIntoSState = true;
  bool byWallEnd = true;

//  bool isFirstcheckRearMoretanMaxClean =false;

  /*program 全局变量*/
  CPose2D mRoadPose,mIntoRobotPose;  /**道路位姿**/
  CPose2D mFirstIntoCleanRobotPose,mEndIntoCleanPose;
  CPose2D mRobotPose,mLastRobotPose;
  bool mIsCleanFinish = false;
  double  mTraTemplate = 1;// 1 正常;2  梯形;3  楔形
  int mTraTimes = 0;
  double mDist_F,mDist_R,mDist_L;
  double mDestTheata =0 ;     //清扫时  每次需要旋转的角度
  double mRealTimeTheata = 0;
  double mstaticTheata = 0;
  double mDistIntoCar=0;    //d
  /********************************
  ***机器人运动参数设定
  ***********************/
  /*机器人运动时，判断因素   单位 m*/
  double mFMinDist = 0.5;    //前方v  最小距离    当前方小于该距离时，机器人旋转
  double mRMinDist = 0.3;    //右方最小距离
  double mRMaxDist = 1;      //右方最大距离   检测到右方距离大于该距离时默认进入清扫区
  double mFCleanDist = 0.3;    //清扫距离
  double mFRealCleanDist;  //实际前进距离
  double mRobotCleanDistLen;    //记录机器人清扫时进去的距离
  double mRobotSize = 0.52;    //机器人半径
  double mIsParkingCleanNext = false;   //下此是否仍然能进行清扫
  double mIsParkingCleanNow = false;    //本次是否可以清扫
  tf::TransformBroadcaster br;
  /*机器人的运行速度   线速度  旋转角速度  矫正角速度*/
  const double mSpeedLiner =0.3;
  const double mSpeedAngule = 0.5;
  const double mSpeedAnguleCheck = 0.2;

  const double muSleepTimes = 1000*200;

  /* 激光、按键、全局地图 回调函数*/
  void laserCallBack(const sensor_msgs::LaserScanConstPtr& laser);
  void imuCallBack(const sensor_msgs::ImuConstPtr&imu);
  void clientCallBack(const client::myMsgConstPtr& client);
  void stringCallBack(const std_msgs::StringConstPtr& str);
  void globalMapCallBack(const nav_msgs::OccupancyGridConstPtr& globalMap);
  /*测试全局地图时的位姿*/
  void subMapCallBack(const nav_msgs::OccupancyGridConstPtr& subMap);

  /*用于图像的翻转 0:不翻转  1:上下翻转  2:左右翻转  3:上下左右翻转*/
  void mMapTransform(const cv::Mat&srcImage,cv::Mat&dstImage,const int& type);

  /*空闲*/
  void mIdleModeDealThing();    //空闲模式下处理的事情
  /*工作*/
  void mWorkModeDealThing();    //工作模式下处理的事情
  void mWorkWalkByWall();       //沿墙走
  void mWorkParkingClean();     //清扫
  void mWorkParkingClean2();     //清扫
  void mWorkWalkNearWall();     //靠近墙

  void mWorkWalkByWall1();       //沿墙走
  void mWorkParkingClean1();     //清扫
  void mWorkWalkNearWall1();     //靠近墙
  /*测试*/
  void mTestModeDeal();     //测试所用
  /*机器人角度矫正函数*/
  void mTheataRotateCorrection();
  /**线程函数   未启动**/
  void run_process();   /*该线程用于处理主要函数 */
  void run_testPointToPoint();
  void run_processTest();
  void listenRobotPose();  /**该线程用于接受机器人当前的位姿  并且转换到栅格地图上**/
  void run_process2();  /*无用*/
  void run_process3();  /*记录得到边沿点*/
  void run_process4();  /*记录得到边沿点*/
  void run_process5();  /*测试导航包 发送目标点  无用*/
  /**该函数用于发送目标点，导航包自动到达该店**/
  //  bool moveToGoal(double xGoal, double yGoal,tf::Quaternion quater);
  bool moveToGoalAsWorld(const double&Goal_x,const double&Goal_y,const tf::Quaternion& GoalOrigen=tf::Quaternion(0.0,0.0,0.0,1));
  bool moveToGoalAsWorld(const double&Goal_x,const double&Goal_y,const double&Angle);
  bool moveToGoalAsWorld(const CPose2D&robotPose);
  bool moveToGoalAsMap(const CPose2D&robotPose);
  bool moveToGoalAsRobot(const double&Goal_x,const double&Goal_y,const tf::Quaternion& GoalOrigen=tf::Quaternion(0.0,0.0,0.0,1));
  bool moveToGoalAsRobot(const double&Goal_x,const double&Goal_y,const double&Angle);
  bool moveToGoalAsRobot(const CPose2D&robotPose);
  /***计算路径点以及方向***/
  int pathPointTransformCoordinateMapToRobot(const CPose2D&robotPose,std::vector<cv::Point2d>& vpathPoint,std::vector<CPose2D>&vrealPathPose);

  void doneCbAsRobot(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void doneCbAsWorld(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  /* 线程函数*/
  ecl::Thread thread_,thread1_,thread_test,thread_test1;
  boost::thread thread11();

  /*得到边沿点的坐标*/
  int getBorderPoint( cv::Mat&srcImage,const double& robotSize,const double& roadDirection,
                               bool& isFirstTest,const cv::Point2d& initPoint, std::vector<cv::Point2d>& vTestMotionPoint);
  /*得到边沿点的坐标*/
  int getBorderPointRealTime( cv::Mat&srcImage,const double&robotSize,const CPose2D&robotPose,std::vector<cv::Point2d>&vMotionPoint);
  /*得到边沿点的坐标*/
  int getBorderPointRealTimeInLine( cv::Mat&srcImage,const double&robotSize,const CPose2D&robotPose,std::vector<cv::Point2d>&vMotionPoint);

  /**计算斜率**/
  int getBorderSlope( cv::Mat&srcImage,const CPose2D&robotPose,CPose2D&borderPose);
  /*清扫某一封闭区域*/
  int coverageCleanUpAClosedArea(cv::Mat&srcImage,const CPose2D& rInitPose,const cv::Point2d&DirectClean,const double&radiusClean);

  /*机器人运动控制函数*/
  void forwardRobot(double Speed);  /*前后 +前 -后*/
  void levelRobot(double Speed);    /*左右平移  +左 -右*/
  void rotateRobot(double Speed);   /*旋转  +左传  -右转*/
  void stopRobot();     /*机器人 stop*/
  void forwardDist(CPose2D& robotPose,double dist,double Speed ); /*前进固定距离 死循环中 基本不用了*/
  void rotateAngle(CPose2D& robotPose,double angle,double Speed); /*旋转固定距离 死循环中 基本不用了*/

  void moveRobot(const double&speed,const double&angle);

  bool  firstTest =true;
  CPose2D dstPose;
  CPose2D startPose;
  int m_i = 0;


  double __speedRobot = 0.1;

  //PID motion control
  double mKp = 2;    //比例系数
  double mKi = 1;    //积分系数
  double mKd = 0.5;    //微分系数
  double mEK = 0;
  double mLEk = 0;    //上一次误差
  double mLLEk = 0;   //上上一次误差
  double mLOP = 0;    //上一次控制量
  double PIDControl(double setValue,double fBValue);  //PID计算其差值
  void PIDParmReset();   //PID参数归零

  /*保留一些全局数据*/
  std::vector<cv::Point2d> mvTestPoint;
//  double mLineSimlarity = 50;

};

#endif // PARKINGCLEAN_H
