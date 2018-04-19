#include "parkingclean.h"
#include <chrono>

using namespace std;

parkingClean::parkingClean(): p_laserDataDeal(new laserDataDeal()),p_dataSave(new dataSave()),p_straightLine(new StraightLine()),ac(new MoveBaseClient("move_base",true))
{

}
parkingClean::~parkingClean(){

      stopRobot();
      cout<<"~parkingClean() "<<endl;
      delete p_laserDataDeal;
      delete p_dataSave;
      delete p_straightLine;
}

bool parkingClean::init(){

    XInitThreads();   /*多线程显示图像*/

  ros::NodeHandle nh("~");
  name_ = nh.getUnresolvedNamespace();

  /*********************
   **Parameters
   **********************/
  nh.getParam("front_min_dist",mFMinDist);
  nh.getParam("right_min_dist",mRMinDist);
//  nh.param("right_min_dist",mRMinDist,200);
  nh.getParam("right_max_dist",mRMaxDist);
  nh.getParam("front_clean_dist",mFCleanDist);

//  cout<<"mFMinDist " << mFMinDist  <<endl;
//    cout<<"mRMinDist " << mRMinDist  <<endl;
  /*********************
   ** Publishers
   **********************/
//  velocity_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi",1,true);   //turtobot
//    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true); //仿真
//    velocity_pub = nh.advertise<geometry_msgs::Twist>("/qfeel/cmd_vel",1,true); //仿真
  velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/output/cmd_vel",1,true);
  /*********************
   ** subscribers
   **********************/
  laser_sub = nh.subscribe("/scan",1, &parkingClean::laserCallBack,this);
  globalMap_sub = nh.subscribe("/map",1,&parkingClean::globalMapCallBack,this);
  client_sub = nh.subscribe<sensor_msgs::Imu>("/qfeel/imu",1,&parkingClean::imuCallBack,this);

 // velocity_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1,&parkingClean::velocityCallBack,this);


  msgs_String_sub = nh.subscribe<std_msgs::String>("/string_chatter",10,&parkingClean::stringCallBack,this);
  client_sub = nh.subscribe<client::myMsg>("/client_chatter",10,&parkingClean::clientCallBack,this);

  stopRobot();
  cout<<"thread_.start "<<endl;
//  thread_test.start(&parkingClean::run_process2, *this);    //求斜率
  thread_test1.start(&parkingClean::run_testPointToPoint, *this);   //测试导航包发送点的坐标
  thread_.start(&parkingClean::run_process4, *this);
//  thread_.start(&parkingClean::run_process, *this);     //以前方案
  thread1_.start(&parkingClean::listenRobotPose, *this);     //接听机器人实时位姿
}

void parkingClean::spin(){

  ros::Rate loop_rate(20);

 geometry_msgs::TwistPtr  cmd_(new geometry_msgs::Twist());



  while (ros::ok()) {

   p_dataSave->getRobotSpeed(cmd_);

    if((cmd_->linear.x != 0.0)||(cmd_->linear.y != 0.0)||(cmd_->linear.z != 0.0)||
       (cmd_->angular.x != 0.0)||(cmd_->angular.y != 0.0) ||(cmd_->angular.z != 0.0))
    {
      velocity_pub.publish(cmd_);
      mlastZeroVelSend = false;
    }else if(mlastZeroVelSend==false){
      velocity_pub.publish(cmd_);
      mlastZeroVelSend = true;
    }
  //  cout<<" run init"<<endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//void parkingClean::velocityCallBack(const geometry_msgs::TwistConstPtr &velocity){

//  geometry_msgs::TwistPtr cmd_(new geometry_msgs::Twist());
//     cmd_->linear.x = velocity->linear.x;
//     cmd_->linear.y = velocity->linear.y;
//     cmd_->linear.z = velocity->linear.z;
//     cmd_->angular.x = velocity->angular.x *180 /3.14;
//     cmd_->angular.y = velocity->angular.y *180 /3.14;
//     cmd_->angular.z = velocity->angular.z *180 /3.14;

//     velocity_pub.publish(cmd_);
////  p_dataSave->setRobotSpeed(cmd_);


//}

void parkingClean::laserCallBack(const sensor_msgs::LaserScanConstPtr& laser){

//   cout<<"ranges.size(): "<<laser->ranges.size()<<endl;
//   cout<<"ranges.angle_min: "<<laser->angle_min<<endl;
//   cout<<"ranges.angle_max: "<<laser->angle_max<<endl;
//   cout<<"ranges.range_min: "<<laser->range_min<<endl;
//   cout<<"ranges.range_min: "<<laser->range_max<<endl;

   int lenth = laser->ranges.size();   //激光数据长度
   std::vector<double>laserData;
   Struct::sDists threeDirectData;
   laserData.clear();
   laserData.resize(lenth);
   for(int i=0;i<(lenth);i++){
     if(laser->ranges[i]<0||laser->ranges[i]>10)
       laserData[i] = 10;
     else
       laserData[i]=(laser->ranges[i]);
    }  
  p_dataSave->setLaserData(laserData);

  p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,0,threeDirectData.F);
  p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,-90,threeDirectData.R);
  p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,90,threeDirectData.L);
  p_dataSave->setThreeDirectDist(threeDirectData);
//    cout<<"threeDirectData F "<< threeDirectData.F <<endl;
//    cout<<"threeDirectData R "<< threeDirectData.R <<endl;
//    cout<<"threeDirectData L "<< threeDirectData.L <<endl;
}

void parkingClean::imuCallBack(const sensor_msgs::ImuConstPtr &imu){
//  imu->orientation
  cout<<"+++++ "<<endl;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0,0,0));
  transform.setRotation(tf::Quaternion(0,0,0,1));
  cout<<"imu->header.frame_id "<< imu->header.frame_id  <<endl;
  std::string std = "/qfeel/imu1";
  br.sendTransform(tf::StampedTransform(transform,imu->header.stamp,"/odom",std));

}
void parkingClean::stringCallBack(const std_msgs::StringConstPtr &str){
  string str1 = str->data;
  int a = 49, b = 57;
  Enum::modes mModes= Enum::emIdle;
  Enum::rControls mRControls= Enum::rStop;
  p_dataSave->getSystermMode(mModes);
  if(str1[0]=='1'&&str1[1]=='a'){
    a = int (str1[2]);
  }else if(str1[0]=='2'&&str1[1]=='a'&&(mModes == Enum::emIdle)){
    b = int (str1[2]);
  }
//  cout<<"stringCallBack stringCallBack "<<endl;
  switch (a) {
  case 49:
    mModes = Enum::emIdle;
    isFirstIntoMode = true;
    break;
  case 50:
    mModes = Enum::emWork;
    isFirstIntoMode = true;
    break;
  case 51:
    mModes = Enum::emTest;
    isFirstIntoMode = true;
    break;
  default:

    break;
  }
  p_dataSave->setSystermMode(mModes);

  switch (b) {
  case 113:
    mRControls = Enum::rLeftRotate;   // q
    break;
  case 119:
    mRControls = Enum::rFront;   //w
    break;
  case 101:
    mRControls = Enum::rRightRotate;   //e
    break;
  case 57:
    mRControls = Enum::rStop; //' '
    break;
  case 115:
    mRControls = Enum::rRear;   //s
    break;
  case 97:
    mRControls = Enum::rLeft;  //a
    break;
  case 100:
    mRControls = Enum::rRight;   //d
    break;
  default:
//    ROS_INFO_STREAM("mode operator is error");
    break;
  }
  p_dataSave->setRobotHandleControl(mRControls);

}

void parkingClean::clientCallBack(const client::myMsgConstPtr& client){

  Enum::modes mModes;
  Enum::rControls mRControls;
  switch (client->mode) {
  case 0:
    mModes = Enum::emIdle;
    isFirstIntoMode = true;
    break;
  case 1:
    mModes = Enum::emWork;
    isFirstIntoMode = true;
    break;
  case 2:
    mModes = Enum::emTest;
    isFirstIntoMode = true;
    break;
  default:

    break;
  }
//  p_dataSave->setSystermMode(mModes);

//  cout<<"clientCallBack clientCallBack "<<endl;

  switch (client->operate) {
  case 0:
    mRControls = Enum::rLeft;
    break;
  case 1:
    mRControls = Enum::rFront;
    break;
  case 2:
    mRControls = Enum::rRight;
    break;
  case 3:
    mRControls = Enum::rStop;
    break;
  case 4:
    mRControls = Enum::rRear;
    break;
  default:
    ROS_INFO_STREAM("mode operator is error");
    break;
  }
//  p_dataSave->setRobotHandleControl(mRControls);
}


void parkingClean::globalMapCallBack(const nav_msgs::OccupancyGridConstPtr& globalMap){

//    CPose2D robotPose;
  std::vector<cv::Point2d> mvRobotPose;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    unsigned char cbuf[globalMap->info.height][globalMap->info.width];

    for(int i=0,k=0;i<globalMap->info.height;i++){    //转变为二位数组
      for(int j=0;j<globalMap->info.width;j++){
        cbuf[i][j]=255-globalMap->data[k];
        k++;
      }
    }
    /**机器人相对与左上角原点的参数**/
    cv::Point2d MapOriginPram;
    MapOriginPram.x = round (- (globalMap->info.origin.position.x/globalMap->info.resolution));   //cols
    MapOriginPram.y = round (globalMap->info.height + globalMap->info.origin.position.y/globalMap->info.resolution);
    p_dataSave->setMapOriginParm(MapOriginPram);
    cv::Mat srcImage(globalMap->info.height,globalMap->info.width,CV_8UC1,(unsigned char*)cbuf);   //得到全局地图
    mMapTransform(srcImage,srcImage,1);
    p_dataSave->setGlobalMapMsg(srcImage);

}

/**该线程用于接受机器人当前的位姿  并且转换到栅格地图上**/
void parkingClean::listenRobotPose(){

  CPose2D robotPose;
  CPose2D mlastPose,robotPoseMap;
  tf::TransformListener listen;
  tf::StampedTransform transform;

  while(ros::ok()){

    /*用来接听机器人当前位姿*/
    if(listen.waitForTransform("map","base_link",ros::Time(0),ros::Duration(0.05))){
      listen.lookupTransform("map","base_link",ros::Time(0),transform);
      double pitch,roll,yaw;
      transform.getBasis().getRPY(pitch,roll,yaw);
      robotPose.x((transform.getOrigin().x()));
      robotPose.y((transform.getOrigin().y()));
      robotPose.phi((yaw));
    }
        /*用于设定机器人的初始位姿*/
     p_dataSave->setRobotPose(robotPose);   /*机器人坐标系*/
     //    cout<<"  robot pose /m:"<<robotPose<<endl;
     robotPoseMap = p_dataSave->coordinateFromRobotToMap(robotPose);   /*地图坐标系*/
     p_dataSave->setRobotPoseMap(robotPoseMap);
     //    cout<<"global map robot pose /grid:"<<robotPose<<endl;

//        cout<<" --------------"<<robotPose.toPoint()<<endl;
     if((robotPoseMap.toPoint().x!=mlastPose.toPoint().x)||(robotPoseMap.toPoint().y!=mlastPose.toPoint().y)){   /**/
        p_dataSave->setAllRobotPose(cv::Point2d(robotPose.x(),robotPose.y()));
        mlastPose = robotPoseMap;
//        cout<<" --------------"<<robotPose.toPoint()<<endl;
     }

  }
}

/*清扫某一封闭区域
 * @param 第一  输入地图
 * @param 第二  机器人当前所处的位姿
 * @param 第三  清扫的方向（两个方向）
 * @param 第四  每次清扫的半径
 * 返回  错误 -1  正确 0
 */
int parkingClean::coverageCleanUpAClosedArea(cv::Mat&srcImage,const CPose2D& rInitPose,const cv::Point2d&DirectClean,const double&radiusClean){

  if(srcImage.empty()){   /*地图为空时报错，返回 -1*/
    std::cerr<<"输入图像为空"<<std::endl;
    return -1;
  }
  int height =  srcImage.rows /radiusClean;
  int width = srcImage.cols / radiusClean;
  cv::Mat cleanMap(height,width,CV_8UC2,cv::Scalar(0));

  for(int i=0;i<srcImage.rows;i+=radiusClean){    /*行*/
     for(int i=0;i<srcImage.cols;i+=radiusClean){   /*列*/

     }
  }


}


int parkingClean::getBorderSlope( cv::Mat&srcImage,const CPose2D&robotPose, CPose2D&borderPose){

  if(srcImage.channels()==3)    /**地图如果是三通道  改成单通道**/
    cv::cvtColor(srcImage,srcImage,CV_BGR2GRAY);

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  /*记录该函数运行前的时间*/

  if(srcImage.empty()){   /*地图为空时报错，返回 -1*/
    std::cerr<<"输入图像为空"<<std::endl;
    return -1;
  }
  std::vector<cv::Point2d> vGetLinePoint;   /**保存在该线上符合条件的点**/
  cv::Mat workMap,borderMap,borderMap_,tolerantFaultMap;  /*定义一些地图信息   求异或时的地图*/
  p_laserDataDeal->expendGridMapRelyNum(srcImage,workMap,2); /*膨胀机器人宽度*/
  p_laserDataDeal->expendGridMapRelyNum(srcImage,borderMap_,3);  /*膨胀机器人宽度+1*/
  p_laserDataDeal->expendGridMapRelyNum(borderMap_,borderMap,1);/*求边界范围*/
  p_laserDataDeal->expendGridMapRelyNum(borderMap_,tolerantFaultMap,5);/*膨胀容错范围*/

  cv::Mat getBorderMap,getTolerantFaultMap;         /*定义一些地图信息  异或地图*/
  p_laserDataDeal->twoMaps_xor(borderMap_,borderMap,getBorderMap,150); /*异或得到边界地图*/
  p_laserDataDeal->twoMaps_xor(workMap,tolerantFaultMap,getTolerantFaultMap,150); /*异或得到边界范围地图*/
  /**地图取反**/
  cv::Mat xorTolerantFaultMap = getBorderMap.clone();
  xorTolerantFaultMap = ~xorTolerantFaultMap;
  /**求取激光到该直线的最近点**/
  std::vector<double> laser; std::vector<CPose2D> laserPose;
  p_laserDataDeal->getLasterData(xorTolerantFaultMap,robotPose,45,90,laser,laserPose);
  if(laser.size()<1||laserPose.size()<1) {
    cout<<"laser data empty   error: "<<endl;
    return -1;
  }
  cv::Point2d minPose; double len;
  if(p_laserDataDeal->getLasterDataMin(laser,laserPose,len,borderPose)==-1){
    cout<<"getLasterDataMin   error: "<<endl;
    return -1;
  }
  minPose = borderPose.toPoint();
  cout<<"len:"<<len<<endl;
  cout<<"minpose:"<<minPose<<endl;

  cv::Point2d lastpose;
  double k,b;   /**该直线的斜率**/
  double theata = 0;
  bool isfirst = true;
  int num = 0;
  while(ros::ok()){
    num++;
    vGetLinePoint.push_back(minPose);
    if((p_laserDataDeal->FindPassablePoint(getBorderMap,robotPose.phi_angle(),minPose,minPose)!=0)){
      if(num<10){
        cout<<"FindPassablePoint is too few   num: "<< num<<endl;
        return -1;
      }else {
        p_laserDataDeal->getLinesPoint2dSlope(vGetLinePoint,k,b);
        if(k=0) k=0.0001;
         theata =(atan(1/k)*180/3.14);
         if(fabs(robotPose.phi_angle())>90){
             if(theata>0) theata -=180;
             else if(theata<0) theata +=180;
         }
        borderPose.phi_angle(getTwoPointAngleInMap(borderPose.toPoint(),lastpose));
        cout<<"FindPassablePoint   num: "<< num<<endl;
        std::cout<<"theta 1: "<<theata<<std::endl;
        std::cout<<"theta 3: "<<borderPose.phi_angle()<<std::endl;
        break;
      }
    }

    if(isfirst) { borderPose.x(minPose.x);borderPose.y(minPose.y);  isfirst = false; }
    if(p_laserDataDeal->isHaveObstacleBetweenPoints(getTolerantFaultMap,borderPose.toPoint(),minPose)){
        if(num<10){
          cout<<"isHaveObstacleBetweenPoints   num: "<< num<<endl;
          return -1;
        }else {
          p_laserDataDeal->getLinesPoint2dSlope(vGetLinePoint,k,b);
           if(k=0) k=0.0001;
           theata =(atan(1/k)*180/3.14);
           if(fabs(robotPose.phi_angle())>90){
             if(theata>0) theata -=180;
             else if(theata<0) theata +=180;
           }
          borderPose.phi_angle(getTwoPointAngleInMap(borderPose.toPoint(),lastpose));
          cout<<"FindPassablePoint   num: "<< num<<endl;
          std::cout<<"theta 1: "<<theata<<std::endl;
          std::cout<<"theta 3: "<<borderPose.phi_angle()<<std::endl;
          break;
        }
    }
    lastpose = minPose;
  }

   std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();   /*计算该函数所需时间*/
   double deltT = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
   cout<<"get slope speed time:"<<deltT<<" seconds. "<<endl;

   return 0;
}


int parkingClean::getBorderPointRealTime(cv::Mat&srcImage,const double& robotSize,const CPose2D&robotPose,
                                         std::vector<cv::Point2d>& vMotionPoint){

  if(srcImage.channels()==3)      /**如果地图为三通道转化为但通道**/
   cv::cvtColor(srcImage,srcImage,CV_BGR2GRAY);

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  /*记录该函数运行前的时间*/

  if(srcImage.empty()){   /*地图为空时报错，返回 -1*/
    std::cerr<<"输入图像为空"<<std::endl;
    return -1;
  }
  if(srcImage.at<uchar>(robotPose.y(),robotPose.x())<150){
      std::cout<<" "<<srcImage.at<uchar>(robotPose.y(),robotPose.x())<<std::endl;
      cout<<"机器人在障碍里"<<endl;
      return -1;
  }

  cv::Mat workMap,borderMap,borderMap_,tolerantFaultMap;  /*定义一些地图信息   求异或时的地图*/
  p_laserDataDeal->expendGridMapRelyNum(srcImage,workMap,robotSize); /*膨胀机器人宽度*/
  p_laserDataDeal->expendGridMapRelyNum(srcImage,borderMap_,robotSize+1);  /*膨胀机器人宽度+1*/
  p_laserDataDeal->expendGridMapRelyNum(borderMap_,borderMap,1);/*求边界范围*/
  p_laserDataDeal->expendGridMapRelyNum(borderMap_,tolerantFaultMap,3);/*膨胀容错范围*/

  cv::Mat getBorderMap,getTolerantFaultMap;         /*定义一些地图信息  异或地图*/
  p_laserDataDeal->twoMaps_xor(borderMap_,borderMap,getBorderMap,150); /*异或得到边界地图*/
  p_laserDataDeal->twoMaps_xor(workMap,tolerantFaultMap,getTolerantFaultMap,150); /*异或得到边界范围地图*/
  /**对得到的异或边界地图取返**/
  cv::Mat xorTolerantFaultMap = getBorderMap.clone();
  xorTolerantFaultMap = ~xorTolerantFaultMap;

  /**得到距离机器人45-90度最近的点**/
  std::vector<double> laser; std::vector<CPose2D> laserPose;
  p_laserDataDeal->getLasterData(xorTolerantFaultMap,robotPose,30,90,laser,laserPose);
  if(laser.size()<1||laserPose.size()<1) {
    cout<<"laser data empty   error: "<<endl;
    return -1;}
  cv::Point2d minPose; double len;
  CPose2D borderPose;
  if(p_laserDataDeal->getLasterDataMin(laser,laserPose,len,borderPose)==-1){
    cout<<"getLasterDataMin   error: "<<endl;
    return -1;}

  minPose = borderPose.toPoint();
    vMotionPoint.push_back(minPose);
    mvTestPoint.push_back(minPose);
     std::cout<<"borderPose: " << borderPose<<std::endl;
     std::cout<<"minPose: " << minPose<<std::endl;
  while(ros::ok()){

    if(mvTestPoint.size()>100){     /**记录所找的点**/
         mvTestPoint.erase(mvTestPoint.begin(),mvTestPoint.begin()+50);
     }

    if(minPose.x!=mvTestPoint[mvTestPoint.size()-1].x||minPose.y!=mvTestPoint[mvTestPoint.size()-1].y)
       mvTestPoint.push_back(minPose);

     int status = p_laserDataDeal->FindPassablePoint(getBorderMap,robotPose.phi_angle(),minPose,minPose);
     if((status==-3)){/**查找边界失败**/
        int i=2;
         std::cout<<"退回重新查找" <<std::endl;
        while(ros::ok()){   /*退回查找该*/
          if((mvTestPoint.size()-i)>0)
               minPose = mvTestPoint[mvTestPoint.size()-i];
          else{
            std::cout<<" 检查了N个点后仍然未找到可通行点" <<std::endl;
            return -1;
          }
          if((p_laserDataDeal->FindPassablePoint(getBorderMap,robotPose.phi_angle(),minPose,minPose))==0) break;
          i++;
          if(i>50) return -1;
        }
        if(minPose.x!=vMotionPoint[vMotionPoint.size()-1].x||minPose.y!=vMotionPoint[vMotionPoint.size()-1].y){
           vMotionPoint.push_back(minPose);
            std::cout<<"-3: " << minPose<<std::endl;
        }
     }else if((status==-2)){/**查找边界失败**/

       if(minPose.x!=vMotionPoint[vMotionPoint.size()-1].x||minPose.y!=vMotionPoint[vMotionPoint.size()-1].y){
          vMotionPoint.push_back(minPose);
           std::cout<<"-2: " << minPose<<std::endl;
       }
       break;
     }

     if(p_laserDataDeal->isHaveObstacleBetweenPoints(getTolerantFaultMap,vMotionPoint[vMotionPoint.size()-1],minPose)){
       if(minPose.x!=vMotionPoint[vMotionPoint.size()-1].x||minPose.y!=vMotionPoint[vMotionPoint.size()-1].y){
          vMotionPoint.push_back(minPose);
          std::cout<<" : " << minPose<<std::endl;
       }
     }
      if(vMotionPoint.size()>=(8)) break;
  }

   std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();   /*计算该函数所需时间*/
   double deltT = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
   cout<<"get slope speed time:"<<deltT<<" seconds. "<<endl;

   return 0;
}
int parkingClean::getBorderPointRealTimeInLine( cv::Mat&srcImage,const double&robotSize,const CPose2D&robotPose,std::vector<cv::Point2d>&vMotionPoint){

  //  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  /*记录该函数运行前的时间*/
  /* 地图如果是三通道转化为单通道*/
  if(srcImage.channels()==3)      /**如果地图为三通道转化为但通道**/
   cv::cvtColor(srcImage,srcImage,CV_BGR2GRAY);

   /*地图为空 时报错，返回 -1*/
  if(srcImage.empty()){
    std::cerr<<"输入图像为空"<<std::endl;
    return -1;
  }
  /**将地图四周变为黑色防止出现目标点在边界到达不了的情形**/
  for(int i=0;i<srcImage.rows;i++){
    srcImage.at<uchar>(i,0) = 0;
    srcImage.at<uchar>(i,srcImage.cols-1) = 0;
  }
  for(int i=0;i<srcImage.cols-1;i++){
    srcImage.at<uchar>(0,i) = 0;
    srcImage.at<uchar>(srcImage.rows-1,i) = 0;
  }

  cv::Mat workMap,borderMap,borderMap_,tolerantFaultMap;  /*定义一些地图信息   求异或时的地图*/
  p_laserDataDeal->expendGridMapRelyNum(srcImage,workMap,robotSize); /*膨胀机器人宽度*/
  p_laserDataDeal->expendGridMapRelyNum(srcImage,borderMap_,robotSize+1);  /*膨胀机器人宽度+1*/
  p_laserDataDeal->expendGridMapRelyNum(borderMap_,borderMap,1);/*求边界范围*/
  p_laserDataDeal->expendGridMapRelyNum(borderMap_,tolerantFaultMap,3);/*膨胀容错范围*/
  cv::Mat getBorderMap,getTolerantFaultMap;         /*定义一些地图信息  异或地图*/
  p_laserDataDeal->twoMaps_xor(borderMap_,borderMap,getBorderMap,150); /*异或得到边界地图*/
  p_laserDataDeal->twoMaps_xor(workMap,tolerantFaultMap,getTolerantFaultMap,150); /*异或得到边界范围地图*/
  cout<<" init robotPose: "<< robotPose  <<endl;

  /**边界查询点**/
  cv::Point2d minPose;

  if(getBorderMap.at<uchar>(robotPose.y(),robotPose.x())>150){    /**该店wei**/
    cout<<"gai dian yan se: "<<(int)getBorderMap.at<uchar>(robotPose.y(),robotPose.x())<<endl;
    minPose = robotPose.toPoint();
  }else{
    /**对得到的异或边界地图取返**/
    cv::Mat xorTolerantFaultMap = getBorderMap.clone();
    xorTolerantFaultMap = ~xorTolerantFaultMap;
     /**得到距离机器人45-90度最近的点**/
    std::vector<double> vlaserLen; std::vector<CPose2D> vlaserPose;
    double len_r,len_l;    CPose2D borderPose_r,borderPose_l;
    /**求取左边距离障碍最近的点**/
    p_laserDataDeal->getLasterData(xorTolerantFaultMap,robotPose,-160,0,vlaserLen,vlaserPose);
    if(vlaserLen.size()<1||vlaserPose.size()<1) {
       cout<<"laser data empty   error: "<<endl;
     return -1;
    }
    if(p_laserDataDeal->getLasterDataMin(vlaserLen,vlaserPose,len_r,borderPose_r)==-1){
      cout<<"getLasterDataMin R   error: "<<endl;
      return -1;}
    vlaserLen.clear(); vlaserPose.clear();
    /**求取右边距离障碍最近的点**/
    p_laserDataDeal->getLasterData(xorTolerantFaultMap,robotPose,0,160,vlaserLen,vlaserPose);
    if(vlaserLen.size()<1||vlaserPose.size()<1) {
       cout<<"laser data empty   error: "<<endl;
     return -1;
    }
    if(p_laserDataDeal->getLasterDataMin(vlaserLen,vlaserPose,len_l,borderPose_l)==-1){
      cout<<"getLasterDataMin R   error: "<<endl;
      return -1;}
     cout<<"len_r:"<<len_r<<" len_l"<<len_l<<endl;
     cout<<" borderPose_r:"<<borderPose_r<< "borderPose_l:"<< borderPose_l<<endl;

     (len_r>len_l)? minPose = borderPose_l.toPoint() : minPose = borderPose_r.toPoint();

  }
    cout<<"minPose: "<< minPose  <<endl;

    if(minPose.x==0&&minPose.y==0) {
       cout<<"程序出问题了"<<endl;
      return -1;}
    mvTestPoint.push_back(minPose);
    vMotionPoint.push_back(minPose);
  while(ros::ok()){

    if(mvTestPoint.size()>100){     /**记录所找的点**/
         mvTestPoint.erase(mvTestPoint.begin(),mvTestPoint.begin()+50);
     }

    if(minPose.x!=mvTestPoint[mvTestPoint.size()-1].x||minPose.y!=mvTestPoint[mvTestPoint.size()-1].y)
       mvTestPoint.push_back(minPose);
//    cout<<"minPose: "<< minPose  <<endl;
      int status = p_laserDataDeal->FindPassablePoint(getBorderMap,robotPose.phi_angle(),minPose,minPose);
//          cout<<"status: "<< status  <<endl;
     if((status==-3)){/**查找边界失败**/
        int i=2;
         std::cout<<"退回重新查找" <<std::endl;
        while(ros::ok()){   /*退回查找该*/
          if((mvTestPoint.size()-i)>0)
               minPose = mvTestPoint[mvTestPoint.size()-i];
          else{
            std::cout<<" 检查了N个点后仍然未找到可通行点" <<std::endl;
            return -1;}
          if(!p_laserDataDeal->FindPassablePoint(getBorderMap,robotPose.phi_angle(),minPose,minPose)) break;
          i++;
          if(i>50) return -1;
        }
        if(minPose.x!=vMotionPoint[vMotionPoint.size()-1].x||minPose.y!=vMotionPoint[vMotionPoint.size()-1].y)
           vMotionPoint.push_back(minPose);
     }else if((status==-2)){/**查找边界失败**/

       if(minPose.x!=vMotionPoint[vMotionPoint.size()-1].x||minPose.y!=vMotionPoint[vMotionPoint.size()-1].y)
          vMotionPoint.push_back(minPose);
       break;
     }

     if(p_laserDataDeal->isHaveObstacleBetweenPoints(getTolerantFaultMap,vMotionPoint[vMotionPoint.size()-1],minPose)){
       if(minPose.x!=vMotionPoint[vMotionPoint.size()-1].x||minPose.y!=vMotionPoint[vMotionPoint.size()-1].y)
          vMotionPoint.push_back(minPose);
     }
      if(vMotionPoint.size()>=(5)) break;
  }

//   std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();   /*计算该函数所需时间*/
//   double deltT = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
//   cout<<"get slope speed time:"<<deltT<<" seconds. "<<endl;

   return 0;
}





int parkingClean::getBorderPoint( cv::Mat&srcImage,const double& robotSize,const double& roadDirection,
                             bool& isFirstTest,const cv::Point2d& initPoint, std::vector<cv::Point2d>&vTestMotionPoint){

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  /*记录该函数运行前的时间*/
  if(srcImage.empty()){   /*地图为空时报错，返回 -1*/
    std::cerr<<"输入图像为空"<<std::endl;
    return -1;
  }
  cv::Mat workMap,borderMap,borderMap_,tolerantFaultMap;  /*定义一些地图信息   求异或时的地图*/
  p_laserDataDeal->expendGridMapRelyNum(srcImage,workMap,robotSize); /*膨胀机器人宽度*/
  p_laserDataDeal->expendGridMapRelyNum(srcImage,borderMap_,robotSize+1);  /*膨胀机器人宽度+1*/
  p_laserDataDeal->expendGridMapRelyNum(borderMap_,borderMap,1);/*求边界范围*/
  p_laserDataDeal->expendGridMapRelyNum(borderMap_,tolerantFaultMap,2);/*膨胀容错范围*/

  cv::Mat getBorderMap,getTolerantFaultMap;         /*定义一些地图信息  异或地图*/
  p_laserDataDeal->twoMaps_xor(borderMap_,borderMap,getBorderMap,150); /*异或得到边界地图*/
  p_laserDataDeal->twoMaps_xor(workMap,tolerantFaultMap,getTolerantFaultMap,150); /*异或得到边界范围地图*/

  cv::Point2d testPoint;    //定义一个点

  if(isFirstTest){      //第一次时
    testPoint = initPoint;
   if(p_laserDataDeal->FindPassablePoint(getBorderMap,roadDirection,testPoint,testPoint) == -1){
        return -1;
   }
    mvTestPoint.clear();

    mvTestPoint.push_back(testPoint);
    vTestMotionPoint.push_back(testPoint);
    cout<<" isFirstTest:"<<testPoint<<endl;
    isFirstTest = false;
  }else{
    testPoint = vTestMotionPoint[vTestMotionPoint.size()-1];
  } //

  for(int i=0;i<mvTestPoint.size();i++){
    getBorderMap.at<uchar>(mvTestPoint[i].y,mvTestPoint[i].x) =0; //将经过的点变成已通过点
  }

  int iii = vTestMotionPoint.size();    //记录该容器内点的个数
  while(ros::ok()){

    if(mvTestPoint.size()>200){
        mvTestPoint.erase(mvTestPoint.begin()+5,mvTestPoint.begin()+100);
    }

      if(testPoint.x!=mvTestPoint[mvTestPoint.size()-1].x
         ||testPoint.y!=mvTestPoint[mvTestPoint.size()-1].y)
         mvTestPoint.push_back(testPoint);


    if(p_laserDataDeal->FindPassablePoint(getBorderMap,roadDirection,testPoint,testPoint)){
       int i=1;
       while(ros::ok()){
         if(sqrt((mvTestPoint[mvTestPoint.size()-1].x-mvTestPoint[1].x)*(mvTestPoint[mvTestPoint.size()-1].x-mvTestPoint[1].x)+
               (mvTestPoint[mvTestPoint.size()-1].y-mvTestPoint[1].y)*(mvTestPoint[mvTestPoint.size()-1].y-mvTestPoint[1].y))<3){
           if(vTestMotionPoint[vTestMotionPoint.size()-1].x!=mvTestPoint[mvTestPoint.size()-1].x
              ||vTestMotionPoint[vTestMotionPoint.size()-1].y!=mvTestPoint[mvTestPoint.size()-1].y)
           vTestMotionPoint.push_back(mvTestPoint[mvTestPoint.size()-1]);
           return -1;
         }
         testPoint = mvTestPoint[mvTestPoint.size()-i];
         if(!p_laserDataDeal->FindPassablePoint(getBorderMap,roadDirection,testPoint,testPoint)) break;
         i++;
         if(i>100) {
           if(vTestMotionPoint[vTestMotionPoint.size()-1].x!=mvTestPoint[mvTestPoint.size()-1].x
              ||vTestMotionPoint[vTestMotionPoint.size()-1].y!=mvTestPoint[mvTestPoint.size()-1].y)
           vTestMotionPoint.push_back(mvTestPoint[mvTestPoint.size()-1]); return -1;
         }
       }
       vTestMotionPoint.push_back(testPoint);
       break;
    }

    if(p_laserDataDeal->isHaveObstacleBetweenPoints(getTolerantFaultMap,vTestMotionPoint[vTestMotionPoint.size()-1],testPoint)){
        vTestMotionPoint.push_back(testPoint);}
    if(vTestMotionPoint.size()>=(2+iii)) break;
  }
   std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
   double deltT = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
   cout<<"deltT:"<<deltT<<endl;
   return 0;
}

int test(){

//  if(mModes==Enum::emIdle){      /*空闲*/
//    if(isFirstIntoMode){       /*第一次进入 模式*/
//      stopRobot();             /*机器人STOP*/
//      isFirstIntoMode = false;
//    }
//    p_dataSave->getRobotHandleControl(mRControls);
//    switch (mRControls) {     /*空闲模式下  前后左右*/
//    case Enum::rLeft:
//      rotateRobot(-1.0);
//      break;
//    case Enum::rRight:
//      rotateRobot(1.0);
//      break;
//    case Enum::rFront:
//      forwardRobot(0.3);
//      break;
//    case Enum::rRear:
//      forwardRobot(-0.3);
//      break;
//    case Enum::rStop:
//      stopRobot();
//      break;
//    default:
//      break;
//    }
//  }else if(mModes==Enum::emWork){   /*工作*/
//    /*  当三个方向距离障碍都为0时，机器人还未获得正确的激光数据*/
//    if((mDist_F==0)&&(mDist_L==0)&&(mDist_R==0)){
//      continue;
//    }
//    if(isFirstIntoMode){
//        mStates = Enum::esWalkByWall;
//        isFirstIntoState =true;
//        isFirstIntoMode = false;
//    }
//    switch (mStates) {
//    case Enum::esWalkByWall:{
//      /*沿墙走*/
//      mWorkWalkByWall();
//      cout<<" 机器人正在沿墙走 "<<endl;
//     } break;
//    case Enum::esWalkNearWall:{
//      /*靠墙走*/
//      mWorkWalkNearWall();
//      cout<<" 机器人正在靠近墙走"<<endl;
//    } break;
//    case Enum::esParkingClean:{
//      /*清扫车位*/
//      mWorkParkingClean();
//      cout<<" 机器人正在清扫工作"<<endl;
//    }break;
//    default:
//      break;
//    }
//  }else if(mModes==Enum::emTest){
//  }
}


bool parkingClean::moveToGoalAsWorld(const double&Goal_x,const double&Goal_y,const tf::Quaternion& GoalOrigen){

//   ac = new MoveBaseClient("move_base",true);
   cout<<" Goal_x"<<Goal_x<<" Goal_y"<<Goal_y<<endl;
   //wait for the action server to come up
   while(!ac->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "odom";
   goal.target_pose.header.stamp = ros::Time::now();
   /* moving towards the goal*/
   goal.target_pose.pose.position.x =  Goal_x;
   goal.target_pose.pose.position.y =  Goal_y;
   goal.target_pose.pose.position.z = 0.0;
   goal.target_pose.pose.orientation.x = GoalOrigen.x();
   goal.target_pose.pose.orientation.y =  GoalOrigen.y();
   goal.target_pose.pose.orientation.z =  GoalOrigen.z();
   goal.target_pose.pose.orientation.w =  GoalOrigen.w();

   ROS_INFO("Sending goal location ...");
   ac->sendGoal(goal,boost::bind( &parkingClean::doneCbAsWorld,this, _1, _2),
               MoveBaseClient::SimpleActiveCallback(),
               MoveBaseClient::SimpleFeedbackCallback());
  cout<<"moveToGoalAsWorld"<<endl;
}
bool parkingClean::moveToGoalAsMap(const CPose2D&robotPose){          //新的点到点3-29
    p_dataSave->setNowMotionPose(robotPose );

}

bool parkingClean::moveToGoalAsWorld(const double&Goal_x,const double&Goal_y,const double&Angle){
  tf::Quaternion GoalOrigen;
  cout<<"发送目标点"<<CPose2D(Goal_x,Goal_y,Angle*3.14/180)   <<endl;
  GoalOrigen.setRPY(0,0,Angle*3.14/180);
  if(moveToGoalAsWorld(Goal_x,Goal_y,GoalOrigen))
    return true;
  else return false;
}
bool parkingClean::moveToGoalAsWorld(const CPose2D&robotPose){
  tf::Quaternion GoalOrigen;
  cout<<"发送目标点"<<robotPose<<endl;
  GoalOrigen.setRPY(0,0,robotPose.phi());
  if(moveToGoalAsWorld(robotPose.x(),robotPose.y(),GoalOrigen))
    return true;
  else return false;
}
bool parkingClean::moveToGoalAsRobot(const CPose2D&robotPose){
  tf::Quaternion GoalOrigen;
  cout<<"发送目标点"<<robotPose<<endl;
  GoalOrigen.setRPY(0,0,robotPose.phi());
  if(moveToGoalAsRobot(robotPose.x(),robotPose.y(),GoalOrigen))
    return true;
  else return false;
}
bool parkingClean::moveToGoalAsRobot(const double&Goal_x,const double&Goal_y,const double&Angle){
  tf::Quaternion q;
  cout<<"发送目标点"<<CPose2D(Goal_x,Goal_y,Angle*3.14/180)   <<endl;
  q.setRPY(0,0,Angle);
  if(moveToGoalAsRobot(Goal_x,Goal_y,q))
    return true;
  else false;
}

bool parkingClean::moveToGoalAsRobot(const double&Goal_x,const double&Goal_y,const tf::Quaternion& GoalOrigen){

//   MoveBaseClient ac("move_base", true);
   //wait for the action server to come up
   while(!ac->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "base_link";
   goal.target_pose.header.stamp = ros::Time::now();
   /* moving towards the goal*/
   goal.target_pose.pose.position.x =  Goal_x;
   goal.target_pose.pose.position.y =  Goal_y;
   goal.target_pose.pose.position.z = 0.0;
   goal.target_pose.pose.orientation.x = GoalOrigen.x();
   goal.target_pose.pose.orientation.y =  GoalOrigen.y();
   goal.target_pose.pose.orientation.z =  GoalOrigen.z();
   goal.target_pose.pose.orientation.w =  GoalOrigen.w();

   ROS_INFO("Sending goal location ...");
   ac->sendGoal(goal,boost::bind( &parkingClean::doneCbAsRobot,this, _1, _2),
               MoveBaseClient::SimpleActiveCallback(),
               MoveBaseClient::SimpleFeedbackCallback());

}

void parkingClean::doneCbAsRobot(const actionlib::SimpleClientGoalState& state,
                           const move_base_msgs::MoveBaseResultConstPtr& result)
{
     cout<<" _____________________________ "<<endl;
    ROS_INFO("RESULT %s",state.toString().c_str());
   // boost::unique_lock<boost::mutex> lock(wp_mutex_);
    switch (state.state_) {
    case actionlib::SimpleClientGoalState::ABORTED:
    {
      ROS_INFO("NavigationManager::moveBaseResultCallback: ABORTED");
    }
      break;
    case actionlib::SimpleClientGoalState::SUCCEEDED:
    {
      ROS_INFO("NavigationManager::moveBaseResultCallback: SUCCEEDED");
      p_dataSave->setRobotIfArriveDestPose(true);
    }
      break;
    default:
      break;
    }
 }

void parkingClean::doneCbAsWorld(const actionlib::SimpleClientGoalState& state,
                           const move_base_msgs::MoveBaseResultConstPtr& result)
{
    cout<<" _____________________________ "<<endl;
   ROS_INFO("RESULT %s",state.toString().c_str());
  // boost::unique_lock<boost::mutex> lock(wp_mutex_);
   switch (state.state_) {
   case actionlib::SimpleClientGoalState::ABORTED:
   {
     ROS_INFO("NavigationManager::moveBaseResultCallback: ABORTED");
   }
     break;
   case actionlib::SimpleClientGoalState::SUCCEEDED:
   {
     ROS_INFO("NavigationManager::moveBaseResultCallback: SUCCEEDED");
     p_dataSave->setRobotIfArriveDestPose(true);
   }
     break;
   default:
     break;
   }

 }
int parkingClean::pathPointTransformCoordinateMapToRobot(const CPose2D &robotPose, std::vector<cv::Point2d> &vpathPoint,
                                                         std::vector<CPose2D> &vrealPathPose){
  if(vpathPoint.size()<1){
    std::cerr<<"路径点为空"<<std::endl;
    return -1;
  }
  vrealPathPose.clear();
  vrealPathPose.reserve(vpathPoint.size());
  double phi;
  for(int i=0;i<vpathPoint.size();i++){
    if(i==0)
      phi = getTwoPointAngleInMap(robotPose.toPoint(),vpathPoint[i]);
    else
      phi = getTwoPointAngleInMap(vpathPoint[i-1],vpathPoint[i]);
    phi = phi*3.14/180;
    CPose2D pathpose(vpathPoint[i].x,vpathPoint[i].y,phi);
    pathpose = p_dataSave->coordinateFromMapToRobot(pathpose);
    vrealPathPose.push_back(pathpose);
  }
   return 0;
}

void parkingClean::run_process5(){


  while(ros::ok()){
  bool status = true;

  std::vector<CPose2D> vMotionPose;
  p_dataSave->getMotionRobotPose(vMotionPose);

    cout<<"vMotionPose.size"<<vMotionPose.size() << endl;

    if(vMotionPose.size()>0){
       cout<<"有目标点"<<endl;

      if(status){
        status =  moveToGoalAsWorld(vMotionPose[0]);
      cout<<"status =  "<<(status?"true":"false")<< endl;
        cout<<"到达目标点"<<endl;
        cout<<"----------------------------"<<endl;
        vMotionPose.erase(vMotionPose.begin());
        p_dataSave->setMotionRobotPose(vMotionPose);
      }else{
        cout<<"到达目标点失败"<<endl;
      }
    }else{
      cout<<"没有目标点"<<endl;
    }
//    sleep(10);

//        p_dataSave->getSystermMode(testMode);

//      if(testMode==Enum::emTest){
//        cout<<"start send a global point "<<endl;
//        moveToGoalAsRobot(1,0);

//            cout<<"finish "<<endl;
//            sleep(10);
//        firstTestModeToIdle = true;
//      }else if(testMode==Enum::emIdle){

//        if(firstTestModeToIdle){
//         moveToGoalAsRobot(0,0);
//          firstTestModeToIdle = false;
//        }
//      }
  }

}

void parkingClean::run_process2(){  //

  vector<cv::Point2d> vTest;
  vector<tf::Quaternion> vQtest;
  cv::Point2d  test1(0.850,-10.398);
  vTest.push_back(test1);
  tf::Quaternion GoalOrigen1(0,0, 0.795   , 0.606 );
  vQtest.push_back(GoalOrigen1);

  cv::Point2d  test2(1.733,-2.025);
  vTest.push_back(test2);
  tf::Quaternion GoalOrigen2(0,0, 0.113   , 0.994  );
  vQtest.push_back(GoalOrigen2);

  cv::Point2d  test3(1.601,-1.492);
  vTest.push_back(test3);
  tf::Quaternion GoalOrigen3(0,0,  0.989  ,  -0.148 );
  vQtest.push_back(GoalOrigen3);

  cv::Point2d  test4(-0.917,-2.046);
  vTest.push_back(test4);
  tf::Quaternion GoalOrigen4(0,0, 0.995   , -0.1  );
  vQtest.push_back(GoalOrigen4);

  cv::Point2d  test5(-0.897,-1.530);
  vTest.push_back(test5);
  tf::Quaternion GoalOrigen5(0,0,  0.115  , 0.993  );
  vQtest.push_back(GoalOrigen5);

  cv::Point2d  test6(1.249,-1.192);
  vTest.push_back(test6);
  tf::Quaternion GoalOrigen6(0,0,  0.077  ,  0.997 );
  vQtest.push_back(GoalOrigen6);

  cv::Point2d  test7(-1.371,-0.459);
  vTest.push_back(test7);
  tf::Quaternion GoalOrigen7(0,0,  0.065  , 0.998  );
  vQtest.push_back(GoalOrigen7);

  cv::Point2d  test8(1.387,-0.023);
  vTest.push_back(test8);
  tf::Quaternion GoalOrigen8(0,0,  0.065  ,  0.998 );
  vQtest.push_back(GoalOrigen8);

  cv::Point2d  test9(1.330,0.501);
  vTest.push_back(test9);
  tf::Quaternion GoalOrigen9(0,0,  0.996  , -0.086 );
  vQtest.push_back(GoalOrigen9);

  cv::Point2d  test10(-1.966,0.059);
  vTest.push_back(test10);
  tf::Quaternion GoalOrigen10(0,0,  0.997  , -0.082  );
  vQtest.push_back(GoalOrigen10);
   p_dataSave->setRobotIfArriveDestPose(true);

  while(ros::ok()){

   bool status = true;
   p_dataSave->getRobotIfArriveDestPose(status);
    cout<<"有几个目标点： "<<(vTest.size()+1)<<endl;
    cout<<"vTest[0] "<<vTest[0]<<endl;
    if(status){
           cout<<"到达一个目标点 "<<endl;
      if(vTest.empty()){
        cout<<"所有点都走完了"<<endl;
      }else{
        cout<<"到达一个目标点 "<<endl;
        status =  moveToGoalAsWorld(vTest[0].x,vTest[0].y,vQtest[0]);
        vTest.erase(vTest.begin());
        vQtest.erase(vQtest.begin());
         p_dataSave->setRobotIfArriveDestPose(false);
      }



  }
    usleep(1000*1000);
  }
}


void hehe(){

  vector<cv::Point2d> vTest;

  cv::Point2d  test1(1,0);
//  tf::Quaternion GoalOrigen1();
  vTest.push_back(test1);

  cv::Point2d  test2(0,0);
  vTest.push_back(test2);

  cv::Point2d  test3(2,0);
  vTest.push_back(test3);

  cv::Point2d  test4(0,0);
  vTest.push_back(test4);

  cv::Point2d  test5(2,2);
  vTest.push_back(test5);

  cv::Point2d  test6(0,0);
  vTest.push_back(test6);

//   p_dataSave->setRobotIfArriveDestPose(true);

  while(ros::ok()){

   bool status = true;
//   p_dataSave->getRobotIfArriveDestPose(status);
    cout<<"有几个目标点： "<<(vTest.size()+1)<<endl;
    cout<<"vTest[0] "<<vTest[0]<<endl;
    if(status){
           cout<<"到达一个目标点 "<<endl;
      if(vTest.empty()){
        cout<<"所有点都走完了"<<endl;
      }else{
        cout<<"到达一个目标点 "<<endl;
//        status =  moveToGoalAsWorld(vTest[0]);
        vTest.erase(vTest.begin());
//         p_dataSave->setRobotIfArriveDestPose(false);
      }



  }
    usleep(1000*1000);
  }


}
void parkingClean::run_process4(){

  bool ifArriveDstPose = true;
  bool lastIfArriveDstPose = true;

  bool firstTest4 = true;
  CPose2D startPose4, dstPose4;

  while(ros::ok()){

//      mIdleModeDealThing();

      cv::Mat srcImgae;
      p_dataSave->getGlobalMapMsg(srcImgae);  /*订阅全局地图*/
      if(srcImgae.empty()) continue; /*全局地图为空 返回*/

        /**为了显示机器人位姿是否正确**/
        CPose2D robotposeMap,robotposeReal;
        p_dataSave->getRobotPose(robotposeReal);    /*获取机器人实际位姿*/
  //            cout<<"实际机器人的位姿 /m:"<<robotposeReal<<endl;
        p_dataSave->getRobotPoseMap(robotposeMap);  /*获取机器人地图位姿*/


        p_dataSave->getRobotIfArriveDestPose(ifArriveDstPose);

        if((ifArriveDstPose==false)&&(lastIfArriveDstPose==true)){
          firstTest4 = true;
        }
if(ifArriveDstPose==false){
        /**目标点获取*/
       if(firstTest4){
          startPose4 = robotposeMap;
          p_dataSave->getNowMotionPose(dstPose4);

          double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPose4.toPoint());
          double distAngle =  robotToDstAngle - robotposeMap.phi_angle();
          if(distAngle>180) distAngle -=360;
          if(distAngle<-180) distAngle +=360;
//          cout<<"distAngle::"<<distAngle<<endl;
          moveRobot(__speedRobot,distAngle);
          firstTest = false;
       }
//       cout<<"地图机器人位姿: /grid:"<<robotposeMap<<endl;
//       cout<<"目标点位姿: /grid:"<<dstPose4<<endl;

       double dist_robotToDst = dstPose4.twoCPose2DDis(robotposeMap);
       double dist_robotToStart = startPose4.twoCPose2DDis(robotposeMap);
       double dist_startToDst = startPose4.twoCPose2DDis(dstPose4);

       if(dist_robotToDst<3){
          stopRobot();
//          cout<<"arrive dst"<<endl;
          p_dataSave->setRobotIfArriveDestPose(1);

       }else{

          double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPose4.toPoint());
          double dist_Dist, distAngle =  robotToDstAngle - robotposeMap.phi_angle();
          if(distAngle>180) distAngle -=360;
          if(distAngle<-180) distAngle +=360;

//          p_laserDataDeal->getPointToLineMsgs(startPose4.toPoint(),dstPose4.toPoint(),robotposeMap.toPoint(),dist_Dist);

     //                 double standard_angle,standard_dist,standard_speed;
     //                 standard_angle = 90 - 10*dist_robotToDst;
     //                 if(standard_angle<10) standard_angle =10;
     //                 standard_dist = 3 +

           if(fabs(distAngle)>5/*||dist_robotToStart>(dist_startToDst+4)*/){
//              cout<<"差值大于5度需要矫正"<<endl;
               moveRobot(0.1,distAngle);

           }

//                cout<<"robotToDstAngle::"<<robotToDstAngle<<endl;
//                cout<<"robotposeMap.phi_angle()::"<<robotposeMap.phi_angle()<<endl;
//                cout<<"distAngle::"<<distAngle<<endl;
//                cout<<"dist_Dist::"<<dist_Dist<<endl;
//                cout<<"-------------------"<<endl;

       }
}

       lastIfArriveDstPose = ifArriveDstPose;

//#define IMSHOW1
  #ifdef IMSHOW1
        {
        cv::Mat workImage; std::vector<cv::Point2d> mVRobotPose;
        p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
        p_laserDataDeal->drawArrow(workImage,robotposeMap,8);/**显示机器人位姿和原图**/

        p_dataSave->getAllRobotPoseInMap(mVRobotPose);
        p_laserDataDeal->pointAndPointLinePaintInMap(workImage,mVRobotPose);
        cv::namedWindow("test1");
        cv::imshow("test1",workImage);
        cv::waitKey(20);
        }
  #endif


  }

}

void parkingClean::run_process3(){
  Struct::sDists threeDist;   //三个方向的距离
  CPose2D robotPoseT(340,450,-3.14/2.5);
  CPose2D frontPose,leftPose,righPose;

  while(ros::ok()){

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//    std::vector<CPose2D> lasterPose;
//    std::vector<double> lasterData;

    cv::Mat srcImage = cv::imread("/home/mayun16/Desktop/testMap.png",0);   //原始地图
    cv::Mat ssrcImage =srcImage.clone();
    cv::Mat ss (100,100,CV_32FC1);
    ss.data;
    cv::Mat workMap,borderMap,tolerantFaultMap1,tolerantFaultMap;
    p_laserDataDeal->expendGridMapRelyNum(srcImage,workMap,6); //膨胀机器人宽度 +1
    p_laserDataDeal->expendGridMapRelyNum(workMap,borderMap,1);//求边界范围
    p_laserDataDeal->expendGridMapRelyNum(srcImage,tolerantFaultMap1,5); //膨胀机器人宽度
    p_laserDataDeal->expendGridMapRelyNum(workMap,tolerantFaultMap,2);//膨胀容错范围
    cv::Mat getBorderMap,getTolerantFaultMap;
    p_laserDataDeal->twoMaps_xor(workMap,borderMap,getBorderMap,150);
    p_laserDataDeal->twoMaps_xor(tolerantFaultMap1,tolerantFaultMap,getTolerantFaultMap,150);

    p_laserDataDeal->getThreeDirectorPoint(workMap,robotPoseT,frontPose,leftPose,righPose);

    int NumTest = 0;
    cv::Point2d testPoint = righPose.toPoint();
    std::vector<cv::Point2d> vTestPoint;
    std::vector<cv::Point2d> vTestMotionPoint;
    while (NumTest<100000) {
     if(p_laserDataDeal->FindPassablePoint(getBorderMap,robotPoseT.phi_angle(),testPoint,testPoint)) {

       break;}
      vTestPoint.push_back(testPoint);
      NumTest++;
    }
     cout<<"vTestPoint.size:"<<vTestPoint.size()<<endl;
      vTestMotionPoint.push_back(robotPoseT.toPoint());
      cv::Point2d start = vTestPoint[0];
      vTestMotionPoint.push_back(start);
      for(int i=1;i<vTestPoint.size();i++){
        if(p_laserDataDeal->isHaveObstacleBetweenPoints(getTolerantFaultMap,start,vTestPoint[i])){
            start = vTestPoint[i];
            vTestMotionPoint.push_back(start);
        }
      }
      vTestMotionPoint.push_back(vTestPoint[vTestPoint.size()-1]);
    cout<<"vTestMotionPoint.size:"<<vTestMotionPoint.size()<<endl;
       p_laserDataDeal->pointAndPointLinePaintInMap(getTolerantFaultMap,vTestPoint);
    p_laserDataDeal->drawArrow(srcImage,robotPoseT,10);
    p_laserDataDeal->pointAndPointLinePaintInMap(srcImage,vTestPoint);
    p_laserDataDeal->drawArrow(ssrcImage,robotPoseT,10);
    p_laserDataDeal->pointAndPointLinePaintInMap(ssrcImage,vTestMotionPoint);
    cv::namedWindow("test1");
    cv::imshow("test1",srcImage);
    cv::waitKey(100);
    cv::imshow("test2",workMap);
    cv::waitKey(100);

    cv::imshow("test3",getTolerantFaultMap);
//    cv::imwrite("/home/mayun16/Desktop/test444.jpg",getTolerantFaultMap);
//    cv::waitKey(100);
//    cv::imshow("test4",ssrcImage);
//    cv::waitKey(100);

    cout<<"-------------------------"<<endl;
  }
}


/*用于图像的翻转 0:不翻转  1:上下翻转  2:左右翻转  3:上下左右翻转*/
void parkingClean::mMapTransform(const cv::Mat &srcImage, cv::Mat &dstImage,const int& type){

  cv::Mat map_x,map_y;
  dstImage.create(srcImage.size(),srcImage.type());
  map_x.create(srcImage.size(),CV_32FC1);
  map_y.create(srcImage.size(),CV_32FC1);

  for(int j=0;j<srcImage.rows;j++)
    for(int i=0;i<srcImage.cols;i++){
      if(type==0){
        map_x.at<float>(j,i) = static_cast<float>(i);
        map_y.at<float>(j,i) = static_cast<float>(j);
      }else if(type==1){
        map_x.at<float>(j,i) = static_cast<float>(i);
        map_y.at<float>(j,i) = static_cast<float>(srcImage.rows-1-j);
      }else if(type==2){
        map_x.at<float>(j,i) = static_cast<float>(srcImage.cols-1-i);
        map_y.at<float>(j,i) = static_cast<float>(j);
      }else if(type==3){
        map_x.at<float>(j,i) = static_cast<float>(srcImage.cols-1-i);
        map_y.at<float>(j,i) = static_cast<float>(srcImage.rows-1-j);
      }
    }
  cv::remap(srcImage,dstImage,map_x,map_y,CV_INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));

}

void parkingClean::mIdleModeDealThing(){

   Enum::rControls rControls = Enum::rStop;  //机器人的控制命令
   Struct::sDists threeDistLaser;   //三个方向的距离
   CPose2D robotposeReal;
   p_dataSave->getRobotPose(robotposeReal);    /*获取机器人实际位姿*/
   cout<<"实际机器人的位姿 /m:"<<robotposeReal<<endl;
   p_dataSave->getThreeDirectDist(threeDistLaser);     /*得到激光数据*/
   if(isFirstIntoMode){       /*第一次进入 模式*/
     stopRobot();             /*机器人STOP*/
     mRControls = Enum::rStop;
     isFirstIntoMode = false;
   }
   p_dataSave->getRobotHandleControl(rControls);    /**获取系统的控制**/
   /***键盘控制   w上 s下 a左 d右 q左传  e右转  ' '暂停***/
   switch (rControls) {     /*空闲模式下 手动控制*/
   case Enum::rLeft:
//#define BAOHU
#ifndef BAOHU
        levelRobot(0.2);
#else
     if(threeDistLaser.L>0.3){
       levelRobot(0.3);
       cout<<"rLeft "<<endl;
     }else{
       stopRobot();
       cout<<"左方距离过小停止 "<<endl;
     }
#endif
     break;
   case Enum::rRight:
#ifndef BAOHU
       levelRobot(-0.2);
#else
     if(threeDistLaser.R>0.3){
       levelRobot(-0.3);
       cout<<"rRight "<<endl;
     }else{
       stopRobot();
       cout<<"右方距离过小停止 "<<endl;
     }
#endif
     break;
   case Enum::rFront:
#ifndef BAOHU
       forwardRobot(0.2);
#else
     if(threeDistLaser.F>0.3){
       forwardRobot(0.3);
       cout<<"rFront "<<endl;
     }else{
       stopRobot();
       cout<<"前方距离过小停止 "<<endl;
     }
#endif
     break;
   case Enum::rRear:
     forwardRobot(-0.2);
      cout<<"rRear "<<endl;
     break;
   case Enum::rStop:
      cout<<"rStop "<<endl;
     stopRobot();
     break;
   case Enum::rLeftRotate:   //左传
     rotateRobot(0.2);
      cout<<"rLeftRotate "<<endl;
     break;
   case Enum::rRightRotate:   //右传
     rotateRobot(-0.2);
      cout<<"rRightRotate "<<endl;
     break;
   default:
     break;
   }

}

void parkingClean::mWorkModeDealThing(){


  if(isFirstIntoMode){
      mStates = Enum::esWalkByWall;    /*沿墙走*/
      isFirstIntoState =true;
      isFirstIntoMode = false;
  }

  switch (mStates) {
  case Enum::esWalkByWall:{          /*沿墙走*/
    cout<<" 机器人正在沿墙走 "<<endl;
    mWorkWalkByWall();

   } break;

  case Enum::esWalkNearWall:{    /*靠墙走*/
    cout<<" 机器人正在靠近墙走"<<endl;
    mWorkWalkNearWall();

  } break;
  case Enum::esParkingClean:{    /*清扫车位*/
    cout<<" 机器人正在清扫工作"<<endl;
    mWorkParkingClean2();
  }break;
  default:
    break;
  }
}

void parkingClean::mWorkWalkByWall(){

  /** 增加 获取边检的函数 **/
  if(isFirstIntoState){
    mSStates = Enum::essGoStraight;   /*直走*/
    isFirstIntoSState = true;
    isFirstIntoState = false;
  }

  switch (mSStates) {
  case Enum::essGoStraight:{       /*直走*/
     cout<<"直行"<<endl;
    if(isFirstIntoSState){
      mLastRobotPose =mRobotPose;
      forwardRobot(mSpeedLiner);
      isFirstIntoSState = false;
    }

    if(mDist_F<mFMinDist){
      cout<<"前方有障碍物左传"<<endl;
       stopRobot();
      mSStates = Enum::essTurnLeft;
      isFirstIntoSState = true;
    }else if(mDist_R>=mRMaxDist){
      cout<<"满足清扫条件"<<endl;
       stopRobot();
      mStates = Enum::esParkingClean;
      isFirstIntoState = true;
    }

  }break;
  case Enum::essTurnLeft:{    //左传
    cout<<"沿边行走  左传90"<<endl;
   if(isFirstIntoSState){
     mDestTheata = mRobotPose.phi_angle()-90;        /*计算目标角度*/
     mLastRobotPose = mRobotPose;
     mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
     if(mstaticTheata>180) mstaticTheata -=360;
     if(mstaticTheata<-180) mstaticTheata +=360;
     if(mstaticTheata>0){
       rotateRobot(-mSpeedAngule);    //
     }else{
       rotateRobot(mSpeedAngule);     //左传
     }
     isFirstIntoSState = false;
   }
     mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
     if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
     if(mRealTimeTheata>=fabs(mstaticTheata)-2){
        stopRobot();
         usleep(muSleepTimes);
         mSStates = Enum::essGoStraight;
         isFirstIntoSState = true;
     }
   }break;
  default:
    break;
  }

}
void parkingClean::mWorkParkingClean(){

  if(isFirstIntoState){

    /*第一次进入清扫车位*/
    mSStates = Enum::essCleanForwardCarDist;
    mRoadPose = mRobotPose;   //第一次进入清扫时道路方向
    mFirstIntoCleanRobotPose = mRobotPose;    //第一次进入清扫时机器人状态
    mFirstIntoCleanRobotPose.phi_angle(0);
    mIsParkingCleanNow = true;
    isFirstIntoSState = true;
    isFirstIntoState = false;
  }
  /*在清扫状态下 的小状态*/
  switch (mSStates) {
  case Enum::essCleanTurnIntoCar:{     /*右传90 方向进入车库*/
     cout<<"清扫步骤  右传90 方向进入车库"<<endl;
     if(isFirstIntoSState){
       if(mDist_R<=mRMaxDist){
          mStates = Enum::esWalkByWall;
          cout<<"前进 机器人宽度后右侧距离不满足清扫条件 "<<endl;
          isFirstIntoState = true;
          stopRobot();
          break;
       }
       mDestTheata = mFirstIntoCleanRobotPose.phi_angle()-90;        /*计算目标角度*/
       mLastRobotPose = mRobotPose;
       mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
       if(mstaticTheata>180) mstaticTheata -=360;
       if(mstaticTheata<-180) mstaticTheata +=360;
       if(mstaticTheata<0){
         rotateRobot(-mSpeedAngule);    //右转
       }else{
         rotateRobot(mSpeedAngule);     //左传
       }
       isFirstIntoSState = false;
     }
     mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
     if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
     if(mRealTimeTheata>=fabs(mstaticTheata)){
        stopRobot();
        usleep(muSleepTimes);
        mSStates = Enum::essCleanGoIntoCar;
        isFirstIntoSState = true;
     }
  } break;
  case Enum::essCleanGoIntoCar:{  // 前进:进入车位，直到碰到障碍物  记录行走的距离
    cout<<"清扫步骤  进入车位，直到碰到障碍物 "<<endl;
    if(isFirstIntoSState){
      mDestTheata = mFirstIntoCleanRobotPose.phi_angle()-90;
      mLastRobotPose =mRobotPose;
      forwardRobot(mSpeedLiner);
      isFirstIntoSState = false;
    }
     // rotateRobot(PIDControl(mDestTheata,mRobotPose.phi_angle()));

    if(mDist_F>mFMinDist){    /*正常进入*/
      cout<<"清扫前方无障碍"<<endl;
    }else if(mDist_F<=mFMinDist){  /*进入时碰上障碍物*/
      mRobotCleanDistLen = mRobotPose.twoCPose2DDis(mLastRobotPose);
      if(mDist_L>(mFCleanDist)){    /*判断本次是否可以清扫*/
          if(mDist_L>(2*mFCleanDist)){    /**判断下此是否可以清扫*/
             cout<<"清扫前方有障碍  还可下次清扫"<<endl;
              mIsParkingCleanNext = true;
          }else {
             cout<<"清扫前方有障碍  不可下次清扫"<<endl;
              mIsParkingCleanNext = false;
          }
           mIsParkingCleanNow = true;
      }else {
          cout<<"清扫前方有障碍  本次原路"<<endl;
          mIsParkingCleanNext = false;
          mIsParkingCleanNow = false;
      }
      stopRobot();
      usleep(muSleepTimes);
      mSStates = Enum::essCleanTurnApeakCar;
      isFirstIntoSState = true;

    }
  }break;
  case Enum::essCleanTurnApeakCar:{     /* 左传90，与道路方向水平*/
     cout<<"清扫步骤  左传90，与道路方向水平 "<<endl;
    if(isFirstIntoSState){
      mDestTheata = mFirstIntoCleanRobotPose.phi_angle();        /*计算目标角度*/
      mLastRobotPose = mRobotPose;
      mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
      if(mstaticTheata>180) mstaticTheata -=360;
      if(mstaticTheata<-180) mstaticTheata +=360;
      if(mstaticTheata<0){
        rotateRobot(-mSpeedAngule);    //右转
      }else{
        rotateRobot(mSpeedAngule);     //左传
      }
      isFirstIntoSState = false;
    }
    mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
    if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
    if(mRealTimeTheata>=fabs(mstaticTheata)-2){
       stopRobot();
       usleep(muSleepTimes);
       mSStates = Enum::essCleanGoCleanDist;
       isFirstIntoSState = true;
    }
  }break;
  case Enum::essCleanGoCleanDist:{  /*前进一个清扫距离*/
     cout<<"清扫步骤  前进一个清扫距离 "<<endl;
    if(isFirstIntoSState){
      mDestTheata = mFirstIntoCleanRobotPose.phi_angle();
      if(!mIsParkingCleanNow||mDist_F<=mFMinDist){    /**本次不可清扫原路返回**/
        cout<<"本次不可清扫 旋转90度 "<<endl;
        mSStates = Enum::essCleanTurnBacktoCar;
        isFirstIntoSState = true;
        break;
      }

      mLastRobotPose =mRobotPose;
      forwardRobot(mSpeedLiner);
      isFirstIntoSState = false;

    }
    if(mRobotPose.twoCPose2DDis(mLastRobotPose)>mFCleanDist){
      stopRobot();
       usleep(muSleepTimes);
      mSStates = Enum::essCleanTurnBacktoCar;
      isFirstIntoSState = true;
    }
     // rotateRobot(PIDControl(mDestTheata,mRobotPose.phi_angle()));
  }break;
  case Enum::essCleanTurnBacktoCar:{     /* 左传90 方向朝出口方向*/
    cout<<"清扫步骤  左传90 方向朝出口方向 "<<endl;
    if(isFirstIntoSState){
      mDestTheata = mFirstIntoCleanRobotPose.phi_angle()+90;        /*计算目标角度*/
      mLastRobotPose = mRobotPose;
      mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
      if(mstaticTheata>180) mstaticTheata -=360;
      if(mstaticTheata<-180) mstaticTheata +=360;
      if(mstaticTheata<0){
        rotateRobot(-mSpeedAngule);    //
      }else{
        rotateRobot(mSpeedAngule);     //左传
      }

      isFirstIntoSState = false;
    }
    mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
    cout<<" mRealTimeTheata:"<<mRealTimeTheata<<endl;
    if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
    if(mRealTimeTheata>=fabs(mstaticTheata)-2){
      if(mDist_L>(mFCleanDist)){
          cout<<"清扫前方有障碍  还可下次清扫"<<endl;
           mIsParkingCleanNext = true;
      }else {
          cout<<"清扫前方有障碍  不可下次清扫"<<endl;
          mIsParkingCleanNext = false;
      }
      stopRobot();
       usleep(muSleepTimes);
       mSStates = Enum::essCleanGoBacktoCar;
       isFirstIntoSState = true;
    }
  }break;
  case Enum::essCleanGoBacktoCar:{  /*清扫车位反向出*/
    cout<<"清扫步骤  清扫车位反向出 "<<endl;
    if(isFirstIntoSState){
      mDestTheata = mFirstIntoCleanRobotPose.phi_angle()+90;
      mLastRobotPose =mRobotPose;
      forwardRobot(mSpeedLiner);
      isFirstIntoSState = false;
    }

    if(mIsParkingCleanNext){
      if(mRobotPose.twoCPose2DDis(mLastRobotPose)>mRobotCleanDistLen){
        stopRobot();
        mSStates = Enum::essCleanTurntoStart;
        isFirstIntoSState = true;
      }

    }else{
      if(mRobotPose.twoCPose2DDis(mLastRobotPose)>mRobotCleanDistLen&&(mDist_R>2*mFCleanDist)){
        stopRobot();
        mSStates = Enum::essCleanTurntoStart;
        isFirstIntoSState = true;
      }
      if(mDist_F<mFMinDist){    /*下次不可进入且前方有障碍*/
         cout<<"清扫步骤  清扫车位反向出 "<<endl;
         mStates = Enum::esWalkByWall;
         isFirstIntoState = true;
      }
      if(mDist_R>2*mFCleanDist){
        stopRobot();
         usleep(muSleepTimes);
        mSStates = Enum::essCleanTurntoStart;
        isFirstIntoSState = true;
      }

    }

     // rotateRobot(PIDControl(mDestTheata,mRobotPose.phi_angle()));
  }break;
  case Enum::essCleanTurntoStart:{   /*右转90  转正*/
       cout<<"清扫步骤  到达原来机器人初始位置 "<<endl;
    if(isFirstIntoSState){
      mDestTheata = mFirstIntoCleanRobotPose.phi_angle();        /*计算目标角度*/
      mLastRobotPose = mRobotPose;

      mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
      if(mstaticTheata>180) mstaticTheata -=360;
      if(mstaticTheata<-180) mstaticTheata +=360;
      if(mstaticTheata<0){
        rotateRobot(-mSpeedAngule);    //
      }else{
        rotateRobot(mSpeedAngule);     //左传
      }

      isFirstIntoSState = false;
    }
    mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
    cout<<" mRealTimeTheata:"<<mRealTimeTheata<<endl;
    if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
    if(mRealTimeTheata>=fabs(mstaticTheata)-2){
      mSStates = Enum::essCleanForwardCarDist;
      isFirstIntoSState = true;
       stopRobot();
        usleep(muSleepTimes);
    }
  }break;
  case Enum::essCleanForwardCarDist:{  /* 机器人已经出来到达路面时  前进一个清扫距离*/
     cout<<"清扫步骤  前进一个清扫距离 "<<endl;
    if(isFirstIntoSState){
      mDestTheata = mFirstIntoCleanRobotPose.phi_angle();
      mLastRobotPose =mRobotPose;
      forwardRobot(mSpeedLiner);
      isFirstIntoSState = false;
    }

    if(mRobotPose.twoCPose2DDis(mLastRobotPose)>mFCleanDist){
      if(mIsParkingCleanNext){
        mSStates = Enum::essCleanTurnIntoCar;
        isFirstIntoSState = true;
        cout<<"清扫步骤  再次清扫 "<<endl;
      }else{
        if(mIsParkingCleanNow){
          mSStates = Enum::essCleanTurnIntoCar;
          isFirstIntoSState = true;
          cout<<"清扫步骤  本次可清扫 "<<endl;
        }else{
          mStates = Enum::esWalkByWall;
          isFirstIntoState = true;
          cout<<"退出清扫  沿墙走起 "<<endl;
        }
      }
      stopRobot();
       usleep(muSleepTimes);
    }
     // rotateRobot(PIDControl(mDestTheata,mRobotPose.phi_angle()));
  }break;
  default:{
    // essCleanTurnIntoCar,  essCleanGoIntoCar,essCleanTurnApeakCar,essCleanGoCleanDist,
    // essCleanTurnBacktoCar,  essCleanGoBacktoCar,  essCleanTurntoStart,
  }break;
  }
}


void parkingClean::mWorkParkingClean2(){

    if(isFirstIntoState){

      /*第一次进入清扫车位*/
      mSStates = Enum::essCleanForwardCarDist;
      mRoadPose = mRobotPose;   //第一次进入清扫时道路方向
      mFirstIntoCleanRobotPose = mRobotPose;    //第一次进入清扫时机器人状态
      mFirstIntoCleanRobotPose.phi_angle(0);
      mIsParkingCleanNow = true;
      isFirstIntoSState = true;
      isFirstIntoState = false;
    }
    /*在清扫状态下 的小状态*/
    switch (mSStates) {
    case Enum::essCleanTurnIntoCar:{     /*右传90 方向进入车库*/
       cout<<"清扫步骤  右传90 方向进入车库"<<endl;
       if(isFirstIntoSState){
         if(mDist_R<=mRMaxDist){
            mStates = Enum::esWalkByWall;
            cout<<"前进 机器人宽度后右侧距离不满足清扫条件 "<<endl;
            isFirstIntoState = true;
            stopRobot();
            break;
         }
         mDestTheata = mFirstIntoCleanRobotPose.phi_angle()-90;        /*计算目标角度*/
         mLastRobotPose = mRobotPose;
         mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
         if(mstaticTheata>180) mstaticTheata -=360;
         if(mstaticTheata<-180) mstaticTheata +=360;
         if(mstaticTheata<0){
           rotateRobot(-mSpeedAngule);    //右转
         }else{
           rotateRobot(mSpeedAngule);     //左传
         }
         isFirstIntoSState = false;
       }
       mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
       if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
       if(mRealTimeTheata>=fabs(mstaticTheata)){
          stopRobot();
          usleep(muSleepTimes);
          mSStates = Enum::essCleanGoIntoCar;
          isFirstIntoSState = true;
       }
    } break;
    case Enum::essCleanGoIntoCar:{  // 前进:进入车位，直到碰到障碍物  记录行走的距离
      cout<<"清扫步骤  进入车位，直到碰到障碍物 "<<endl;
      if(isFirstIntoSState){
        mDestTheata = mFirstIntoCleanRobotPose.phi_angle()-90;
        mLastRobotPose =mRobotPose;
        forwardRobot(mSpeedLiner);
        isFirstIntoSState = false;
      }
       // rotateRobot(PIDControl(mDestTheata,mRobotPose.phi_angle()));

      if(mDist_F>mFMinDist){    /*正常进入*/
        cout<<"清扫前方无障碍"<<endl;
      }else if(mDist_F<=mFMinDist){  /*进入时碰上障碍物*/
        mRobotCleanDistLen = mRobotPose.twoCPose2DDis(mLastRobotPose);
        if(mDist_L>(mFCleanDist)){    /*判断本次是否可以清扫*/
            if(mDist_L>(2*mFCleanDist)){    /**判断下此是否可以清扫*/
               cout<<"清扫前方有障碍  还可下次清扫"<<endl;
                mIsParkingCleanNext = true;
            }else {
               cout<<"清扫前方有障碍  不可下次清扫"<<endl;
                mIsParkingCleanNext = false;
            }
             mIsParkingCleanNow = true;
        }else {
            cout<<"清扫前方有障碍  本次原路"<<endl;
            mIsParkingCleanNext = false;
            mIsParkingCleanNow = false;
        }
        stopRobot();
        usleep(muSleepTimes);
        mSStates = Enum::essCleanTurnApeakCar;
        isFirstIntoSState = true;

      }
    }break;
    case Enum::essCleanTurnApeakCar:{     /* 左传90，与道路方向水平*/
       cout<<"清扫步骤  左传90，与道路方向水平 "<<endl;
      if(isFirstIntoSState){
        mDestTheata = mFirstIntoCleanRobotPose.phi_angle();        /*计算目标角度*/
        mLastRobotPose = mRobotPose;
        mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
        if(mstaticTheata>180) mstaticTheata -=360;
        if(mstaticTheata<-180) mstaticTheata +=360;
        if(mstaticTheata<0){
          rotateRobot(-mSpeedAngule);    //右转
        }else{
          rotateRobot(mSpeedAngule);     //左传
        }
        isFirstIntoSState = false;
      }
      mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
      if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
      if(mRealTimeTheata>=fabs(mstaticTheata)-2){
         stopRobot();
         usleep(muSleepTimes);
         mSStates = Enum::essCleanGoCleanDist;
         isFirstIntoSState = true;
      }
    }break;
    case Enum::essCleanGoCleanDist:{  /*前进一个清扫距离*/
       cout<<"清扫步骤  前进一个清扫距离 "<<endl;
      if(isFirstIntoSState){
        mDestTheata = mFirstIntoCleanRobotPose.phi_angle();
        if(!mIsParkingCleanNow||mDist_F<=mFMinDist){    /**本次不可清扫原路返回**/
          cout<<"本次不可清扫 旋转90度 "<<endl;
          mSStates = Enum::essCleanTurnBacktoCar;
          isFirstIntoSState = true;
          break;
        }

        mLastRobotPose =mRobotPose;
        forwardRobot(mSpeedLiner);
        isFirstIntoSState = false;

      }
      if(mRobotPose.twoCPose2DDis(mLastRobotPose)>mFCleanDist){
        stopRobot();
         usleep(muSleepTimes);
        mSStates = Enum::essCleanTurnBacktoCar;
        isFirstIntoSState = true;
      }
       // rotateRobot(PIDControl(mDestTheata,mRobotPose.phi_angle()));
    }break;
    case Enum::essCleanTurnBacktoCar:{     /* 左传90 方向朝出口方向*/
      cout<<"清扫步骤  左传90 方向朝出口方向 "<<endl;
      if(isFirstIntoSState){
        mDestTheata = mFirstIntoCleanRobotPose.phi_angle()+90;        /*计算目标角度*/
        mLastRobotPose = mRobotPose;
        mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
        if(mstaticTheata>180) mstaticTheata -=360;
        if(mstaticTheata<-180) mstaticTheata +=360;
        if(mstaticTheata<0){
          rotateRobot(-mSpeedAngule);    //
        }else{
          rotateRobot(mSpeedAngule);     //左传
        }

        isFirstIntoSState = false;
      }
      mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
      cout<<" mRealTimeTheata:"<<mRealTimeTheata<<endl;
      if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
      if(mRealTimeTheata>=fabs(mstaticTheata)-2){
        if(mDist_L>(mFCleanDist)){
            cout<<"清扫前方有障碍  还可下次清扫"<<endl;
             mIsParkingCleanNext = true;
        }else {
            cout<<"清扫前方有障碍  不可下次清扫"<<endl;
            mIsParkingCleanNext = false;
        }
        stopRobot();
         usleep(muSleepTimes);
         mSStates = Enum::essCleanGoBacktoCar;
         isFirstIntoSState = true;
      }
    }break;
    case Enum::essCleanGoBacktoCar:{  /*清扫车位反向出*/
      cout<<"清扫步骤  清扫车位反向出 "<<endl;
      if(isFirstIntoSState){
        mDestTheata = mFirstIntoCleanRobotPose.phi_angle()+90;
        mLastRobotPose =mRobotPose;
        forwardRobot(mSpeedLiner);
        isFirstIntoSState = false;
      }

      if(mIsParkingCleanNext){
        if(mRobotPose.twoCPose2DDis(mLastRobotPose)>mRobotCleanDistLen){
          stopRobot();
          mSStates = Enum::essCleanTurntoStart;
          isFirstIntoSState = true;
        }

      }else{
        if(mRobotPose.twoCPose2DDis(mLastRobotPose)>mRobotCleanDistLen&&(mDist_R>2*mFCleanDist)){
          stopRobot();
          mSStates = Enum::essCleanTurntoStart;
          isFirstIntoSState = true;
        }
        if(mDist_F<mFMinDist){    /*下次不可进入且前方有障碍*/
           cout<<"清扫步骤  清扫车位反向出 "<<endl;
           mStates = Enum::esWalkByWall;
           isFirstIntoState = true;
        }
        if(mDist_R>2*mFCleanDist){
          stopRobot();
           usleep(muSleepTimes);
          mSStates = Enum::essCleanTurntoStart;
          isFirstIntoSState = true;
        }

      }

       // rotateRobot(PIDControl(mDestTheata,mRobotPose.phi_angle()));
    }break;
    case Enum::essCleanTurntoStart:{   /*右转90  转正*/
         cout<<"清扫步骤  到达原来机器人初始位置 "<<endl;
      if(isFirstIntoSState){
        mDestTheata = mFirstIntoCleanRobotPose.phi_angle();        /*计算目标角度*/
        mLastRobotPose = mRobotPose;

        mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
        if(mstaticTheata>180) mstaticTheata -=360;
        if(mstaticTheata<-180) mstaticTheata +=360;
        if(mstaticTheata<0){
          rotateRobot(-mSpeedAngule);    //
        }else{
          rotateRobot(mSpeedAngule);     //左传
        }

        isFirstIntoSState = false;
      }
      mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
      cout<<" mRealTimeTheata:"<<mRealTimeTheata<<endl;
      if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
      if(mRealTimeTheata>=fabs(mstaticTheata)-2){
        mSStates = Enum::essCleanForwardCarDist;
        isFirstIntoSState = true;
         stopRobot();
          usleep(muSleepTimes);
      }
    }break;
    case Enum::essCleanForwardCarDist:{  /* 机器人已经出来到达路面时  前进一个清扫距离*/
       cout<<"清扫步骤  前进一个清扫距离 "<<endl;
      if(isFirstIntoSState){
        mDestTheata = mFirstIntoCleanRobotPose.phi_angle();
        mLastRobotPose =mRobotPose;
        forwardRobot(mSpeedLiner);
        isFirstIntoSState = false;
      }

      if(mRobotPose.twoCPose2DDis(mLastRobotPose)>mFCleanDist){
        if(mIsParkingCleanNext){
          mSStates = Enum::essCleanTurnIntoCar;
          isFirstIntoSState = true;
          cout<<"清扫步骤  再次清扫 "<<endl;
        }else{
          if(mIsParkingCleanNow){
            mSStates = Enum::essCleanTurnIntoCar;
            isFirstIntoSState = true;
            cout<<"清扫步骤  本次可清扫 "<<endl;
          }else{
            mStates = Enum::esWalkByWall;
            isFirstIntoState = true;
            cout<<"退出清扫  沿墙走起 "<<endl;
          }
        }
        stopRobot();
         usleep(muSleepTimes);
      }
       // rotateRobot(PIDControl(mDestTheata,mRobotPose.phi_angle()));
    }break;
    default:{
      // essCleanTurnIntoCar,  essCleanGoIntoCar,essCleanTurnApeakCar,essCleanGoCleanDist,
      // essCleanTurnBacktoCar,  essCleanGoBacktoCar,  essCleanTurntoStart,
    }break;
    }

}


void parkingClean::mWorkWalkNearWall(){

  if(isFirstIntoState){
    mSStates = Enum::essCleanTurnIntoCar;
    isFirstIntoSState = true;
    isFirstIntoState = false;
  }
  switch (mSStates) {
  case Enum::essCleanTurnIntoCar:{    //右转90度
    if(isFirstIntoSState){
      mLastRobotPose = mRobotPose;
    //  p_laserDataDeal->getMapByLasterDataMappingToMap();

      rotateRobot(1.0);
      isFirstIntoSState = false;
    }
    if(fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle())>=90){
       stopRobot();
       mSStates = Enum::essCleanGoIntoCar;
       isFirstIntoSState = true;
    }
  }
    break;
  case Enum::essCleanGoIntoCar:{
    if(isFirstIntoSState){
      mLastRobotPose =mRobotPose;
      forwardRobot(0.2);

      isFirstIntoSState = false;
    }
    if(mDist_F<mFMinDist){
       cout<<"机器人前进到底了"<<endl;
       mDistIntoCar = mRobotPose.twoCPose2DDis(mLastRobotPose);
       stopRobot();
       mSStates = Enum::essCleanTurnApeakCar;
       isFirstIntoSState = true;

       if(mDist_L>mFCleanDist+mRobotSize){

       }else{

       }
    }
  }
    break;
  default:
    break;
  }

}

void parkingClean::mWorkWalkByWall1(){    //沿墙走


}
void parkingClean::mWorkParkingClean1(){    //清扫

}
void parkingClean::mWorkWalkNearWall1(){    //靠近墙

}

#define IMSHOW1
void parkingClean::run_testPointToPoint(){


    while (ros::ok()) {

//       cout<<"-------------------"<<endl;
      p_dataSave->getSystermMode(mModes);   /**获取系统模式**/
      if(mModes==Enum::emIdle){                   /*空闲*/
          cout<<"空闲 "<<endl;
          mIdleModeDealThing();

          firstTest = true;
          m_i = 0;
#ifdef IMSHOW1
          {
          cv::Mat srcImgae;
          p_dataSave->getGlobalMapMsg(srcImgae);  /*订阅全局地图*/
          if(srcImgae.empty()) continue; /*全局地图为空 返回*/
            /**为了显示机器人位姿是否正确**/
            CPose2D robotposeMap,robotposeReal;
            p_dataSave->getRobotPose(robotposeReal);    /*获取机器人实际位姿*/
            cout<<"实际机器人的位姿 /m:"<<robotposeReal<<endl;
            p_dataSave->getRobotPoseMap(robotposeMap);  /*获取机器人地图位姿*/
            cout<<"地图机器人位姿: /grid:"<<robotposeMap<<endl;
            {
            cv::Mat workImage; std::vector<cv::Point2d> mVRobotPose;
            p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
            p_laserDataDeal->drawArrow(workImage,robotposeMap,8);/**显示机器人位姿和原图**/
            p_dataSave->getAllRobotPoseInMap(mVRobotPose);
            p_laserDataDeal->pointAndPointLinePaintInMap(workImage,mVRobotPose);
  //            std::cout<<"mVRobotPose.size(): "<< mVRobotPose.size()<<std::endl;
            cv::namedWindow("test1");
            cv::imshow("test1",workImage);
            cv::waitKey(20);
            }
          }
#endif
      }else if(mModes==Enum::emWork){             /*工作*/
          cout<<"工作 "<<endl;

          cv::Mat srcImgae;   /**初始地图**/
          p_dataSave->getGlobalMapMsg(srcImgae);  /*订阅全局地图*/
          if(srcImgae.empty()) continue; /*全局地图为空 返回*/

            /**为了显示机器人位姿是否正确**/
            CPose2D robotposeMap,robotposeReal;
            p_dataSave->getRobotPose(robotposeReal);    /*获取机器人实际位姿*/
            cout<<"实际机器人的位姿 /m:"<<robotposeReal<<endl;
            p_dataSave->getRobotPoseMap(robotposeMap);  /*获取机器人地图位姿*/
            cout<<"地图机器人位姿: /grid:"<<robotposeMap<<endl;
#ifdef IMSHOW1
             /*为显示障碍膨胀后机器人的位姿*/
            {
            cv::Mat workImage; std::vector<cv::Point2d> mVRobotPose;
            p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
            p_laserDataDeal->drawArrow(workImage,robotposeMap,8);/**显示机器人位姿和原图**/
            p_dataSave->getAllRobotPoseInMap(mVRobotPose);
            p_laserDataDeal->pointAndPointLinePaintInMap(workImage,mVRobotPose);
            cv::namedWindow("test1");
            cv::imshow("test1",workImage);
            cv::waitKey(20);
            }
#endif

          if(isFirstIntoMode){  /*first into work init some state */
              mStates = Enum::esWalkNearWall;    /*靠墙走*/
              isFirstIntoState =true;
              mRoadPose = robotposeReal;
              mIntoRobotPose = robotposeReal;

              vector<cv::Point2d> motionPoint;
              p_dataSave->getMotionRobotPoint(motionPoint);
              motionPoint.push_back(robotposeReal.toPoint());
              p_dataSave->setMotionRobotPoint(motionPoint);

              isFirstIntoMode = false;
          }

          switch (mStates) {

          case Enum::esWalkNearWall:{    /*靠墙走*/
            cout<<" 机器人正在靠近墙走"<<endl;


            if(isFirstIntoState){

              /**求取边界点**/
              std::vector<CPose2D> vrealPathPose;

              std::vector<cv::Point2d> vMotionPoint;
              getBorderPointRealTime(srcImgae,(mRobotSize)/0.05,robotposeMap,vMotionPoint); /**获取边界点**/
              if(vMotionPoint.size()<1){
                cout<<"所求边界点失败"<<endl;
                sleep(1);
                continue;
              }

#ifdef IMSHOW1
              /**显示所找到的点**/
              {
              cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
              p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
              cv::namedWindow("test_");
              cv::imshow("test_",test);
              cv::waitKey(20);
              }
#endif

             /**保存目标点**/
             pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionPoint,vrealPathPose);
              p_dataSave->setRobotIfArriveDestPose(false);
              p_dataSave->setMotionRobotPose(vrealPathPose);

              /**发送目标点**/

             cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//             double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPoint);
//             robotToDstAngle -= mIntoRobotPose.phi_angle();
//              if(robotToDstAngle>180) robotToDstAngle -=360;
//              if(robotToDstAngle<-180) robotToDstAngle +=360;
//             moveRobot(__speedRobot,robotToDstAngle);
             cout<<"dstPoint---------------"<<dstPoint<<endl;
             moveToGoalAsMap(dstPoint);

             vector<cv::Point2d> motionPoint;
             p_dataSave->getMotionRobotPoint(motionPoint);
             motionPoint.push_back(vrealPathPose[0].toPoint());
             p_dataSave->setMotionRobotPoint(motionPoint);

              isFirstIntoState = false;
            }

            /**判断目标是否完成**/
             bool status = false;

//             std::vector<CPose2D> vrealPathPose;
//             p_dataSave->getMotionRobotPose(vrealPathPose);
//             cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//             if(robotposeMap.twoCPose2DDisPoint(dstPoint)<3)
//                 status = true;
             p_dataSave->getRobotIfArriveDestPose(status);

             if(status){
               cout<<" 机器人靠近墙走结束 ----"<<endl;
               mStates = Enum::esWalkByWall;
               isFirstIntoState = true;
               byWallEnd = true;
             }

          } break;
          case Enum::esWalkByWall:{          /*沿墙走*/
            cout<<" 机器人正在沿墙走 "<<endl;

            if(isFirstIntoState){

              CPose2D robotSearch;
              std::vector<CPose2D> vrealPathPose;
              std::vector<cv::Point2d> vMotionPoint;
              p_dataSave->getMotionRobotPose(vrealPathPose);
              if(vrealPathPose.size()>0){
                robotSearch = vrealPathPose[0];
                robotSearch.phi_angle(mRoadPose.phi_angle()+30);
              }else {
                cout<<"沿墙走有误 "<<endl;
                robotSearch = robotposeReal;
                 robotSearch.phi_angle(mRoadPose.phi_angle());
              }

              robotSearch = p_dataSave->coordinateFromRobotToMap(robotSearch);
              if(byWallEnd){
                getBorderPointRealTimeInLine(srcImgae,(mRobotSize)/0.05,robotSearch,vMotionPoint);
              }
//              getBorderPointRealTimeInLine(srcImgae,(mRobotSize)/0.05,robotSearch,vMotionPoint);
              cout<<"vMotionPoint的长度 /m:"<<vMotionPoint.size()<<endl;
#ifdef IMSHOW1
              /**显示一下所找的边界点**/{
                cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
                p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
                cv::namedWindow("test_");
                cv::imshow("test_",test);
                cv::waitKey(20);
              }
#endif
               /**发送目标点**/
              if(vMotionPoint.size()<1){
                cout<<"所求边界点失败"<<endl;
//                vrealPathPose.erase(vrealPathPose.begin());
                if(vrealPathPose.size()>1){
                  vrealPathPose.erase(vrealPathPose.begin());
                }else {
                  byWallEnd = true;
                }

                /**发送目标点**/
               cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//               double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPoint);
//               moveRobot(__speedRobot,robotToDstAngle);

               moveToGoalAsMap(dstPoint);

                p_dataSave->setRobotIfArriveDestPose(false);
                p_dataSave->setMotionRobotPose(vrealPathPose);
              }else{
                cout<<"所求边界成功"<<endl;
                pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionPoint,vrealPathPose);
//                vrealPathPose.erase(vrealPathPose.begin());
                if(vrealPathPose.size()>1){
                  vrealPathPose.erase(vrealPathPose.begin());
                }else {
                  byWallEnd = true;
                }


                /**发送目标点**/
               cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//               double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPoint);
//               moveRobot(__speedRobot,robotToDstAngle);
               moveToGoalAsMap(dstPoint);

                p_dataSave->setRobotIfArriveDestPose(false);
                p_dataSave->setMotionRobotPose(vrealPathPose);
              }

              vector<cv::Point2d> motionPoint;
              p_dataSave->getMotionRobotPoint(motionPoint);
              motionPoint.push_back(vrealPathPose[0].toPoint());
              p_dataSave->setMotionRobotPoint(motionPoint);

              isFirstIntoState = false;
            }

            std::vector<CPose2D> vrealPathPose;
            p_dataSave->getMotionRobotPose(vrealPathPose);
            CPose2D arrivePose = vrealPathPose[0];
            arrivePose.phi_angle(mRoadPose.phi_angle());

            double dist_front,dist_right,dist_left;   /**相对于世界坐标系**/
            cv::Mat workImage;
            p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
            p_laserDataDeal->getThreeDirectorDis(workImage,robotposeMap,dist_front,dist_left,dist_right);

            cout<<"dist_front:"<<dist_front<<" dist_left:"<<dist_left<<" dist_right:"<<dist_right<<endl;

//            sleep(5);


            if(dist_right>=mRMaxDist){

               cout<<"状态完成 满足清扫条件"<<endl;
               stopRobot();
               mStates = Enum::esParkingClean;
               isFirstIntoState = true;
               byWallEnd = true;
             }else {
               mStates = Enum::esWalkByWall;
                isFirstIntoState = true;
             }
           } break;


          case Enum::esParkingClean:{    /*清扫车位*/
            cout<<" 机器人正在清扫工作"<<endl;

             mRobotPose = robotposeReal;
            if(isFirstIntoState){  /*第一次进入清扫车位*/

              mSStates = Enum::essCleanGoIntoCar1;

              std::vector<CPose2D> vrealPathPose;
              p_dataSave->getMotionRobotPose(vrealPathPose);
              if(vrealPathPose.size()>0){
                cout<<"vrealPathPose[0]:"<<vrealPathPose[0]<<endl;
                mFirstIntoCleanRobotPose = vrealPathPose[0];    //第一次进入清扫时机器人状态
                vrealPathPose.erase(vrealPathPose.begin()+1,vrealPathPose.end());
                p_dataSave->setMotionRobotPose(vrealPathPose);
              }else{
                mFirstIntoCleanRobotPose = robotposeReal;;    //第一次进入清扫时机器人状态
              }

              mIsCleanFinish = false;
              mIsParkingCleanNow = true;
              isFirstIntoSState = true;
              isFirstIntoState = false;

            }
            /*在清扫状态下 的小状态*/
            switch (mSStates) {
            case Enum::essCleanGoIntoCar1:{  // 前进:进入车位，直到碰到障碍物  记录行走的距离
              cout<<"清扫步骤  进入车位，直到碰到障碍物 "<<endl;
              if(isFirstIntoSState){
  //              cout<<"------------------------------- "<<endl;
                CPose2D robotSearch;
                std::vector<CPose2D> vrealPathPose ;
                std::vector<cv::Point2d> vMotionPoint;
                p_dataSave->getMotionRobotPose(vrealPathPose);
                if(vrealPathPose.size()>0){
                   cout<<"进入车位  有初始点"<<endl;
                   robotSearch = vrealPathPose[0];
                }else{
                  cout<<"进入车位  无初始点"<<endl;
                  robotSearch = robotposeReal;
                }
                double roadAngle = mRoadPose.phi_angle()-80;
                if(roadAngle>180) roadAngle -=360;
                if(roadAngle<-180) roadAngle +=360;
                robotSearch.phi_angle(roadAngle);
                cout<<"进入车位 查找边界点: "<< robotSearch<<endl;
                robotSearch = p_dataSave->coordinateFromRobotToMap(robotSearch);  /*转换到地图坐标系*/

                getBorderPointRealTimeInLine(srcImgae,((mRobotSize))/0.05,robotSearch,vMotionPoint);
#ifdef IMSHOW1
                {   /*显示所求的边界点*/
                cout<<"vMotionPoint的长度22 :"<<vMotionPoint.size()<<endl;
                cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
                p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
                cv::namedWindow("test_");
                cv::imshow("test_",test);
                cv::waitKey(20);
                }
#endif
                if(vMotionPoint.size()<1){
                   cout<<"所求边界点失败"<<endl;
                  if(vrealPathPose.size()>1){   /*所求边界点失败  但是上次成功*/
                     vrealPathPose.erase(vrealPathPose.begin());

                     /**发送目标点**/
                    cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//                    double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPoint);
//                    robotToDstAngle -= mIntoRobotPose.phi_angle();
//                     if(robotToDstAngle>180) robotToDstAngle -=360;
//                     if(robotToDstAngle<-180) robotToDstAngle +=360;
//                    moveRobot(__speedRobot,robotToDstAngle);
                    moveToGoalAsMap(dstPoint);

                     p_dataSave->setRobotIfArriveDestPose(false);
                     p_dataSave->setMotionRobotPose(vrealPathPose);
                  }else{
                    cout<<"error" <<endl;
                    continue;
                  }
                }else{
                  cout<<"所求边界成功"<<endl;
                  pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionPoint,vrealPathPose);
                  vrealPathPose.erase(vrealPathPose.begin());
                  if(vrealPathPose.size()>0){

                      /**发送目标点**/
                     cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//                     double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPoint);
//                     robotToDstAngle -= mIntoRobotPose.phi_angle();
//                      if(robotToDstAngle>180) robotToDstAngle -=360;
//                      if(robotToDstAngle<-180) robotToDstAngle +=360;
//                     moveRobot(__speedRobot,robotToDstAngle);
                     moveToGoalAsMap(dstPoint);

                    p_dataSave->setRobotIfArriveDestPose(false);
                    p_dataSave->setMotionRobotPose(vrealPathPose);
                  }
                }
                isFirstIntoSState = false;
              }
//                cout<<"__________________"<<endl;
//                cout<<robotposeReal<<"++++++++++"<<mFirstIntoCleanRobotPose<<endl;
//                cout<<"robotposeReal.twoCPose2DDis(mFirstIntoCleanRobotPose)"<<robotposeReal.twoCPose2DDis(mFirstIntoCleanRobotPose)<<endl;
              if(robotposeReal.twoCPose2DDis(mFirstIntoCleanRobotPose)>2){

                mEndIntoCleanPose =robotposeReal.toPoint();
                stopRobot();
               mSStates = Enum::essCleanNormalClean;
               isFirstIntoSState = true;
               cout<<"前进两米到头了"<<endl;
               sleep(1);
               break;
              }

              bool status = false;

              std::vector<CPose2D> vrealPathPose;
              p_dataSave->getMotionRobotPose(vrealPathPose);
              cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//              if(robotposeMap.twoCPose2DDisPoint(dstPoint)<3)
//                  status = true;
              p_dataSave->getRobotIfArriveDestPose(status);

              CPose2D arrivePose = vrealPathPose[0];
              arrivePose.phi_angle(mRoadPose.phi_angle());

              if(status){

                  double dist_front,dist_right,dist_left;   /**相对于世界坐标系**/
                  cv::Mat workImage;
                  p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
                  p_laserDataDeal->getThreeDirectorDis(workImage,arrivePose,dist_front,dist_left,dist_right);

                  cout<<"dist_front:"<<dist_front<<" dist_left:"<<dist_left<<" dist_right:"<<dist_right<<endl;

                if(dist_right<=mFMinDist){
                   cout<<"到头了"<<endl;

                   if(dist_front<=mFMinDist){        /****/
                     cout<<"右侧距离太近原路返回 "<<endl;
                     stopRobot();
                     mSStates = Enum::essCleanGoBacktoCar1;
                     isFirstIntoSState = true;
                   }else{
                      cout<<"正常清扫 "<<endl;
                      std::vector<CPose2D> vrealPathPose ;
                     p_dataSave->getMotionRobotPose(vrealPathPose);
                      mEndIntoCleanPose = cv::Point2d(vrealPathPose[0].x(),vrealPathPose[0].y());
                      vrealPathPose.clear();
                      p_dataSave->setMotionRobotPose(vrealPathPose);
                     mSStates = Enum::essCleanNormalClean;
                     isFirstIntoSState = true;
                   }

                }
                else {

                  std::vector<CPose2D> vrealPathPose ;
                  p_dataSave->getMotionRobotPose(vrealPathPose);

                  if(vrealPathPose.size()>1){

                      cout<<"vrealPathPose"<<vrealPathPose[0].twoCPose2DDis(vrealPathPose[1])<<endl;
                    if((vrealPathPose[0].twoCPose2DDis(vrealPathPose[1])>1)){

                      cout<<"还没走完 继续行走"<<endl;
                      stopRobot();
                      mSStates = Enum::essCleanGoIntoCar1;
                      isFirstIntoSState = true;
                    }else{
                      cout<<" 还没走到头 正常清扫 "<<endl;
                      std::vector<CPose2D> vrealPathPose ;
                     p_dataSave->getMotionRobotPose(vrealPathPose);
                      mEndIntoCleanPose = cv::Point2d(vrealPathPose[0].x(),vrealPathPose[0].y());
                      vrealPathPose.clear();
                      p_dataSave->setMotionRobotPose(vrealPathPose);
                     mSStates = Enum::essCleanNormalClean;
                     isFirstIntoSState = true;
                    }


                  }else{      /*边界点就一个时*/
                    cout<<"正常清扫 "<<endl;
                    std::vector<CPose2D> vrealPathPose ;
                   p_dataSave->getMotionRobotPose(vrealPathPose);
                    mEndIntoCleanPose = cv::Point2d(vrealPathPose[0].x(),vrealPathPose[0].y());
                    vrealPathPose.clear();
                    p_dataSave->setMotionRobotPose(vrealPathPose);
                   mSStates = Enum::essCleanNormalClean;
                   isFirstIntoSState = true;
                  }

                }
              }

            }break;
            case Enum::essCleanNormalClean:{     /* 正常清扫便利*/
               cout<<"清扫步骤  走一个前进距离 "<<endl;
               if(isFirstIntoSState){
                 CPose2D start,end;
                 end = mEndIntoCleanPose;
                 start = mFirstIntoCleanRobotPose;
                 end = p_dataSave ->coordinateFromRobotToMap(end);
                 start = p_dataSave ->coordinateFromRobotToMap(start);

                 end.phi_angle(mRoadPose.phi_angle());
                 start.phi_angle(mRoadPose.phi_angle());

                   cv::Mat workImage;
                   p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,(mRobotSize)/0.05);   /*膨胀机器人宽度得到工作地图*/
                   CPose2D end11,end21,start11,start21;
                  end11 = p_laserDataDeal->PoseInFrontOfRobot(workImage,end,4);
                  start11 = p_laserDataDeal->PoseInFrontOfRobot(workImage,start,4);
                  end21 = p_laserDataDeal->PoseInFrontOfRobot(workImage,end,8);
                  start21 = p_laserDataDeal->PoseInFrontOfRobot(workImage,start,8);


                   std::vector<cv::Point2d>vMotionDestPose;
                   cv::Point2d end1(end11.toPoint()),end2(end21.toPoint()),start1(start11.toPoint()),start2(start21.toPoint());

                   if(workImage.at<uchar>(end1.y,end1.x)>150)
                      vMotionDestPose.push_back(end1);
                   else { cout<<"1 错误"<<endl; mIsCleanFinish = true;}
                   if(workImage.at<uchar>(start1.y,start1.x)>150)
                      vMotionDestPose.push_back(cv::Point2d(start1.x,start1.y));
                   else { cout<<"2 错误"<<endl;  mIsCleanFinish = true;}
                   if(workImage.at<uchar>(start2.y,start2.x)>150)
                      vMotionDestPose.push_back(cv::Point2d(start2.x,start2.y));
                   else { cout<<"3 错误"<<endl; mIsCleanFinish = true;}
                   if(workImage.at<uchar>(end2.y,end2.x)>150)
                      vMotionDestPose.push_back(cv::Point2d(end2.x,end2.y));
                   else {
                       vMotionDestPose.push_back(end1);
                       cout<<"4 错误"<<endl; mIsCleanFinish = true;
                   }




//                  mTraTimes++;
//                   if(mTraTemplate==1){                       // 1 正常
//                     cout<<"1 正常"<<endl;
//                     if(workImage.at<uchar>(end.y,end.x+8)>150)
//                        vMotionDestPose.push_back(cv::Point2d(end.x+8,end.y));
//                     else {
//                       mTraTemplate = 2;
//                       //vMotionDestPose.push_back(cv::Point2d(end.x,end.y));
//                        cout<<"2 梯形"<<endl;
//                     }
//                     if(workImage.at<uchar>(start.y,start.x+8)>150||mTraTimes<=1)
//                        vMotionDestPose.push_back(cv::Point2d(start.x+8,start.y));
//                     else {
//                       mTraTemplate = 3;
//                       vMotionDestPose.push_back(cv::Point2d(start.x,start.y));
//                        cout<<"3 楔形"<<endl;

//                     }
//                     if(workImage.at<uchar>(start.y,start.x+16)>150)
//                        vMotionDestPose.push_back(cv::Point2d(start.x+16,start.y));
//                     else {
//                       mTraTemplate = 3;
//                      // vMotionDestPose.push_back(cv::Point2d(end.x+16,end.y));
//                        cout<<"3 楔形"<<endl;
//                     }
//                     if(workImage.at<uchar>(end.y,end.x+16)>150)
//                        vMotionDestPose.push_back(cv::Point2d(end.x+16,end.y));
//                     else {
//                       mTraTemplate = 2;
//                       vMotionDestPose.push_back(cv::Point2d(end.x+8,end.y));
//                       cout<<"2 梯形"<<endl;
//                     }
//                   }
//                   else if(mTraTemplate==2){                        //2  梯形
//                     if((p_laserDataDeal->isHaveObstacleBetweenPoints(workImage,cv::Point2d (start.x,start.y),cv::Point2d (end.x,end.y)))){
//                       mSStates = Enum::essCleanGoBacktoCar1;
//                       mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;
//                       break;
//                     }

//                     cout<<" 梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形"<<endl;
//  //                   if(workImage.at<uchar>(end.x,end.y)>150){
//  //                     vMotionDestPose.push_back(cv::Point2d(end.x,end.y));
//  //                   }else{
//  //                     cout<<"定位不准/失败"<<endl;
//  //                     vMotionDestPose.push_back(cv::Point2d(end.x-4,end.y));
//  //                   }
//                     if(workImage.at<uchar>(start.y,start.x+8)>150)
//                        vMotionDestPose.push_back(cv::Point2d(start.x+8,start.y));
//                     else { cout<<" 错误"<<endl;  mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;}
//                     if(workImage.at<uchar>(start.y,start.x+16)>150){
//                       vMotionDestPose.push_back(cv::Point2d(start.x+16,start.y));
//                       if(workImage.at<uchar>(end.x,end.y)>150){
//                         vMotionDestPose.push_back(cv::Point2d(end.x,end.y));
//                       }else{
//                         cout<<"定位不准/失败"<<endl;
//                         vMotionDestPose.push_back(cv::Point2d(end.x-4,end.y));
//                       }

//                     }
//                     else {
//                        cout<<" 错误"<<endl; mIsCleanFinish = true;
//                       // vMotionDestPose.push_back(cv::Point2d(end.x,end.y));
//                        mTraTemplate = 1;mTraTimes = 0;
//                     }

//                   }else if(mTraTemplate==3){                     //3  楔形
//                     if(p_laserDataDeal->isHaveObstacleBetweenPoints(workImage,cv::Point2d (start.x,start.y),cv::Point2d (end.x,end.y))){
//                       mSStates = Enum::essCleanGoBacktoCar1;
//                       mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;
//                       break;
//                     }

//                     cout<<" 楔形楔形楔形楔形楔形楔形楔形楔形楔形楔形楔形楔形"<<endl;
//                     if(workImage.at<uchar>(end.y,end.x+8)>150)
//                       vMotionDestPose.push_back(cv::Point2d(end.x+8,end.y));
//                     else { cout<<" 错误"<<endl;  mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;}
//                     if(workImage.at<uchar>(end.y,end.x+16)>150){
//                       vMotionDestPose.push_back(cv::Point2d(end.x+16,end.y));
//                     }else { cout<<" 错误"<<endl;  mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;}
//                     if(workImage.at<uchar>(end.y,end.x+24)>150){
//                       vMotionDestPose.push_back(cv::Point2d(start.x,start.y));

//                     }else { cout<<" 错误"<<endl;  mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;}

//                   }



                   if(vMotionDestPose.empty()){

                     mSStates = Enum::essCleanGoBacktoCar1;
                     isFirstIntoSState = true;

                   }else{
#ifdef IMSHOW1
                         {   /*显示所求的边界点*/
                             cout<<"vvMotionDestPose:"<<vMotionDestPose.size()<<endl;
                             cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
                             p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionDestPose);
                             cv::namedWindow("test_");
                             cv::imshow("test_",test);
                             cv::waitKey(20);
                         }
#endif
                         std::vector<CPose2D> vrealPathPose ;
                         pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionDestPose,vrealPathPose);
                         p_dataSave->setMotionRobotPose(vrealPathPose);
                         mSStates = Enum::essCleanNormalStart;
                         isFirstIntoSState = true;
                   }
               }

            }break;
            case Enum::essCleanGoBacktoCar1:{     /* 反向出*/
              cout<<"清扫步骤  清扫车位反向出 "<<endl;

              if(isFirstIntoSState){

                  std::vector<CPose2D> vrealPathPose;
                  p_dataSave->getMotionRobotPose(vrealPathPose);
                  CPose2D arrivePose = vrealPathPose[0];
                  arrivePose.phi_angle(mRoadPose.phi_angle());

                  double dist_front,dist_right,dist_left;   /**相对于世界坐标系**/
                  cv::Mat workImage;
                  p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
                  p_laserDataDeal->getThreeDirectorDis(workImage,arrivePose,dist_front,dist_left,dist_right);

                  cout<<"dist_front:"<<dist_front<<" dist_left:"<<dist_left<<" dist_right:"<<dist_right<<endl;


                if(dist_front>=mRMaxDist){
                   cout<<"状态完成 开始沿边"<<endl;
                   stopRobot();
                   mStates = Enum::esWalkByWall;
                   isFirstIntoState = true;
                   break;
                }

                CPose2D robotSearch;
//                std::vector<CPose2D> vrealPathPose ;
//                p_dataSave->getMotionRobotPose(vrealPathPose);
                std::vector<cv::Point2d> vMotionPoint;

                double roadAngle = mRoadPose.phi_angle()+45;
                if(roadAngle>180) roadAngle -=360;
                if(roadAngle<-180) roadAngle +=360;

                if(vrealPathPose.size()>0){
                  robotSearch = p_dataSave->coordinateFromRobotToMap(vrealPathPose[0]);

                  robotSearch.phi_angle(roadAngle);
                }else{
                  robotSearch = mRobotPose;
                  robotSearch = p_dataSave->coordinateFromRobotToMap(robotSearch);
                  robotSearch.phi_angle(roadAngle);
                }

                getBorderPointRealTimeInLine(srcImgae,((mRobotSize))/0.05,robotSearch,vMotionPoint);

                cout<<"vMotionPoint的长度22 :"<<vMotionPoint.size()<<endl;
#ifdef IMSHOW1
                {
                cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
                p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
                cv::namedWindow("test_");
                cv::imshow("test_",test);
                cv::waitKey(20);
                }
#endif
                if(vMotionPoint.size()<1){
                  cout<<"所求边界点失败"<<endl;

                  if(vrealPathPose.size()>1){
                    vrealPathPose.erase(vrealPathPose.begin());

                    /**发送目标点**/
                   cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//                   double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPoint);
//                   robotToDstAngle -= mIntoRobotPose.phi_angle();
//                    if(robotToDstAngle>180) robotToDstAngle -=360;
//                    if(robotToDstAngle<-180) robotToDstAngle +=360;
//                   moveRobot(__speedRobot,robotToDstAngle);
                   moveToGoalAsMap(dstPoint);

                    p_dataSave->setRobotIfArriveDestPose(false);
                    p_dataSave->setMotionRobotPose(vrealPathPose);
                  }

                }else{
                  cout<<"所求边界成功"<<endl;

                  pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionPoint,vrealPathPose);
                  vrealPathPose.erase(vrealPathPose.begin());
  //                vrealPathPose[0].phi_angle(0);
//                  moveToGoalAsWorld(vrealPathPose[0]);
                  moveToGoalAsMap(vrealPathPose[0]);
                  p_dataSave->setRobotIfArriveDestPose(false);
                  p_dataSave->setMotionRobotPose(vrealPathPose);
                }
                isFirstIntoSState = false;
              }

              bool status = false;

              std::vector<CPose2D> vrealPathPose;
              p_dataSave->getMotionRobotPose(vrealPathPose);
              cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//              if(robotposeMap.twoCPose2DDisPoint(dstPoint)<3)
//                  status = true;
              p_dataSave->getRobotIfArriveDestPose(status);

              if(status){

                  std::vector<CPose2D> vrealPathPose;
                  p_dataSave->getMotionRobotPose(vrealPathPose);
                  CPose2D arrivePose = vrealPathPose[0];
                  arrivePose.phi_angle(mRoadPose.phi_angle());
                  double dist_front,dist_right,dist_left;   /**相对于世界坐标系**/
                  cv::Mat workImage;
                  p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
                  p_laserDataDeal->getThreeDirectorDis(workImage,arrivePose,dist_front,dist_left,dist_right);

                  cout<<"dist_front:"<<dist_front<<" dist_left:"<<dist_left<<" dist_right:"<<dist_right<<endl;

                if(dist_front>=mRMaxDist){
                   cout<<"状态完成 开始沿边"<<endl;
                   stopRobot();
                   mStates = Enum::esWalkByWall;
                   isFirstIntoState = true;
                }else {
                  cout<<"状态完成 还在退出清扫"<<endl;
                  stopRobot();
                   mSStates = Enum::essCleanGoBacktoCar1;
                   isFirstIntoSState = true;
                }
              }

            }break;
            case Enum::essCleanNormalStart:{     /* 正常清扫*/
              cout<<"正常清扫  开始清扫"<<endl;

              if(isFirstIntoSState){
               std::vector<CPose2D> vrealPathPose ;
                p_dataSave->getMotionRobotPose(vrealPathPose);

                if(vrealPathPose.size()>0){
                  cout<<"move "<<vrealPathPose[0]<<endl;

                  /**发送目标点**/
                 cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//                 double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPoint);
//                 robotToDstAngle -= mIntoRobotPose.phi_angle();
//                  if(robotToDstAngle>180) robotToDstAngle -=360;
//                  if(robotToDstAngle<-180) robotToDstAngle +=360;
//                 moveRobot(__speedRobot,robotToDstAngle);
                  moveToGoalAsMap(dstPoint);

                  p_dataSave->setRobotIfArriveDestPose(false);
                }
                isFirstIntoSState = false;
              }
              bool status = false;

              std::vector<CPose2D> vrealPathPose;
              p_dataSave->getMotionRobotPose(vrealPathPose);
              cv::Point2d dstPoint =p_dataSave->coordinateFromRobotToMap(vrealPathPose[0].toPoint());
//              if(robotposeMap.twoCPose2DDisPoint(dstPoint)<3)
//                  status = true;
              p_dataSave->getRobotIfArriveDestPose(status);

              if(status){

                std::vector<CPose2D> vrealPathPose ;
                p_dataSave->getMotionRobotPose(vrealPathPose);
                if(vrealPathPose.size()>1){
                  vrealPathPose.erase(vrealPathPose.begin());
                  p_dataSave->setMotionRobotPose(vrealPathPose);
                  mSStates = Enum::essCleanNormalStart;
                  isFirstIntoSState = true;
                }else{

                  if(mIsCleanFinish){
                    mSStates = Enum::essCleanGoBacktoCar1;
                    isFirstIntoSState = true;
                  }else{

                    double dist_l =5;
//                    std::vector<double>laserData;
//                    p_dataSave->getLaserData(laserData);
//                    p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,-robotposeReal.phi_angle(),dist_l);
//                    cout<<"dist_l:"<<dist_l<<endl;
                    CPose2D start,end;
                    end = mEndIntoCleanPose;
                    start = mFirstIntoCleanRobotPose;
                    end = p_dataSave ->coordinateFromRobotToMap(end);
                    start = p_dataSave ->coordinateFromRobotToMap(start);

                    end.phi_angle(mRoadPose.phi_angle());
                    start.phi_angle(mRoadPose.phi_angle());

                      cv::Mat workImage;
                      p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,(mRobotSize)/0.05);   /*膨胀机器人宽度得到工作地图*/
                      CPose2D end11,end21,start11,start21;
                     end11 = p_laserDataDeal->PoseInFrontOfRobot(workImage,end,4);
                     start11 = p_laserDataDeal->PoseInFrontOfRobot(workImage,start,4);
                     end21 = p_laserDataDeal->PoseInFrontOfRobot(workImage,end,8);
                     start21 = p_laserDataDeal->PoseInFrontOfRobot(workImage,start,8);

                      end11 = p_dataSave->coordinateFromMapToRobot(end11);
                      end21 = p_dataSave->coordinateFromMapToRobot(end21);
                      start11 = p_dataSave->coordinateFromMapToRobot(start11);
                      start21 = p_dataSave->coordinateFromMapToRobot(start21);

                    if(dist_l>/*mFMinDist*/0){
                      if(mTraTemplate==1){
                        mFirstIntoCleanRobotPose = start21;
                        mEndIntoCleanPose = end21;
                      }else if(mTraTemplate==2){
                        mFirstIntoCleanRobotPose = start11;
                        mEndIntoCleanPose = mEndIntoCleanPose;
                      }else if(mTraTemplate==3){
                        mFirstIntoCleanRobotPose = mFirstIntoCleanRobotPose;
                        mEndIntoCleanPose = end11;
                      }


                      mSStates = Enum::essCleanNormalClean;
                      isFirstIntoSState = true;
                    }else {
                      mSStates = Enum::essCleanGoBacktoCar1;
                      isFirstIntoSState = true;
                    }
                  }
                }
              }
            }break;

            default:
              break;
            }
  //          mWorkParkingClean2();
          }break;
          default:
            break;
          }


      }else if(mModes==Enum::emTest){            /*测试*/
//          p_dataSave->clearAllRobotPose();
cout<<"em test "<<endl;
//          mIdleModeDealThing();
cout<<"em i "<<m_i<<endl;
#ifdef IMSHOW1
          {
          cv::Mat srcImgae;
          p_dataSave->getGlobalMapMsg(srcImgae);  /*订阅全局地图*/
          if(srcImgae.empty()) continue; /*全局地图为空 返回*/

            /**为了显示机器人位姿是否正确**/
            CPose2D robotposeMap,robotposeReal;
            p_dataSave->getRobotPose(robotposeReal);    /*获取机器人实际位姿*/
//            cout<<"实际机器人的位姿 /m:"<<robotposeReal<<endl;
            p_dataSave->getRobotPoseMap(robotposeMap);  /*获取机器人地图位姿*/


            if(firstTest){                 /**目标点获取*/
                 CPose2D robotposeMap1;
                 robotposeMap1 = robotposeMap;
                 startPose = robotposeMap1;
                 cout<<"m_i: "<<  m_i <<endl;
//                 sleep(1);
                 if(m_i==0){
                   robotposeMap1.phi_angle(robotposeMap.phi_angle()+45);
                   dstPose =  p_laserDataDeal->PoseInFrontOfRobot(srcImgae,robotposeMap1,20);
                 }else  if(m_i==1){
                   robotposeMap1.phi_angle(robotposeMap.phi_angle()-45);
                   dstPose =  p_laserDataDeal->PoseInFrontOfRobot(srcImgae,robotposeMap1,20);
                 }else  if(m_i==2){
                   robotposeMap1.phi_angle(robotposeMap.phi_angle());
                   dstPose =  p_laserDataDeal->PoseInFrontOfRobot(srcImgae,robotposeMap1,20);
                }else  if(m_i==3){
                   robotposeMap1.phi_angle(robotposeMap.phi_angle()+180);
                   dstPose =  p_laserDataDeal->PoseInFrontOfRobot(srcImgae,robotposeMap1,20);
                }
                 double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPose.toPoint());
                 robotToDstAngle -= robotposeMap.phi_angle();
                  if(robotToDstAngle>180) robotToDstAngle -=360;
                  if(robotToDstAngle<-180) robotToDstAngle +=360;
                 cout<<"robotToDstAngle::"<<robotToDstAngle<<endl;
                 moveRobot(__speedRobot,robotToDstAngle);
                 firstTest = false;

            }
            double robotToDstAngle =  getTwoPointAngleInMap(robotposeMap.toPoint(),dstPose.toPoint());
            double distAngle =  robotToDstAngle - robotposeMap.phi_angle();
             if(distAngle>180) distAngle -=360;
             if(distAngle<-180) distAngle +=360;
             double mrobotDist = startPose.twoCPose2DDis(robotposeMap);

            cout<<"robotToDstAngle::"<<robotToDstAngle<<endl;
            cout<<"robotposeMap.phi_angle()::"<<robotposeMap.phi_angle()<<endl;
            cout<<"distAngle::"<<distAngle<<endl;
            cout<<"mrobotDist"<<mrobotDist<<endl;
            cout<<"-------------------"<<endl;
            if(fabs(distAngle)>5/*||mrobotDist>26*/)
                 moveRobot(__speedRobot,distAngle);
             cout<<"地图机器人位姿: /grid:"<<robotposeMap<<endl;
             cout<<"目标点位姿: /grid:"<<dstPose<<endl;

             cout<<" m_i"<< m_i<<endl;
             if(dstPose.twoCPose2DDis(robotposeMap)<3){
                stopRobot();
                cout<<"dst is arrive "<<endl;
                sleep(5);
                firstTest = true;
                m_i++;
                if(m_i>=4){
                  cout<<"finish "<<endl;
                  sleep(5);
                }
//                break;
             }



#ifdef IMSHOW1

            {
            cv::Mat workImage; std::vector<cv::Point2d> mVRobotPose;
            p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
            p_laserDataDeal->drawArrow(workImage,robotposeMap,8);/**显示机器人位姿和原图**/
            p_dataSave->getAllRobotPoseInMap(mVRobotPose);
            p_laserDataDeal->pointAndPointLinePaintInMap(workImage,mVRobotPose);
            cv::namedWindow("test1");
            cv::imshow("test1",workImage);
            cv::waitKey(20);
            }
#endif

          }
#endif






      }
  //    sleep(1);
    }



}

void parkingClean:: run_process(){



  while (ros::ok()) {

     cout<<"-------------------"<<endl;
    p_dataSave->getSystermMode(mModes);   /**获取系统模式**/
    if(mModes==Enum::emIdle){                   /*空闲*/
        cout<<"空闲 "<<endl;
        mIdleModeDealThing();
        {
        cv::Mat srcImgae;
        p_dataSave->getGlobalMapMsg(srcImgae);  /*订阅全局地图*/
        if(srcImgae.empty()) continue; /*全局地图为空 返回*/
          /**为了显示机器人位姿是否正确**/
          CPose2D robotposeMap,robotposeReal;
          p_dataSave->getRobotPose(robotposeReal);    /*获取机器人实际位姿*/
          cout<<"实际机器人的位姿 /m:"<<robotposeReal<<endl;
          p_dataSave->getRobotPoseMap(robotposeMap);  /*获取机器人地图位姿*/
          cout<<"地图机器人位姿: /grid:"<<robotposeMap<<endl;
          {
          cv::Mat workImage; std::vector<cv::Point2d> mVRobotPose;
          p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
          p_laserDataDeal->drawArrow(workImage,robotposeMap,8);/**显示机器人位姿和原图**/
          p_dataSave->getAllRobotPoseInMap(mVRobotPose);
          p_laserDataDeal->pointAndPointLinePaintInMap(workImage,mVRobotPose);
//            std::cout<<"mVRobotPose.size(): "<< mVRobotPose.size()<<std::endl;
          cv::namedWindow("test1");
          cv::imshow("test1",workImage);
          cv::waitKey(20);
          }

        }
    }else if(mModes==Enum::emWork){             /*工作*/
        cout<<"工作 "<<endl;

        cv::Mat srcImgae;   /**初始地图**/
        p_dataSave->getGlobalMapMsg(srcImgae);  /*订阅全局地图*/
        if(srcImgae.empty()) continue; /*全局地图为空 返回*/

          /**为了显示机器人位姿是否正确**/
          CPose2D robotposeMap,robotposeReal;
          p_dataSave->getRobotPose(robotposeReal);    /*获取机器人实际位姿*/
          cout<<"实际机器人的位姿 /m:"<<robotposeReal<<endl;
          p_dataSave->getRobotPoseMap(robotposeMap);  /*获取机器人地图位姿*/
          cout<<"地图机器人位姿: /grid:"<<robotposeMap<<endl;

           /*为显示障碍膨胀后机器人的位姿*/
          {
          cv::Mat workImage; std::vector<cv::Point2d> mVRobotPose;
          p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*膨胀机器人宽度得到工作地图*/
          p_laserDataDeal->drawArrow(workImage,robotposeMap,8);/**显示机器人位姿和原图**/
          p_dataSave->getAllRobotPoseInMap(mVRobotPose);
          p_laserDataDeal->pointAndPointLinePaintInMap(workImage,mVRobotPose);
          cv::namedWindow("test1");
          cv::imshow("test1",workImage);
          cv::waitKey(20);
          }
//          continue;

        if(isFirstIntoMode){  /*first into work init some state */
            mStates = Enum::esWalkNearWall;    /*靠墙走*/
            isFirstIntoState =true;
            isFirstIntoMode = false;
        }

        switch (mStates) {
//        case Enum::esNavigateOneToOne:{
//          cout<<"从一个点到另一个点 "<<endl;
//          if(isFirstIntoState){
//          std::vector<CPose2D> vrealPathPose;
//          std::vector<cv::Point2d> vMotionPoint;
//          p_dataSave->getMotionRobotPose(vrealPathPose);

//        }

//        }break;
        case Enum::esWalkByWall:{          /*沿墙走*/
          cout<<" 机器人正在沿墙走 "<<endl;

          if(isFirstIntoState){

            CPose2D robotSearch;
            std::vector<CPose2D> vrealPathPose;
            std::vector<cv::Point2d> vMotionPoint;
            p_dataSave->getMotionRobotPose(vrealPathPose);
            if(vrealPathPose.size()>0){
              mRoadPose = CPose2D(vrealPathPose[0].x(),vrealPathPose[0].y(),45);
              robotSearch = vrealPathPose[0];
              robotSearch.phi_angle(0);
            }else {
              cout<<"沿墙走有误 "<<endl;
              robotSearch = robotposeReal;
            }

            robotSearch = p_dataSave->coordinateFromRobotToMap(robotSearch);

            getBorderPointRealTimeInLine(srcImgae,(mRobotSize+0.1)/0.05,robotSearch,vMotionPoint);
            cout<<"vMotionPoint的长度 /m:"<<vMotionPoint.size()<<endl;

            /**显示一下所找的边界点**/{
              cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
              p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
              cv::namedWindow("test_");
              cv::imshow("test_",test);
              cv::waitKey(20);
            }

             /**发送目标点**/
            if(vMotionPoint.size()<1){
              cout<<"所求边界点失败"<<endl;
              vrealPathPose.erase(vrealPathPose.begin());
              moveToGoalAsMap(vrealPathPose[0]);
              p_dataSave->setRobotIfArriveDestPose(false);
              p_dataSave->setMotionRobotPose(vrealPathPose);
            }else{
              cout<<"所求边界成功"<<endl;
              pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionPoint,vrealPathPose);
              vrealPathPose.erase(vrealPathPose.begin());
              moveToGoalAsWorld(vrealPathPose[0]);
              p_dataSave->setRobotIfArriveDestPose(false);
              p_dataSave->setMotionRobotPose(vrealPathPose);
            }

            isFirstIntoState = false;
          }

          double dist_180,dist_210,dist_150;   /**相对于世界坐标系**/
          double Angle_180 = -90 - robotposeReal.phi_angle();
          if(Angle_180<-120) Angle_180 = -115;
          double Angle_150 = -60 - robotposeReal.phi_angle();
          double Angle_210 = -120 - robotposeReal.phi_angle();
          if(Angle_210<-120) Angle_210 = -117;

          std::vector<double>laserData;
          p_dataSave->getLaserData(laserData);
          p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,Angle_180,dist_180);
          p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,Angle_150,dist_150);
          p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,Angle_210,dist_210);
          cout<<"dist_210:"<<dist_210<<" dist_180:"<<dist_180<<" dist_150:"<<dist_150<<endl;

          bool status = false;
          p_dataSave->getRobotIfArriveDestPose(status);
          if(status){ /*navigation  finish*/
                      if(dist_180>=mRMaxDist){
                         cout<<"状态完成 满足清扫条件"<<endl;
                         stopRobot();
                         mStates = Enum::esParkingClean;
                         isFirstIntoState = true;
                      }else {
                        mStates = Enum::esWalkByWall;
                        isFirstIntoState = true;
                      }
                    }else{      /*navigation not finish*/

                       std::vector<CPose2D> vrealPathPose;
                       p_dataSave->getMotionRobotPose(vrealPathPose);
                       cout<<" --- dstPOSE: "<<  vrealPathPose[0] <<endl;
                       double lenRTD = robotposeReal.twoCPose2DDis(vrealPathPose[0]);
                     if((dist_180>=mRMaxDist)){      /*检测到右侧*/
                       cout<<"激光检测到右侧可能为清扫"<<endl;
                       if(lenRTD>(mRobotSize+0.1)){
                         if((dist_210>2*(mRobotSize+0.1))&&(dist_150>2*(mRobotSize+0.1))){
                            cout<<"现在应该是清扫"<<endl;
                            moveToGoalAsRobot(0,0);
                            usleep(1000*100);
                            mStates = Enum::esParkingClean;
                            isFirstIntoState = true;
                         }else{
                            cout<<"道路过小 不符合"<<endl;
                         }
                       }else{
                         cout<<"目标点过近不考虑"<<endl;
                       }

                       }
                   usleep(muSleepTimes); }
//          if(status){
//            double dist_r,dist_f;
//            double Angle_r =-90 - robotposeReal.phi_angle();
//            if(Angle_r<-120) Angle_r = -115;
//            std::vector<double>laserData;
//            p_dataSave->getLaserData(laserData);
//            p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,Angle_r,dist_r);
//            p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,0,dist_f);
//            cout<<"dist_r:"<<dist_r<<endl;

//            if(dist_r>=mRMaxDist){
//               cout<<"满足清扫条件"<<endl;
//               stopRobot();
//               if(mIsCleanFinish){
//                 mStates = Enum::esWalkByWall;
//                 cout<<"退出清扫"<<endl;
//                 mIsCleanFinish =  false;
//               }
//               else{
//                 mStates = Enum::esParkingClean;
//               }

//               isFirstIntoState = true;
////               std::vector<CPose2D> vrealPathPose ,vMotionDestPose;
////               p_dataSave->getMotionRobotPose(vrealPathPose);
////               cout<<"vrealPathPose[0]:"<<vrealPathPose[0]<<endl;
//            }/*else if (dist_r<mRMaxDist&&dist_r>=mFMinDist) {
//               mStates = Enum::esWalkNearWall;
//               isFirstIntoState = true;
//            }*/
//            else {
//              mStates = Enum::esWalkByWall;
//              isFirstIntoState = true;
//            }
//          }
         } break;

        case Enum::esWalkNearWall:{    /*靠墙走*/
          cout<<" 机器人正在靠近墙走"<<endl;
          if(isFirstIntoState){

            mRoadPose = CPose2D(0,0,0);

            /**求取边界点**/
            std::vector<CPose2D> vrealPathPose;
            std::vector<cv::Point2d> vMotionPoint;
            getBorderPointRealTime(srcImgae,(mRobotSize+0.1)/0.05,robotposeMap,vMotionPoint); /**获取边界点**/
            if(vMotionPoint.size()<1){
              cout<<"所求边界点失败"<<endl;
              sleep(1);
              continue;
            }
            /**显示所找到的点**/
            {
            cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
            p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
            cv::namedWindow("test_");
            cv::imshow("test_",test);
            cv::waitKey(20);
            }

           /**发送目标点**/
            pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionPoint,vrealPathPose); /*将边界点转化到机器人坐标系*/
            vrealPathPose[0].phi_angle(0);  /**第一个目标点的方向为 0度 **/
            moveToGoalAsWorld(vrealPathPose[0]);
            p_dataSave->setRobotIfArriveDestPose(false);
            p_dataSave->setMotionRobotPose(vrealPathPose);
            isFirstIntoState = false;
          }

          /**判断目标是否完成**/
           bool status = false;
           p_dataSave->getRobotIfArriveDestPose(status);
           if(status){
             cout<<" 机器人靠近墙走结束 ----"<<endl;
             mStates = Enum::esWalkByWall;
             isFirstIntoState = true;
           }

        } break;
        case Enum::esParkingClean:{    /*清扫车位*/
          cout<<" 机器人正在清扫工作"<<endl;

           mRobotPose = robotposeReal;
          if(isFirstIntoState){  /*第一次进入清扫车位*/

            mSStates = Enum::essCleanTurnIntoCar1;
//            mRoadPose = mRobotPose;   //第一次进入清扫时道路方向
            std::vector<CPose2D> vrealPathPose;
            p_dataSave->getMotionRobotPose(vrealPathPose);
            if(vrealPathPose.size()>0){
              cout<<"vrealPathPose[0]:"<<vrealPathPose[0]<<endl;
              mFirstIntoCleanRobotPose = vrealPathPose[0];    //第一次进入清扫时机器人状态
              vrealPathPose.erase(vrealPathPose.begin()+1,vrealPathPose.end());
              p_dataSave->setMotionRobotPose(vrealPathPose);
            }else{
              mFirstIntoCleanRobotPose = robotposeReal;;    //第一次进入清扫时机器人状态
            }

            mIsCleanFinish = false;
            mIsParkingCleanNow = true;
            isFirstIntoSState = true;
            isFirstIntoState = false;

          }
          /*在清扫状态下 的小状态*/
          switch (mSStates) {
          case Enum::essCleanTurnIntoCar1:{     /*右传90 方向进入车库*/
             cout<<"清扫步骤  右传90 方向进入车库"<<endl;
             if(isFirstIntoSState){

               mDestTheata = -80;        /*计算目标角度*/
               mLastRobotPose = mRobotPose;
               mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
               if(mstaticTheata>180) mstaticTheata -=360;
               if(mstaticTheata<-180) mstaticTheata +=360;
               if(mstaticTheata<0){
                 rotateRobot(-mSpeedAngule);    //右转
               }else{
                 rotateRobot(mSpeedAngule);     //左传
               }
               isFirstIntoSState = false;
             }
             mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
             if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;

             if(mRealTimeTheata+20>=fabs(mstaticTheata)){
                stopRobot();
                usleep(muSleepTimes);
                mSStates = Enum::essCleanGoIntoCar1;
                isFirstIntoSState = true;
             }
          } break;
          case Enum::essCleanGoIntoCar1:{  // 前进:进入车位，直到碰到障碍物  记录行走的距离
            cout<<"清扫步骤  进入车位，直到碰到障碍物 "<<endl;
            if(isFirstIntoSState){
//              cout<<"------------------------------- "<<endl;
              CPose2D robotSearch;
              std::vector<CPose2D> vrealPathPose ;
              std::vector<cv::Point2d> vMotionPoint;
              p_dataSave->getMotionRobotPose(vrealPathPose);
              if(vrealPathPose.size()>0){
                 cout<<"进入车位  有初始点"<<endl;
                 robotSearch = vrealPathPose[0];
              }else{
                cout<<"进入车位  无初始点"<<endl;
                robotSearch = robotposeReal;
              }
              robotSearch.phi_angle(-80);
              cout<<"进入车位 查找边界点: "<< robotSearch<<endl;
              robotSearch = p_dataSave->coordinateFromRobotToMap(robotSearch);  /*转换到地图坐标系*/

              getBorderPointRealTimeInLine(srcImgae,((mRobotSize+0.1))/0.05,robotSearch,vMotionPoint);
              {   /*显示所求的边界点*/
              cout<<"vMotionPoint的长度22 :"<<vMotionPoint.size()<<endl;
              cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
              p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
              cv::namedWindow("test_");
              cv::imshow("test_",test);
              cv::waitKey(20);
              }

              if(vMotionPoint.size()<1){
                 cout<<"所求边界点失败"<<endl;
                if(vrealPathPose.size()>1){   /*所求边界点失败  但是上次成功*/
                   vrealPathPose.erase(vrealPathPose.begin());
                   moveToGoalAsWorld(vrealPathPose[0]);
                   p_dataSave->setRobotIfArriveDestPose(false);
                   p_dataSave->setMotionRobotPose(vrealPathPose);
                }else{
                  cout<<"error" <<endl;
                  continue;
                }
              }else{
                cout<<"所求边界成功"<<endl;
                pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionPoint,vrealPathPose);
                vrealPathPose.erase(vrealPathPose.begin());
                if(vrealPathPose.size()>0){
                  moveToGoalAsWorld(vrealPathPose[0]);
                  p_dataSave->setRobotIfArriveDestPose(false);
                  p_dataSave->setMotionRobotPose(vrealPathPose);
                }
              }
              isFirstIntoSState = false;
            }
              cout<<"__________________"<<endl;
              cout<<robotposeReal<<"++++++++++"<<mFirstIntoCleanRobotPose<<endl;
              cout<<"robotposeReal.twoCPose2DDis(mFirstIntoCleanRobotPose)"<<robotposeReal.twoCPose2DDis(mFirstIntoCleanRobotPose)<<endl;
            if(robotposeReal.twoCPose2DDis(mFirstIntoCleanRobotPose)>1.5){

              mEndIntoCleanPose =robotposeReal.toPoint();
              moveToGoalAsRobot(CPose2D(0,0,0));
             mSStates = Enum::essCleanNormalClean;
             isFirstIntoSState = true;
             cout<<"qianjin 22222m   dao  tou le"<<endl;
             sleep(1);
             break;
            }

            bool status = false;
            p_dataSave->getRobotIfArriveDestPose(status);
            if(status){

              double dist_180,dist_90;   /**相对于世界坐标系**/
              double Angle_180 =-90 - robotposeReal.phi_angle();
              if(Angle_180<-120) Angle_180 = -115;
              std::vector<double>laserData;
              p_dataSave->getLaserData(laserData);
              p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,Angle_180,dist_180);
              p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,-robotposeReal.phi_angle(),dist_90);
    //          cout<<"dist_180:"<<dist_180<<endl;

              if(dist_180<=mFMinDist){
                 cout<<"到头了"<<endl;

                 if(dist_90<=mFMinDist){        /****/
                   cout<<"右侧距离太近原路返回 "<<endl;
                   stopRobot();
                   mSStates = Enum::essCleanGoBacktoCar1;
                   isFirstIntoSState = true;
                 }else{
                    cout<<"正常清扫 "<<endl;
                    std::vector<CPose2D> vrealPathPose ;
                   p_dataSave->getMotionRobotPose(vrealPathPose);
                    mEndIntoCleanPose = cv::Point2d(vrealPathPose[0].x(),vrealPathPose[0].y());
                    vrealPathPose.clear();
                    p_dataSave->setMotionRobotPose(vrealPathPose);
                   mSStates = Enum::essCleanNormalClean;
                   isFirstIntoSState = true;
                 }

              }else {

                std::vector<CPose2D> vrealPathPose ;
                p_dataSave->getMotionRobotPose(vrealPathPose);

                if(vrealPathPose.size()>1){

                    cout<<"vrealPathPose"<<vrealPathPose[0].twoCPose2DDis(vrealPathPose[1])<<endl;
                  if(/*(vrealPathPose[0].twoCPose2DAngle(vrealPathPose[1])>45)&&*/(vrealPathPose[0].twoCPose2DDis(vrealPathPose[1])>1)){

                    cout<<"还没走完 继续行走"<<endl;
                    stopRobot();
                    mSStates = Enum::essCleanGoIntoCar1;
                    isFirstIntoSState = true;
                  }else{
                    cout<<" 还没走到头 正常清扫 "<<endl;
                    std::vector<CPose2D> vrealPathPose ;
                   p_dataSave->getMotionRobotPose(vrealPathPose);
                    mEndIntoCleanPose = cv::Point2d(vrealPathPose[0].x(),vrealPathPose[0].y());
                    vrealPathPose.clear();
                    p_dataSave->setMotionRobotPose(vrealPathPose);
                   mSStates = Enum::essCleanNormalClean;
                   isFirstIntoSState = true;
                  }


                }else{      /*边界点就一个时*/
                  cout<<"正常清扫 "<<endl;
                  std::vector<CPose2D> vrealPathPose ;
                 p_dataSave->getMotionRobotPose(vrealPathPose);
                  mEndIntoCleanPose = cv::Point2d(vrealPathPose[0].x(),vrealPathPose[0].y());
                  vrealPathPose.clear();
                  p_dataSave->setMotionRobotPose(vrealPathPose);
                 mSStates = Enum::essCleanNormalClean;
                 isFirstIntoSState = true;
                }

              }
            }

          }break;
          case Enum::essCleanNormalClean:{     /* 左传90，与道路方向水平*/
             cout<<"清扫步骤  走一个前进距离 "<<endl;
             if(isFirstIntoSState){
               cv::Point2d start,end;
               end.x = mEndIntoCleanPose.x();
               end.y = mEndIntoCleanPose.y();
               start.x = mFirstIntoCleanRobotPose.x();
               start.y = mFirstIntoCleanRobotPose.y();
               end = p_dataSave ->coordinateFromRobotToMap(end);
               start = p_dataSave ->coordinateFromRobotToMap(start);
                 cv::Mat workImage;
                 p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,(mRobotSize+0.1)/0.05);   /*膨胀机器人宽度得到工作地图*/
                mTraTimes++;
                 std::vector<cv::Point2d>vMotionDestPose;
//                 if(workImage.at<uchar>(end.y,end.x+8)>150)
//                    vMotionDestPose.push_back(cv::Point2d(end.x+8,end.y));
//                 else { cout<<"1 错误"<<endl; mIsCleanFinish = true;}
//                 if(workImage.at<uchar>(start.y,start.x+8)>150)
//                    vMotionDestPose.push_back(cv::Point2d(start.x+8,start.y));
//                 else { cout<<"2 错误"<<endl;  mIsCleanFinish = true;}
//                 if(workImage.at<uchar>(start.y,start.x+16)>150)
//                    vMotionDestPose.push_back(cv::Point2d(start.x+16,start.y));
//                 else { cout<<"3 错误"<<endl; mIsCleanFinish = true;}
//                 if(workImage.at<uchar>(end.y,end.x+16)>150)
//                    vMotionDestPose.push_back(cv::Point2d(end.x+16,end.y));
////                 else { cout<<"4 错误"<<endl; mIsCleanFinish = true;}
//                 else{
//                   vMotionDestPose.push_back(cv::Point2d(end.x+8,end.y));
//                   mIsCleanFinish = true;
//                 }




                 if(mTraTemplate==1){                       // 1 正常
                   cout<<"1 正常"<<endl;
                   if(workImage.at<uchar>(end.y,end.x+8)>150)
                      vMotionDestPose.push_back(cv::Point2d(end.x+8,end.y));
                   else {
                     mTraTemplate = 2;
                     //vMotionDestPose.push_back(cv::Point2d(end.x,end.y));
                      cout<<"2 梯形"<<endl;
                   }
                   if(workImage.at<uchar>(start.y,start.x+8)>150||mTraTimes<=1)
                      vMotionDestPose.push_back(cv::Point2d(start.x+8,start.y));
                   else {
                     mTraTemplate = 3;
                     vMotionDestPose.push_back(cv::Point2d(start.x,start.y));
                      cout<<"3 楔形"<<endl;

                   }
                   if(workImage.at<uchar>(start.y,start.x+16)>150)
                      vMotionDestPose.push_back(cv::Point2d(start.x+16,start.y));
                   else {
                     mTraTemplate = 3;
                    // vMotionDestPose.push_back(cv::Point2d(end.x+16,end.y));
                      cout<<"3 楔形"<<endl;
                   }
                   if(workImage.at<uchar>(end.y,end.x+16)>150)
                      vMotionDestPose.push_back(cv::Point2d(end.x+16,end.y));
                   else {
                     mTraTemplate = 2;
                     vMotionDestPose.push_back(cv::Point2d(end.x+8,end.y));
                     cout<<"2 梯形"<<endl;
                   }
                 }
                 else if(mTraTemplate==2){                        //2  梯形
                   if((p_laserDataDeal->isHaveObstacleBetweenPoints(workImage,cv::Point2d (start.x,start.y),cv::Point2d (end.x,end.y)))){
                     mSStates = Enum::essCleanGoBacktoCar1;
                     mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;
                     break;
                   }

                   cout<<" 梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形梯形"<<endl;
//                   if(workImage.at<uchar>(end.x,end.y)>150){
//                     vMotionDestPose.push_back(cv::Point2d(end.x,end.y));
//                   }else{
//                     cout<<"定位不准/失败"<<endl;
//                     vMotionDestPose.push_back(cv::Point2d(end.x-4,end.y));
//                   }
                   if(workImage.at<uchar>(start.y,start.x+8)>150)
                      vMotionDestPose.push_back(cv::Point2d(start.x+8,start.y));
                   else { cout<<" 错误"<<endl;  mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;}
                   if(workImage.at<uchar>(start.y,start.x+16)>150){
                     vMotionDestPose.push_back(cv::Point2d(start.x+16,start.y));
                     if(workImage.at<uchar>(end.x,end.y)>150){
                       vMotionDestPose.push_back(cv::Point2d(end.x,end.y));
                     }else{
                       cout<<"定位不准/失败"<<endl;
                       vMotionDestPose.push_back(cv::Point2d(end.x-4,end.y));
                     }

                   }
                   else {
                      cout<<" 错误"<<endl; mIsCleanFinish = true;
                     // vMotionDestPose.push_back(cv::Point2d(end.x,end.y));
                      mTraTemplate = 1;mTraTimes = 0;
                   }

                 }else if(mTraTemplate==3){                     //3  楔形
                   if(p_laserDataDeal->isHaveObstacleBetweenPoints(workImage,cv::Point2d (start.x,start.y),cv::Point2d (end.x,end.y))){
                     mSStates = Enum::essCleanGoBacktoCar1;
                     mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;
                     break;
                   }

                   cout<<" 楔形楔形楔形楔形楔形楔形楔形楔形楔形楔形楔形楔形"<<endl;
                   if(workImage.at<uchar>(end.y,end.x+8)>150)
                     vMotionDestPose.push_back(cv::Point2d(end.x+8,end.y));
                   else { cout<<" 错误"<<endl;  mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;}
                   if(workImage.at<uchar>(end.y,end.x+16)>150){
                     vMotionDestPose.push_back(cv::Point2d(end.x+16,end.y));
                   }else { cout<<" 错误"<<endl;  mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;}
                   if(workImage.at<uchar>(end.y,end.x+24)>150){
                     vMotionDestPose.push_back(cv::Point2d(start.x,start.y));

                   }else { cout<<" 错误"<<endl;  mIsCleanFinish = true;mTraTemplate = 1;mTraTimes = 0;}

                 }
//                 if(p_laserDataDeal->isHaveObstacleBetweenPoints(workImage,cv::Point2d (start.x,start.y),cv::Point2d (end.x,end.y))){
//                   mStates = Enum::esWalkByWall;
//                   mIsCleanFinish = true;mTraTemplate = 1;
//                   break;
//                 }











                 if(vMotionDestPose.empty()){

                //   mSStates = Enum::essCleanGoBacktoCar1;
                   isFirstIntoSState = true;

                 }else{
                   {   /*显示所求的边界点*/
                   cout<<"vvMotionDestPose:"<<vMotionDestPose.size()<<endl;
                   cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
                   p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionDestPose);
                   cv::namedWindow("test_");
                   cv::imshow("test_",test);
                   cv::waitKey(20);
                   }
                   std::vector<CPose2D> vrealPathPose ;
                   pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionDestPose,vrealPathPose);
                   p_dataSave->setMotionRobotPose(vrealPathPose);
                   mSStates = Enum::essCleanNormalStart;
                   isFirstIntoSState = true;
                 }
             }

          }break;
          case Enum::essCleanGoBacktoCar1:{     /* 左传90，与道路方向水平*/
            cout<<"清扫步骤  清扫车位反向出 "<<endl;

            if(isFirstIntoSState){


              double dist_180,dist_90;   /**相对于世界坐标系**/
              double Angle_180 =-90 - robotposeReal.phi_angle();
              if(Angle_180<-120) Angle_180 = -115;
              std::vector<double>laserData;
              p_dataSave->getLaserData(laserData);
              p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,Angle_180,dist_180);
              p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,-robotposeReal.phi_angle(),dist_90);

              if(dist_90>=mRMaxDist){
                 cout<<"状态完成 开始沿边"<<endl;
                 stopRobot();
                 mStates = Enum::esWalkByWall;
                 isFirstIntoState = true;
                 break;
              }

              CPose2D robotSearch;
              std::vector<CPose2D> vrealPathPose ;
              p_dataSave->getMotionRobotPose(vrealPathPose);
              std::vector<cv::Point2d> vMotionPoint;
              if(vrealPathPose.size()>0){
                robotSearch = p_dataSave->coordinateFromRobotToMap(vrealPathPose[0]);
                robotSearch.phi_angle(45);
              }else{
                robotSearch = mRobotPose;
                robotSearch = p_dataSave->coordinateFromRobotToMap(robotSearch);
                robotSearch.phi_angle(45);
              }

              getBorderPointRealTimeInLine(srcImgae,((mRobotSize+0.1))/0.05,robotSearch,vMotionPoint);

              cout<<"vMotionPoint的长度22 :"<<vMotionPoint.size()<<endl;

              cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
              p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
              cv::namedWindow("test_");
              cv::imshow("test_",test);
              cv::waitKey(20);

              if(vMotionPoint.size()<1){
                cout<<"所求边界点失败"<<endl;

                if(vrealPathPose.size()>1){
                  vrealPathPose.erase(vrealPathPose.begin());
                  moveToGoalAsWorld(vrealPathPose[0]);
                  p_dataSave->setRobotIfArriveDestPose(false);
                  p_dataSave->setMotionRobotPose(vrealPathPose);
                }

              }else{
                cout<<"所求边界成功"<<endl;

                pathPointTransformCoordinateMapToRobot(robotposeMap,vMotionPoint,vrealPathPose);
                vrealPathPose.erase(vrealPathPose.begin());
//                vrealPathPose[0].phi_angle(0);
                moveToGoalAsWorld(vrealPathPose[0]);
                p_dataSave->setRobotIfArriveDestPose(false);
                p_dataSave->setMotionRobotPose(vrealPathPose);
              }
              isFirstIntoSState = false;
            }

            bool status = false;
            p_dataSave->getRobotIfArriveDestPose(status);
            if(status){

              double dist_180,dist_90;   /**相对于世界坐标系**/
              double Angle_180 =-90 - robotposeReal.phi_angle();
              if(Angle_180<-120) Angle_180 = -115;
              std::vector<double>laserData;
              p_dataSave->getLaserData(laserData);
              p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,Angle_180,dist_180);
              p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,-robotposeReal.phi_angle(),dist_90);

              if(dist_90>=mRMaxDist){
                 cout<<"状态完成 开始沿边"<<endl;
                 stopRobot();
                 mStates = Enum::esWalkByWall;
                 isFirstIntoState = true;
              }else {
                cout<<"状态完成 还在退出清扫"<<endl;
                stopRobot();
                 mSStates = Enum::essCleanGoBacktoCar1;
                 isFirstIntoSState = true;
              }
            }
//            else{
//              std::vector<CPose2D> vrealPathPose ;
//              p_dataSave->getMotionRobotPose(vrealPathPose);
//              if(vrealPathPose[0].phi_angle()<20){
//                mStates = Enum::esWalkByWall;
//              }else{
//                mStates = Enum::esWalkByWall;
//              }
//            }

          }break;
          case Enum::essCleanNormalStart:{     /* 左传90，与道路方向水平*/
            cout<<"正常清扫  开始清扫"<<endl;

            if(isFirstIntoSState){
             std::vector<CPose2D> vrealPathPose ;
              p_dataSave->getMotionRobotPose(vrealPathPose);

              if(vrealPathPose.size()>0){
                cout<<"move "<<vrealPathPose[0]<<endl;
                moveToGoalAsWorld(vrealPathPose[0]);
                p_dataSave->setRobotIfArriveDestPose(false);
              }
              isFirstIntoSState = false;
            }
            bool status = false;
            p_dataSave->getRobotIfArriveDestPose(status);
            if(status){

              std::vector<CPose2D> vrealPathPose ;
              p_dataSave->getMotionRobotPose(vrealPathPose);
              if(vrealPathPose.size()>1){
                vrealPathPose.erase(vrealPathPose.begin());
                p_dataSave->setMotionRobotPose(vrealPathPose);
                mSStates = Enum::essCleanNormalStart;
                isFirstIntoSState = true;
              }else{

                if(mIsCleanFinish){
                  mSStates = Enum::essCleanGoBacktoCar1;
                  isFirstIntoSState = true;
                }else{
                  double dist_l;
                  std::vector<double>laserData;
                  p_dataSave->getLaserData(laserData);
                  p_laserDataDeal->getLenAsLaserAngle(laserData,-120,120,-robotposeReal.phi_angle(),dist_l);
                  cout<<"dist_l:"<<dist_l<<endl;

                  if(dist_l>/*mFMinDist*/0){
                    if(mTraTemplate==1){
                      mFirstIntoCleanRobotPose = CPose2D(mFirstIntoCleanRobotPose.x()+0.8,mFirstIntoCleanRobotPose.y(),mFirstIntoCleanRobotPose.phi());
                      mEndIntoCleanPose = CPose2D(mEndIntoCleanPose.x()+0.8,mEndIntoCleanPose.y(),mEndIntoCleanPose.phi());
                    }else if(mTraTemplate==2){
                      mFirstIntoCleanRobotPose = CPose2D(mFirstIntoCleanRobotPose.x()+0.8,mFirstIntoCleanRobotPose.y(),mFirstIntoCleanRobotPose.phi());
                      mEndIntoCleanPose = CPose2D(mEndIntoCleanPose.x(),mEndIntoCleanPose.y(),/*mEndIntoCleanPose.phi()*/45);
                    }else if(mTraTemplate==3){
                      mFirstIntoCleanRobotPose = CPose2D(mFirstIntoCleanRobotPose.x(),mFirstIntoCleanRobotPose.y(),mFirstIntoCleanRobotPose.phi());
                      mEndIntoCleanPose = CPose2D(mEndIntoCleanPose.x()+0.8,mEndIntoCleanPose.y(),mEndIntoCleanPose.phi());
                    }


                    mSStates = Enum::essCleanNormalClean;
                    isFirstIntoSState = true;
                  }else {
                    mSStates = Enum::essCleanGoBacktoCar1;
                    isFirstIntoSState = true;
                  }
                }
              }
            }
          }break;

          default:
            break;
          }
//          mWorkParkingClean2();
        }break;
        default:
          break;
        }


    }else if(mModes==Enum::emTest){            /*测试*/
        p_dataSave->clearAllRobotPose();
    }
//    sleep(1);
  }
}

void parkingClean::run_processTest(){

  Struct::sDists threeDist,threeDistLaser;   //三个方向的距离
  while (ros::ok()) {

//#ifndef TEST_ROBOT_CONTROL
    cout<<"----------------------"<<endl;

    cv::Mat srcImgae;
    CPose2D robotpose,robotpose1;

    p_dataSave->getGlobalMapMsg(srcImgae);  /*订阅全局地图*/
    if(srcImgae.empty()) continue; /*全局地图为空 返回*/
    CPose2D robotposeMap,robotposeReal;
    p_dataSave->getRobotPose(robotposeReal);    /*获取机器人实际位姿*/
    cout<<"实际机器人的位姿 /m:"<<robotposeReal<<endl;
    p_dataSave->getRobotPoseMap(robotposeMap);  /*获取机器人地图位姿*/
    cout<<"地图机器人位姿: /grid:"<<robotposeMap<<endl;

    /*为显示障碍膨胀后机器人的位姿*/
    cv::Mat workImage;
    p_laserDataDeal->expendGridMapRelyNum(srcImgae,workImage,mRobotSize/0.05);   /*碰撞机器人宽度得到工作地图*/
    p_laserDataDeal->drawArrow(workImage,robotposeMap,8);/**显示机器人位姿和原图**/
    cv::namedWindow("test1");
    cv::imshow("test1",workImage);
    cv::waitKey(20);
    Struct::sDists threeDist;
    p_laserDataDeal->getThreeDirectorDis(workImage,robotposeMap,threeDist.F,threeDist.L,threeDist.R);
    Struct::sDists threeDistLaser;
    p_dataSave->getThreeDirectDist(threeDistLaser);
    mDist_F = threeDistLaser.F;
    mDist_R = threeDistLaser.R;
    mDist_L = threeDistLaser.L;

    mRobotPose = robotposeReal;
    p_dataSave->getSystermMode(mModes);   /**获取系统模式**/
//#endif
    if(mModes==Enum::emIdle){                   /*空闲*/
        cout<<"空闲 "<<endl;
        mIdleModeDealThing();

    }else if(mModes==Enum::emWork){             /*工作*/

      if((mDist_F==0)&&(mDist_L==0)&&(mDist_R==0)){
         cout<<"激光没有开启 "<<endl;
         sleep(1);
        continue;     //防止激光为开启
      }
        cout<<"工作 "<<endl;

      mWorkModeDealThing();

    }else if(mModes==Enum::emTest){            /*测试*/
      Enum::rControls rControls = Enum::rStop;  //机器人的控制命令
         if(isFirstIntoMode){       /*第一次进入 模式*/
           stopRobot();             /*机器人STOP*/
           mRControls = Enum::rStop;
           isFirstIntoMode = false;
         }
         p_dataSave->getRobotHandleControl(rControls);
         /***键盘控制   w上 s下 a左 d右 q左传  e右转  ' '暂停***/
         switch (rControls) {     /*空闲模式下 手动控制*/
         case Enum::rLeft:
           levelRobot(-0.3);
           cout<<"rLeft "<<endl;
           break;
         case Enum::rRight:
           levelRobot(0.3);
            cout<<"rRight "<<endl;
           break;
         case Enum::rFront:
           forwardRobot(0.3);
            cout<<"rFront "<<endl;
           break;
         case Enum::rRear:
           forwardRobot(-0.3);
            cout<<"rRear "<<endl;
           break;
         case Enum::rStop:
            cout<<"rStop "<<endl;
           stopRobot();
           break;
         case Enum::rLeftRotate:   //左传
           rotateRobot(0.3);
            cout<<"rLeftRotate "<<endl;
           break;
         case Enum::rRightRotate:   //右传
           rotateRobot(-0.3);
            cout<<"rRightRotate "<<endl;
           break;
         default:
           break;
         }

         cv::Mat test(srcImgae.size(),CV_8UC1,cv::Scalar(255));
         std::vector<cv::Point2d> vMotionPoint;
          getBorderPointRealTime(workImage,2,robotpose1,vMotionPoint);
          p_laserDataDeal->pointAndPointLinePaintInMap(test,vMotionPoint);
         cv::namedWindow("test_");
         cv::imshow("test_",test);
         cv::waitKey(20);

//     Enum::rControls rControls = Enum::rStop;  //机器人的控制命令
//      p_dataSave->getRobotHandleControl(rControls);
////     if(rControls==Enum::rFront){
//        moveToGoalAsWorld(1,2,18);
////     } else if(rControls==Enum::rLeftRotate){
////        moveToGoalAsWorld(2,2,50);
////     } else
////        mTestModeDeal();3weeqqweeqq
//       cout<<"测试 "<<endl;
//        sleep(1);
         /*所求右边边的斜率*/
//         CPose2D testpoint2d;
//         if(getBorderSlope(workImage,robotpose1,testpoint2d)==0)
//            cout<<"所求边界斜率正确 "<< testpoint2d << endl;
//         else
//            cout<<"所求边界斜率有误 "<<endl;
//         usleep(1000);
    }
  }
}

void parkingClean::mTestModeDeal(){


  cout<<"mFMinDist: "<< mFMinDist  <<endl;
  cout<<"mRMinDist: "<< mRMinDist  <<endl;
  cout<<"mRMaxDist: "<< mRMaxDist  <<endl;
  cout<<"mFCleanDist: "<< mFCleanDist  <<endl;
}


void parkingClean::mTheataRotateCorrection(){

  if(isFirstIntoSState){
    if(mDist_R<=mRMaxDist){
       mStates = Enum::esWalkByWall;
       cout<<"前进 机器人宽度后右侧距离不满足清扫条件 "<<endl;
       isFirstIntoState = true;
       stopRobot();
//       break;
    }
    mDestTheata = mFirstIntoCleanRobotPose.phi_angle()+90;        /*计算目标角度*/
    mLastRobotPose = mRobotPose;
    mstaticTheata = mDestTheata - mLastRobotPose.phi_angle();
    if(mstaticTheata>180) mstaticTheata -=360;
    if(mstaticTheata<-180) mstaticTheata +=360;
    if(mstaticTheata>0){
      rotateRobot(-mSpeedAngule);    //右转
    }else{
      rotateRobot(mSpeedAngule);     //左传
    }
    isFirstIntoSState = false;
  }
  mRealTimeTheata = fabs(mRobotPose.phi_angle()-mLastRobotPose.phi_angle());
  if(mRealTimeTheata>180) mRealTimeTheata=360-mRealTimeTheata;
  if(mRealTimeTheata>=fabs(mstaticTheata)){
     stopRobot();
     usleep(muSleepTimes);
     mSStates = Enum::essCleanGoIntoCar;
     isFirstIntoSState = true;
  }

}

void parkingClean::levelRobot(double Speed){

  geometry_msgs::TwistPtr cmd_(new geometry_msgs::Twist());
  cmd_->linear.x = 0.0;
  cmd_->linear.y = Speed;
  cmd_->linear.z = 0.0;
  cmd_->angular.x = 0.0;
  cmd_->angular.y = 0.0;
  cmd_->angular.z = 0.0;
  p_dataSave->setRobotSpeed(cmd_);
}
void parkingClean::forwardRobot(double Speed){

  geometry_msgs::TwistPtr cmd_(new geometry_msgs::Twist());
  cmd_->linear.x = Speed;
  cmd_->linear.y = 0.0;
  cmd_->linear.z = 0.0;
  cmd_->angular.x = 0.0;
  cmd_->angular.y = 0.0;
  cmd_->angular.z = 0.0;
  p_dataSave->setRobotSpeed(cmd_);
}
void parkingClean::rotateRobot(double Speed){

  geometry_msgs::TwistPtr cmd_(new geometry_msgs::Twist());
  cmd_->linear.x = 0.0;
  cmd_->linear.y = 0.0;
  cmd_->linear.z = 0.0;
  cmd_->angular.x = 0.0;
  cmd_->angular.y = 0.0;
  cmd_->angular.z = Speed;
  p_dataSave->setRobotSpeed(cmd_);
}
void parkingClean::stopRobot(){

  geometry_msgs::TwistPtr cmd_(new geometry_msgs::Twist());
  cmd_->linear.x = 0.0;
  cmd_->linear.y = 0.0;
  cmd_->linear.z = 0.0;
  cmd_->angular.x = 0.0;
  cmd_->angular.y = 0.0;
  cmd_->angular.z = 0.0;
  p_dataSave->setRobotSpeed(cmd_);
}

void parkingClean::forwardDist(CPose2D &robotPose, double dist, double Speed){

  CPose2D initPose = robotPose,destPose;
    cout<<"initPose:"<<robotPose <<endl;
  destPose.x(robotPose.x()+dist*robotPose.phi_sin());
  destPose.y(robotPose.y()+dist*robotPose.phi_cos());
  cout<<"initPose:"<<initPose <<endl;
  cout<<"destPose:"<<destPose <<endl;

  cout<<"zhixing forwardDist:"<<endl;

  PIDParmReset();

  while(ros::ok()){

      if(robotPose.twoCPose2DDis(initPose)>=fabs(dist)){
         stopRobot();
        break;
      }else{

        usleep(10000);
        rotateRobot(PIDControl(initPose.phi_angle(),robotPose.phi_angle()));
        if(dist>0){
         forwardRobot(Speed);
        }else {
         forwardRobot(-Speed);
        }

      }
  }
}
void parkingClean::rotateAngle(CPose2D &robotPose, double angle, double Speed){

  CPose2D initPose = robotPose;
  while(ros::ok()){
    usleep(10000);
    double theata = fabs(robotPose.phi_angle()-initPose.phi_angle());
    if(theata>180) theata = 360 - theata;
    if(theata>=fabs(angle)){
       stopRobot();
       break;
    }else {
        if(angle>0){
          rotateRobot(Speed);
        }else{
          rotateRobot(-Speed);
        }
    }
  }
}

void parkingClean::moveRobot(const double &speed, const double &angle){

    double angle_ = angle/180*3.14;
      geometry_msgs::TwistPtr cmd_(new geometry_msgs::Twist());
      cmd_->linear.x = speed*cos(angle_);
      cmd_->linear.y = -speed*sin(angle_);
      cmd_->linear.z = 0.0;
      cmd_->angular.x = 0.0;
      cmd_->angular.y = 0.0;
      cmd_->angular.z = 0.0;
      p_dataSave->setRobotSpeed(cmd_);

}

//PID控制，参数1：setValue(设定值)，参数2：fBValue(反馈值)
double parkingClean::PIDControl( double setValue,double fBValue){

  double outPutError;
  double pError=0,iError=0,dError=0;
  mEK = setValue - fBValue;
  pError = mEK - mLEk;    //比例误差(等于当前误差减去前一次的误差)
  iError = mEK;   //积分误差(等于当前误差值)
  dError = mEK - 2*mLEk + mLLEk;  //微分误差(等于当前误差减去前一次2倍误差再加上前两次的误差)

  mLLEk = mLEk;   //上次复值 上上次
  mLEk = mEK;     //本次复值 上次

  outPutError = mKp*pError+mKi*iError+mKd* dError+mLOP;

  mLOP = outPutError;

  return outPutError;
}
void parkingClean::PIDParmReset(){
   mEK =0;
   mLEk = 0;
   mLLEk = 0;
   mLOP = 0;
}





