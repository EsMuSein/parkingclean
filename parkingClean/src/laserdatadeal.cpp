#include "laserdatadeal.h"

laserDataDeal::laserDataDeal():m_lacalMap(600,600,CV_8UC1,cv::Scalar::all(255))
{

}
laserDataDeal::~laserDataDeal(){

}

/* 画箭头函数  地图  该点位姿  箭头长度*/
void laserDataDeal::drawArrow(cv::Mat &Map,const CPose2D & RobotPose,int len){

  if(Map.channels()==3)
    cv::cvtColor(Map,Map,CV_BGR2GRAY);

  CPose2D frontIn(RobotPose),leftIn(RobotPose),rightIn(RobotPose),backIn(RobotPose);
  CPose2D frontOut,leftOut,rightOut,backOut;
  frontIn.phi_angle(RobotPose.phi_angle());
  leftIn.phi_angle(leftIn.phi_angle()-90);
  rightIn.phi_angle(rightIn.phi_angle()+90);
  backIn.phi_angle(backIn.phi_angle()+180);

  /*计算机器人前后左右前进len的长度*/
  frontOut = PoseInFrontOfRobot(Map,frontIn,len);     /*地图坐标系*/
  leftOut = PoseInFrontOfRobot(Map,leftIn,round(len/2));
  rightOut = PoseInFrontOfRobot(Map,rightIn,round(len/2));
  backOut = PoseInFrontOfRobot(Map,backIn,len);

  /*转化成彩色图画线*/
  if(Map.channels()==1)
   cv::cvtColor(Map,Map,CV_GRAY2RGB);
    /*画机器人显示的线段*/
    cv::line(Map,frontOut.toPoint(),leftOut.toPoint(),cv::Scalar(255,0,0),2);
    cv::line(Map,frontOut.toPoint(),rightOut.toPoint(),cv::Scalar(255,0,0),2);
    cv::line(Map,RobotPose.toPoint(),frontOut.toPoint(),cv::Scalar(255,0,0),2);
    cv::line(Map,RobotPose.toPoint(),backOut.toPoint(),cv::Scalar(255,0,0),2);
    cv::circle(Map,RobotPose.toPoint(),2,cv::Scalar(255,0,0),2);
}


/* 起点在障碍里时 ，将起点移出来   */

int laserDataDeal::startPointInobstacle(cv::Mat &gridMap,cv::Point& start)
{
     int height = gridMap.rows;    //栅格地图的行数   460
     int width  = gridMap.cols;    //栅格地图的列数   400

    if(gridMap.channels()==3)
        cv::cvtColor(gridMap,gridMap,CV_BGR2GRAY);

     cv::Mat expendGridMap(height,width,CV_8UC1,125);

      expendGridMap = gridMap.clone();

    //maps地图规则为0为free，1为占有
    std::vector<std::vector<char>> maps(expendGridMap.rows,std::vector<char>(expendGridMap.cols));

    for(int i=0;i<expendGridMap.rows;++i)       //row  行数
    {
        for(int j=0;j<expendGridMap.cols;++j)   //cols 列数
        {
            if(expendGridMap.at<uchar>(i,j)<200)
            {
                maps[i][j]=1;//把灰色和黑色都置为不可行//灰色未知
            }
            else if(expendGridMap.at<uchar>(i,j)>200)
            {
                maps[i][j]=0;//把白色置为可同行
            }
        }
    }
    int safeGridCountersign=1;
    int j=1;
    int safeGridCounter1_1=0, safeGridCounter2_1=0;
    int safeGridCounter1_2=0, safeGridCounter2_2=0;
    int safeGridCounter1_3=0, safeGridCounter2_3=0;
    int safeGridCounter1_4=0, safeGridCounter2_4=0;

    cv::Point obstaclePoint1_1, obstaclePoint2_1;
    cv::Point obstaclePoint1_2, obstaclePoint2_2;
    cv::Point obstaclePoint1_3, obstaclePoint2_3;
    cv::Point obstaclePoint1_4, obstaclePoint2_4;
    bool overCheck = true;
    if(maps[start.y][start.x]==1)
    {
        std::cout<<" 起点在障碍里 "<<start.x<<"  "<<start.y<<std::endl;
        //水平搜索
        //首先向左搜索
        for(j=1;j<start.x;j++)
        {
            if(maps[start.y][start.x-j]==1)
                safeGridCounter1_1++;
            else
            {
                obstaclePoint1_1 = cv::Point(start.x-j,start.y);//第一个白点
                overCheck = false;
                break;
            }
        }
        if(overCheck)
          safeGridCounter1_1=(int) cv::max(width , height);

        overCheck = true;
        //接下来向右搜索
        for(j=1;j<width - start.x;j++)
        {
            if(maps[start.y][start.x+j]==1)
                safeGridCounter2_1++;
            else
            {
                obstaclePoint2_1 = cv::Point(start.x+j,start.y);
                overCheck = false;
                break;
            }
        }
        if(overCheck)
            safeGridCounter2_1=(int)cv::max(width,height);
        overCheck = true;
        //竖直搜索
        //首先向上搜索
        for( j=1;j<start.y;j++)
        {
            if(maps[start.y-j][start.x]==1)
                safeGridCounter1_2++;
            else
            {
                obstaclePoint1_2 = cv::Point(start.x,start.y-j);
                overCheck = false;
                break;
            }
        }
        if(overCheck)
              safeGridCounter1_2=(int)cv::max(width,height);
        overCheck = true;
        //接下来向下搜索
        for(j=1;j<height-start.y;j++)
        {
            if(maps[start.y+j][start.x]==1)
                safeGridCounter2_2++;
            else
            {
                obstaclePoint2_2 = cv::Point(start.x,start.y+j);
                overCheck = false;
                break;
            }
        }
            if(overCheck)
            {
                safeGridCounter2_2=(int)cv::max(width,height);
            }
        //左上
        overCheck = true;
        unsigned int minP1 = cv::min(start.y, start.x);
        unsigned int minP2 = cv::min(height-start.y, width-start.x);
        for(j=1;j<minP1;j++)
        {
            if(maps[start.y-j][start.x-j]==1)
                safeGridCounter1_3++;
            else
            {
                obstaclePoint1_3 = cv::Point( start.x-j,start.y-j);
                overCheck = false;
                break;
            }
        }
        if(overCheck)
            safeGridCounter1_3=(int)cv::max(width,height);
        //右下
        overCheck = true;
        for(j=1;j<minP2;j++)
        {
            if(maps[start.y+j][start.x+j]==1)
                safeGridCounter2_3++;
            else
            {
                obstaclePoint2_3 = cv::Point( start.x+j,start.y+j);
                overCheck = false;
                break;
            }
        }
        if(overCheck)
            safeGridCounter2_3=(int)cv::max(width,height);
        //左下右上
        overCheck = true;
        //首先向左下搜索
         minP1 = cv::min(height-start.y, start.x);
         minP2 = cv::min(start.y, width-start.x);
         //左下
        for(j=1;j<minP1;j++)
        {
            if(maps[start.y+j][start.x-j]==1)
                safeGridCounter1_4++;
            else
            {
                obstaclePoint1_4 = cv::Point( start.x-j,start.y+j);
                overCheck = false;
                break;
            }
        }
            if(overCheck)
                safeGridCounter1_4=(int)cv::max(width,height);

        overCheck = true;
        //接着向右shang搜索
        for( j=1;j<minP2;j++)
        {
            if(maps[start.y-j][start.x+j]==1)
               {
                safeGridCounter2_4++;
                }
            else
            {
                obstaclePoint2_4 = cv::Point( start.x+j,start.y-j);
                overCheck = false;
                break;
            }
        }

            if(overCheck)
                safeGridCounter2_4=(int)cv::max(width,height);


        unsigned int minCounter = cv::max(width, height);
        if(safeGridCounter1_1>safeGridCounter2_1)
        {
            minCounter = safeGridCounter2_1;
            safeGridCountersign = 2;
        }
        else
        {
            minCounter = safeGridCounter1_1;
            safeGridCountersign = 1;
        }
        if(safeGridCounter1_2<minCounter)
        {
            minCounter = safeGridCounter1_2;
            safeGridCountersign = 3;
        }
        if(safeGridCounter2_2<minCounter)
        {
            minCounter = safeGridCounter2_2;
            safeGridCountersign = 4;
        }
        if(safeGridCounter1_3<minCounter)
        {
            minCounter =safeGridCounter1_3;
            safeGridCountersign = 5;
        }
        if(safeGridCounter2_3<minCounter)
        {
            minCounter = safeGridCounter2_3;
            safeGridCountersign = 6;
        }
        if(safeGridCounter1_4<minCounter)
        {
            minCounter = safeGridCounter1_4;
            safeGridCountersign = 7;
        }
        if(safeGridCounter2_4<minCounter)
        {
            minCounter = safeGridCounter2_4;
            safeGridCountersign = 8;
        }


                   switch (safeGridCountersign) {
                        case 1:    start =  obstaclePoint1_1;break;
                        case 2:    start =  obstaclePoint2_1;break;
                        case 3:    start =  obstaclePoint1_2;break;
                        case 4:    start =  obstaclePoint2_2;break;
                        case 5:    start =  obstaclePoint1_3;break;
                        case 6:    start =  obstaclePoint2_3;break;
                        case 7:    start =  obstaclePoint1_4;break;
                        case 8:    start =  obstaclePoint2_4;break;
                   }

                 std::cout<<" 障碍点移除 "<<start.x<<"  "<<start.y<<std::endl;
    }

    return 0;
}

/* 判断两点之间有障碍  地图 起点 终点*/
bool laserDataDeal::isHaveObstacleBetweenPoints(cv::Mat &obstacleMap,const cv::Point &start,const cv::Point& end){

  int sub_x = abs(start.x-end.x);
  int sub_y = abs(start.y-end.y);


  bool isAbsXBigY=0;
  if(sub_x>=sub_y)
      isAbsXBigY=1;
  else if(sub_x<sub_y)
      isAbsXBigY=0;

  int x1,y1,x2,y2;
  if(isAbsXBigY){

   x1=start.x,   y1=start.y;
   x2=end.x,     y2=end.y;

  }else{

       x1=start.y,   y1=start.x;
       x2=end.y,     y2=end.x;
  }

  if(obstacleMap.channels()==3)
  cv::cvtColor(obstacleMap,obstacleMap,CV_RGB2GRAY);
  if(x1 == x2)                      // start.y?=end.y
  {                                 // start.y==end.y
      if(y1 == y2)                  // start.x?=end.x
      {                             // start.x==end.x
          return false;
      } else if(y1 > y2)              //start.x>end.x
      {
          int num=abs(y1-y2);
          for(int i=1;i<num;++i)
          {
              if(isAbsXBigY){
              if(obstacleMap.at<uchar>(y1-i,x1) < mJudgeObstacleParm)
                   return true;
              }else {
                  if(obstacleMap.at<uchar>(x1,y1-i) < mJudgeObstacleParm)
                  return true;
              }
          }
      }
      else if(y1 < y2)              //start.x<end.x
      {
          int num=abs(y1-y2);
          for(int i=1;i<num;++i)
          {
               if(isAbsXBigY){
                  if(obstacleMap.at<uchar>(y1+i,x1) < mJudgeObstacleParm)
                  return true;
               }else{
                  if(obstacleMap.at<uchar>(x1,y1+i) < mJudgeObstacleParm)
                   return true;
               }
          }
      }
  }
  else if(x1 != x2)
  {
      float a=0,b=0;

      a=(float)(y1-y2)/(float)(x1-x2);
      b=(float)y1-a*(float)x1;

      if(x1 > x2)
      {
          for(int i=x1-1;i>x2;--i)
          {
              int j=round (a*i+b);

               if(isAbsXBigY){
                    if(obstacleMap.at<uchar>(j,i) < mJudgeObstacleParm)
                    return true;
               }else{

                 if(obstacleMap.at<uchar>(i,j) < mJudgeObstacleParm)
                    return true;
                 }

          }
      }
      else if(x1 < x2)
      {
          for(int i=x1+1;i<x2;++i)
          {
              int j=round (a*i+b);

             if(isAbsXBigY){
            if(obstacleMap.at<uchar>(j,i) < mJudgeObstacleParm)
                return true;
             }else{

             if(obstacleMap.at<uchar>(i,j) < mJudgeObstacleParm)
                return true;
             }
          }
      }
  }

  return false;

}

 /* 点和点之间的连线画在地图上  地图 点坐标*/
int laserDataDeal::pointAndPointLinePaintInMap(cv::Mat& Map,  std::vector<cv::Point2d> Path_Point){

  /*转化成彩色图画线*/
  if(Map.channels()==1)
   cv::cvtColor(Map,Map,CV_GRAY2RGB);

  /*将double型 转化为 int */
  for(int i=0;i<Path_Point.size();i++){
    Path_Point[i].x = round(Path_Point[i].x);
    Path_Point[i].y = round(Path_Point[i].y);
  }


  if(Path_Point.size()<1){
    return -1;
  }else if(Path_Point.size() == 1){
    cv::circle(Map, *Path_Point.begin(), 4, cv::Scalar(0,0,255), 1, 8, 0);
//      std::cout<<"1 "<< *Path_Point.begin() <<std::endl;
    return 0;
  }

  for(std::vector<cv::Point2d>::iterator iter = Path_Point.begin();iter<Path_Point.end()-1;++iter)
  {
      cv::circle(Map, *iter, 4, cv::Scalar(0,0,255), 1, 8, 0);
      cv::line(Map,*iter,*(iter+1),cv::Scalar(0,255,0),4);
  }
      cv::circle(Map, *(Path_Point.end()-1), 4, cv::Scalar(0,0,255), 1, 8, 0);

  return 0;
}

/*从栅格地图中得到子图*/
int laserDataDeal::getSubMapFromGridMap(const cv::Mat &src, cv::Mat &dst,
                                        const CPose2D&globalPose,CPose2D&subPose, int len){

  int sub_left,sub_right,sub_top,sub_bottom;

  if(src.empty()){
    std::cerr<<"srcImage is empty"<<std::endl;
    return -1;
  }

  int height = src.rows;
  int width = src.cols;

  subPose.phi(globalPose.phi());

  if(globalPose.x()-len<0){
    sub_left = 0;
    subPose.x(globalPose.x());
  } else {
    subPose.x(len);
    sub_left = globalPose.x()-len;
  }
  if(globalPose.x()+len>width){
    sub_right =width;
  }else {
    sub_right = globalPose.x()+len;
  }
  if(globalPose.y()-len<0){
    sub_top = 0;
    subPose.y(globalPose.y());
  } else {
    subPose.y(len);
    sub_top = globalPose.y()-len;
  }

  if(globalPose.y()+len>height){
    sub_bottom = height;
  } else {
    sub_bottom = globalPose.y()+len;
  }

  dst = src(cv::Range(sub_top,sub_bottom),cv::Range(sub_left,sub_right));

  return 0;
}


/* 得到激光数据  地图 当前位姿 最小角 最大角 保存激光数据 保存障碍点*/
int laserDataDeal::getLasterData(cv::Mat &Map, const CPose2D &RobotPose, const int &MinAngle, const int &MaxAngle,
                                 std::vector<double> &Laster, std::vector<CPose2D> &LasterPose){

  if(Map.channels()==3)
   cv::cvtColor(Map,Map,CV_BGR2GRAY);


  if(Map.at<uchar>(RobotPose.y(),RobotPose.x())<mJudgeObstacleParm){
    std::cerr<<"起点在障碍里面"<<std::endl;
    return -1;
  }

  CPose2D pose_point;
  for(int i=MinAngle;i<=MaxAngle;i++){
      pose_point=RobotPose;
      pose_point.phi_angle(pose_point.phi_angle()+i);
      pose_point = extendUntilHaveObstacle(Map,pose_point);
      LasterPose.push_back(pose_point);
      Laster.push_back(RobotPose.twoCPose2DDis(pose_point));
  }

  return 0;
}

int laserDataDeal::getLasterDataMax( std::vector<double> &Laster, std::vector<CPose2D> &LasterPose,
                                    double &Maxlong, CPose2D &MaxPose){

  if(Laster.empty()||LasterPose.empty()){
    std::cerr<<"laster or te is empty"<<std::endl;
    return -1;
  }
  if(Laster.size()!=LasterPose.size()){
    std::cerr<<"laster data is error"<<std::endl;
    return -1;
  }
  Maxlong = Laster[0];
  double maxFlag = 0;
  for(int i=0;i<Laster.size();i++){
      if(Maxlong<Laster[i]){
          Maxlong = Laster[i];
          maxFlag = i;
      }
  }
  MaxPose = LasterPose[maxFlag];
  return 0;
}

int laserDataDeal::getLasterDataMin( std::vector<double> &Laster, std::vector<CPose2D> &LasterPose,
                                    double &MinLong, CPose2D &MinPose){

  if(Laster.empty()||LasterPose.empty()){
    std::cerr<<"laster or te is empty"<<std::endl;
    return -1;
  }
  if(Laster.size()!=LasterPose.size()){
    std::cerr<<"laster data is error"<<std::endl;
    return -1;
  }
  MinLong = Laster[0];
  double minFlag = 0;

    for(int i=1;i<Laster.size();i++){
        if(MinLong>Laster[i]){
            MinLong = Laster[i];
            minFlag = i;
        }
    }



  MinPose = LasterPose[minFlag];
  return 0;
}







int laserDataDeal::getThreeDirectorPoint(cv::Mat &Map, const CPose2D &RobotPose,
                                         CPose2D &FrontPose, CPose2D &LeftPose, CPose2D &RightPose){
  if(Map.channels()==3)
   cv::cvtColor(Map,Map,CV_BGR2GRAY);

  if(Map.at<uchar>(RobotPose.y(),RobotPose.x())<mJudgeObstacleParm){
    return -1;
  }
  CPose2D frontIn(RobotPose),leftIn(RobotPose),rightIn(RobotPose);

  frontIn.phi_angle(RobotPose.phi_angle());
  leftIn.phi_angle(leftIn.phi_angle()-90);
  rightIn.phi_angle(rightIn.phi_angle()+90);


  FrontPose = extendUntilHaveObstacle(Map,frontIn);
  LeftPose = extendUntilHaveObstacle(Map,leftIn);
  RightPose = extendUntilHaveObstacle(Map,rightIn);

  return 0;
}

/* 得到激光数据  地图 当前位姿 前方 左方 右方*/
int laserDataDeal::getThreeDirectorDis( cv::Mat& Map,const CPose2D& RobotPose,double & Front,
                                        double & Left, double &Right){

    if(Map.channels()==3)
     cv::cvtColor(Map,Map,CV_BGR2GRAY);

    if(Map.at<uchar>(RobotPose.y(),RobotPose.x())<mJudgeObstacleParm){
      return -1;
    }
    CPose2D frontIn(RobotPose),leftIn(RobotPose),rightIn(RobotPose);
    CPose2D frontOut,leftOut,rightOut;
    double mfront,mleft,mright;
    frontIn.phi_angle(RobotPose.phi_angle());
    leftIn.phi_angle(leftIn.phi_angle()-90);
    rightIn.phi_angle(rightIn.phi_angle()+90);


    frontOut = extendUntilHaveObstacle(Map,frontIn);
    leftOut = extendUntilHaveObstacle(Map,leftIn);
    rightOut = extendUntilHaveObstacle(Map,rightIn);
    std::cout<<"frontIn"<<frontIn<<"frontOut"<<frontOut<<std::endl;
    std::cout<<"rightIn"<<rightIn<<"rightIn"<<rightIn<<std::endl;
    mfront = RobotPose.twoCPose2DDis(frontOut);
    mleft = RobotPose.twoCPose2DDis(leftOut);
    mright = RobotPose.twoCPose2DDis(rightOut);

    Front = mfront*0.05;
    Left = mleft*0.05;
    Right = mright*0.05;


    return 0;
}


int laserDataDeal::getLenAsLaserAngle(const std::vector<double> &Laser, const double &MinAngle, const double &MaxAngle,
                                      const double &Angle, double &Len){


//    std::cout<<"Laser.size() "<< Laser.size() <<std::endl;

 std::vector<double>data;     //保存激光10个激光数据的容器
 double lenth = Laser.size()/(MaxAngle-MinAngle);   /*每度的步数*/
 double centerNum = round((Angle-MinAngle)*lenth);  /*所求角度对应的中心位置*/

  data.clear();

  if(centerNum>5){

    for(int i=-5;i<6;i++){
      data.push_back(Laser[centerNum+i]);
    }
    std::sort(data.begin(),data.end());
    Len = (data[4]+data[5]+data[6])/3;

  }else if(centerNum>1){

    for(int i=-centerNum;i<centerNum+1;i++){
      data.push_back(Laser[centerNum+i]);
    }
    std::sort(data.begin(),data.end());
    Len = data[centerNum]/3;
  }else {
     std::cout<<"centerNum"<<centerNum<<std::endl;
    std::cerr<<"所取的点不合适"<<std::endl;
    return -1;
  }

  return 0;
}

/* 机器人前方障碍点  地图  机器人位姿*/
CPose2D laserDataDeal::extendUntilHaveObstacle(const cv::Mat& Map,const CPose2D &RobotPose){

  /**如果图像为三通道转换为单通道**/
  if(Map.channels()==3)
    cv::cvtColor(Map,Map,CV_BGR2GRAY);

    /**前方无障碍点方位**/
    CPose2D Point;
    Point.phi(RobotPose.phi());   /*方向为机器人朝向*/

    /**地图的 高和宽**/
    int height = Map.rows;
    int width = Map.cols;

     /**角度在不同的范围不一样**/
    if(RobotPose.phi_angle()==0){     /**角度为0时**/
      for(int i=0;i<=RobotPose.y();i++){
        if(Map.at<uchar>(RobotPose.y()-i,RobotPose.x())<mJudgeObstacleParm){
          Point.y(RobotPose.y()-i);
          Point.x(RobotPose.x());
          return Point;
        }
      }
      Point.y(0);
      Point.x(RobotPose.x());
      return Point;

    }else if(RobotPose.phi_angle()==180){   /**角度为180时**/
      for(int i=0;i<=(height-RobotPose.y()-1);i++){
        if(Map.at<uchar>(RobotPose.y()+i,RobotPose.x())<mJudgeObstacleParm){
          Point.y(RobotPose.y()+i);
          Point.x(RobotPose.x());
          return Point;
        }
      }
      Point.y(height-1);
      Point.x(RobotPose.x());
      return Point;
    }else if(RobotPose.phi_angle()==90){    /**角度为90时**/
      for(int i=0;i<=(width-RobotPose.x()-1);i++){
        if(Map.at<uchar>(RobotPose.y(),RobotPose.x()+i)<mJudgeObstacleParm){
           Point.y(RobotPose.y());
           Point.x(RobotPose.x()+i);
           return Point;
         }
      }
      Point.y(RobotPose.y()-1);
      Point.x(width);
      return Point;

    }else if(RobotPose.phi_angle()==-90){   /**角度为-90时**/
      for(int i=0;i<=RobotPose.x();i++){
        if(Map.at<uchar>(RobotPose.y(),RobotPose.x()-i)<mJudgeObstacleParm){
           Point.y(RobotPose.y());
           Point.x(RobotPose.x()-i);
           return Point;
         }
     }
      Point.y(RobotPose.y());
      Point.x(0);
      return Point;
    }else if((abs(RobotPose.phi_angle())<=45)){ /** 在该范围y为自变量 x为因变量**/
      int num;
      /*根据边界范围计算自变量的范围*/
      if(RobotPose.phi_angle()<0){
         num = fabs(floor(RobotPose.x()/tan(RobotPose.phi())));
      }else if(RobotPose.phi_angle()>0){
         num = fabs(floor((width - RobotPose.x())/tan(RobotPose.phi())));
      }
      if(num>RobotPose.y())  num = RobotPose.y();

       /*遍历自变量改变时的点是否为障碍*/
      for(int i=0;i<=num;i++){
        if(Map.at<uchar>(RobotPose.y()-i,round(RobotPose.x()+i*(tan(RobotPose.phi()))))<mJudgeObstacleParm){
           Point.y(RobotPose.y()-i);
           Point.x(round(RobotPose.x()+i*(tan(RobotPose.phi()))));
           return Point;
         }
      }
      /*都为非障碍时返回边界点*/
      Point.y(RobotPose.y()-num);
      Point.x(round(RobotPose.x()+num*(tan(RobotPose.phi()))));
      return Point;


   }else if(abs(RobotPose.phi_angle())>=135){
      int num;
      if(RobotPose.phi_angle()<0){
         num = fabs(floor(RobotPose.x()/tan(RobotPose.phi())));
      }else if(RobotPose.phi_angle()>0){
         num = fabs(floor((width - RobotPose.x())/tan(RobotPose.phi())));
      }
      if(num>(height - RobotPose.y()))  num = (height - RobotPose.y());

      for(int i=0;i<=num;i++){

        if(Map.at<uchar>(RobotPose.y()+i,round(RobotPose.x()-i*(tan(RobotPose.phi()))))<mJudgeObstacleParm){
           Point.y(RobotPose.y()+i);
           Point.x(round(RobotPose.x()-i*(tan(RobotPose.phi()))));
           return Point;
        }
     }
      Point.y(RobotPose.y()+num);
      Point.x(round(RobotPose.x()-num*(tan(RobotPose.phi()))));
      return Point;
    }else if((RobotPose.phi_angle()<135)&&(RobotPose.phi_angle()>45)){
      int num;
      if(RobotPose.phi_angle()<90){
         num = fabs(floor(RobotPose.y()*tan(RobotPose.phi())));
      }else if(RobotPose.phi_angle()>90){
         num = fabs(floor((height - RobotPose.y())*tan(RobotPose.phi())));
      }
      if(num>(width - RobotPose.x()))  num = (width - RobotPose.x());

      for(int i=0;i<num;i++){
        if(Map.at<uchar>(round(RobotPose.y()-i/(tan(RobotPose.phi()))),RobotPose.x()+i)<mJudgeObstacleParm){
           Point.y(round(RobotPose.y()-i/(tan(RobotPose.phi()))));
           Point.x(RobotPose.x()+i);
           return Point;
         }
     }
      Point.y(round(RobotPose.y()-num/(tan(RobotPose.phi()))));
      Point.x(RobotPose.x()+num);
      return Point;
    }else if((RobotPose.phi_angle()<-45)&&(RobotPose.phi_angle()>-135)){
      int num;
      if(RobotPose.phi_angle()>-90){
         num = fabs(floor(RobotPose.y()*tan(RobotPose.phi())));
      }else if(RobotPose.phi_angle()<-90){
         num = fabs(floor((height - RobotPose.y())*tan(RobotPose.phi())));
      }
      if(num>RobotPose.x())  num = RobotPose.x();

      for(int i=0;i<num;i++){
        if(Map.at<uchar>(round(RobotPose.y()+i/(tan(RobotPose.phi()))),RobotPose.x()-i)<mJudgeObstacleParm){
           Point.y(round(RobotPose.y()+i/(tan(RobotPose.phi()))));
           Point.x(RobotPose.x()-i);
           return Point;
        }
      }
      Point.y(round(RobotPose.y()+num/(tan(RobotPose.phi()))));
      Point.x(RobotPose.x()-num);
      return Point;
   }
}

/* 得到机器人下一个点位姿  地图  机器人位姿  前进长度*/
CPose2D laserDataDeal::PoseInFrontOfRobot(const cv::Mat& Map,const CPose2D &RobotPose,const double & Len){

  if(Map.channels()==3)
   cv::cvtColor(Map,Map,CV_BGR2GRAY);

   CPose2D Pose;

    int point_x,point_y;
    /*机器人坐标系  x=x0+detx  y=y0-dety */
    point_x = round(RobotPose.x()+Len*RobotPose.phi_sin());
    point_y = round(RobotPose.y()-Len*RobotPose.phi_cos());

    if(point_y<0) point_y =0;
    else if(point_y>Map.rows) point_y = Map.rows;

    if(point_x<0) point_x =0;
    else if(point_x>Map.cols) point_x = Map.cols;

    Pose.x(point_x);
    Pose.y(point_y);
    Pose.phi(RobotPose.phi());
    return Pose;
}

CPose2D laserDataDeal::PoseInRearOfRobot(const cv::Mat &Map, const CPose2D &RobotPose, const double &Len){


   CPose2D pose(RobotPose);
   pose.phi_angle(pose.phi_angle()+180);
   pose = PoseInFrontOfRobot(Map,pose,Len);
   pose.phi_angle(pose.phi_angle()+180);
  return pose;
}


/* 得到机器人下一个点位姿  地图  机器人位姿  旋转的角度*/
CPose2D laserDataDeal::nextCPose2DRo(const CPose2D &RobotPose,const double & Angle){
  CPose2D robotPose = RobotPose;
  robotPose.phi_angle(RobotPose.phi_angle()+Angle);
  return robotPose;
}

/*沿着某一方向寻找可通行的点   //地图左上角坐标系 */
int laserDataDeal::FindPassablePoint( cv::Mat&Map,const double angle,const cv::Point2d&RobotPose,cv::Point2d&nextPint){

//  std::cout<<"RobotPose "<< RobotPose <<std::endl;
//  std::cout<<"Map cols "<< Map.cols<<" rows"<<Map.rows <<std::endl;
//  cv::namedWindow("test11");
//  cv::imshow("test11",Map);
//  cv::waitKey(10);

  if(Map.channels()==3)       //灰度图
      cv::cvtColor(Map,Map,CV_BGR2GRAY);

  cv::Point2d robotPose;    //Point 点
  std::vector<double> vAngle;       //可通行点与初始方向的夹角
  std::vector<cv::Point2d> vPoint;    //存放周围可通行点

   Map.at<uchar>(RobotPose.y,RobotPose.x) = 0;  //将机器人所在位置标记为不可通行的点

   if(RobotPose.y-1>0){
       if(Map.at<uchar>(RobotPose.y-1,RobotPose.x)>160){    //正上      /*首先检测上下左右*/
          robotPose.x = RobotPose.x;
          robotPose.y = RobotPose.y-1;
          vPoint.push_back(robotPose);
          vAngle.push_back(0);
          std::cout<<"正上"<<std::endl;
       }
   }else{
     std::cerr<<"到达边界"<<std::endl;
     nextPint = RobotPose;
     return -2;
   }

   if(RobotPose.y+1<Map.rows){
       if(Map.at<uchar>(RobotPose.y+1,RobotPose.x)>160){    //正下
         robotPose.x = RobotPose.x;
         robotPose.y = RobotPose.y+1;

         vPoint.push_back(robotPose);
         vAngle.push_back(180);
         std::cout<<"正下"<<std::endl;
       }
   }else{
     std::cerr<<"到达边界"<<std::endl;
     nextPint = RobotPose;
     return -2;
   }
   if(RobotPose.x-1>0){
       if(Map.at<uchar>(RobotPose.y,RobotPose.x-1)>160){    //左
         robotPose.x = RobotPose.x-1;
         robotPose.y = RobotPose.y;
         vPoint.push_back(robotPose);
         vAngle.push_back(-90);
         std::cout<<"左"<<std::endl;
       }
   }else{
     std::cerr<<"到达边界"<<std::endl;
     nextPint = RobotPose;
     return -2;
   }
  if(RobotPose.x+1<Map.cols){
       if(Map.at<uchar>(RobotPose.y,RobotPose.x+1)>160){    //右
         robotPose.x = RobotPose.x+1;
         robotPose.y = RobotPose.y;
         vPoint.push_back(robotPose);
         vAngle.push_back(90);
         std::cout<<"右"<<std::endl;
       }
  }else{
    std::cerr<<"到达边界"<<std::endl;
    nextPint = RobotPose;
    return -2;
  }
  if(vAngle.size()<1){         /*上下左右无可通行区域时检测*/

     if((RobotPose.x-1>0)&&(RobotPose.y-1>0)){
         if(Map.at<uchar>(RobotPose.y-1,RobotPose.x-1)>160){  //左上
            robotPose.x = RobotPose.x-1;
            robotPose.y = RobotPose.y-1;
            vPoint.push_back(robotPose);
            vAngle.push_back(-45);
            std::cout<<"左上"<<std::endl;
         }
    }else{
       std::cerr<<"到达边界"<<std::endl;
       nextPint = RobotPose;
       return -2;
    }
     if((RobotPose.x+1<Map.cols)&&(RobotPose.y-1>0)){
         if(Map.at<uchar>(RobotPose.y-1,RobotPose.x+1)>160){    //右上
            robotPose.x = RobotPose.x+1;
            robotPose.y = RobotPose.y-1;
            vPoint.push_back(robotPose);
            vAngle.push_back(45);
            std::cout<<"右上"<<std::endl;
         }
     }else{
       std::cerr<<"到达边界"<<std::endl;
       nextPint = RobotPose;
       return -2;
     }
     if((RobotPose.y+1<Map.rows)&&(RobotPose.x-1>0)){
         if(Map.at<uchar>(RobotPose.y+1,RobotPose.x-1)>160){    //左下
           robotPose.x = RobotPose.x-1;
           robotPose.y = RobotPose.y+1;
           vPoint.push_back(robotPose);
           vAngle.push_back(-135);
           std::cout<<"左下"<<std::endl;
         }
     }else{
       std::cerr<<"到达边界"<<std::endl;
       nextPint = RobotPose;
       return -2;
     }
     if((RobotPose.y+1<Map.rows)&&(RobotPose.x+1<Map.cols)){
        if(Map.at<uchar>(RobotPose.y+1,RobotPose.x+1)>160){    //右下
          robotPose.x = RobotPose.x+1;
          robotPose.y = RobotPose.y+1;
          vPoint.push_back(robotPose);
          vAngle.push_back(135);
          std::cout<<"右下"<<std::endl;
        }
     }else{
       std::cerr<<"到达边界"<<std::endl;
       nextPint = RobotPose;
       return -2;
     }
  }

  if(vAngle.size()<1){
     std::cerr<<"周围没有可通行点"<<std::endl;
     nextPint = RobotPose;
     std::cout<<"RobotPose "<< RobotPose <<std::endl;
     std::cout<<"nextPint "<< nextPint <<std::endl;
     return -3;
   }

  for(int i=0;i<vAngle.size();i++){
    if(fabs(vAngle[i]-angle)<=180)
      vAngle[i] = fabs(vAngle[i]-angle);
    else
      vAngle[i] =360 - fabs(vAngle[i]-angle);
  }
  int flag=0; double minAngle = vAngle[0];
  for(int i=1;i<vAngle.size();i++){
    if(minAngle>vAngle[i]){
       minAngle = vAngle[i];
       flag = i;
     }
  }
  nextPint = vPoint[flag];

  return 0;
}









/* 膨胀障碍距离  地图  膨胀长度*/
int laserDataDeal::expendGridMapRelyNum(cv::Mat &gridMap,cv::Mat& expendGridMap,int expendGridMapNum){

     if(gridMap.channels()==3)
         cv::cvtColor(gridMap,gridMap,CV_BGR2GRAY);

     int height = gridMap.rows;   //栅格地图的行数
     int width = gridMap.cols;    //栅格地图的列数

    /* //一次膨胀地图 //膨胀障碍地图 */
     cv::Mat expendGridMap1(height,width,CV_8UC1,125);

     for(int i=0;i<gridMap.rows;++i){
         for(int j=0;j<gridMap.cols;++j){
             if((gridMap.at<uchar>(i,j)>=125)&&(expendGridMap1.at<uchar>(i,j)>60))   //遍历栅格地图，将白色栅格存储在expendGridMap地图中
                 expendGridMap1.at<uchar>(i,j)=gridMap.at<uchar>(i,j);
             else if(gridMap.at<uchar>(i,j)<125)    //将黑色障碍膨胀runApathPlaning_expendGridsNum个栅格
             {
                 for(int m=-expendGridMapNum;m<=expendGridMapNum;++m)   //expendGridsNum为膨胀指数
                     for(int n=-expendGridMapNum;n<=expendGridMapNum;++n)
                     {
                         int row=i+m,col=j+n;
                         if(row<0)	row=0;
                         if(col<0)	col=0;
                         if(row>gridMap.rows-1)	row=gridMap.rows-1;
                         if(col>gridMap.cols-1)	col=gridMap.cols-1;

                         expendGridMap1.at<uchar>(row,col)= 0;
                     }
             }
         }
     }

     expendGridMap=expendGridMap1.clone();

     return 0;
}


/* 得到线段上所有的点  线段的起始点  存放所有点的容器  该直线的倾角*/
bool laserDataDeal::getLineAllPoint(const cv::Vec4i& Line, std::vector<cv::Point2d>&vPoint,double& Angle ){

  cv::Point2d start(cv::Point2d(Line[0],Line[1])),end(cv::Point2d(Line[2],Line[3]));
  std::cout<<start<<" -"<<end<<std::endl;
  int sub_x = abs(start.x-end.x);
  int sub_y = abs(start.y-end.y);

  Angle = atan2((start.x-end.x),(start.y-end.y))/3.14*180;

  bool isAbsXBigY=0;

  if(sub_x>=sub_y)
      isAbsXBigY=1;
  else if(sub_x<sub_y)
      isAbsXBigY=0;

  int x1,y1,x2,y2;
  if(isAbsXBigY){

   x1=start.x,   y1=start.y;
   x2=end.x,     y2=end.y;

  }else{

       x1=start.y,   y1=start.x;
       x2=end.y,     y2=end.x;
  }
  if(x1 == x2)                      // start.y?=end.y
  {                                 // start.y==end.y
      if(y1 == y2)                  // start.x?=end.x
      {                             // start.x==end.x
          return false;
      } else if(y1 > y2)              //start.x>end.x
      {
          int num=abs(y1-y2);
          for(int i=1;i<num;++i)
          {
              if(isAbsXBigY){
                  vPoint.push_back(cv::Point2d(x1,y1-i));
              }else {
                   vPoint.push_back(cv::Point2d(y1-i,x1));
              }
          }
      }
      else if(y1 < y2)              //start.x<end.x
      {
          int num=abs(y1-y2);
          for(int i=1;i<num;++i)
          {
               if(isAbsXBigY){
                   vPoint.push_back(cv::Point2d(x1,y1+i));
               }else{
                   vPoint.push_back(cv::Point2d(y1+i,x1));
               }
          }
      }
  }
  else if(x1 != x2)
  {
      float a=0,b=0;

      a=(float)(y1-y2)/(float)(x1-x2);
      b=(float)y1-a*(float)x1;

      if(x1 > x2)
      {
          for(int i=x1-1;i>x2;--i)
          {
              int j=round (a*i+b);

               if(isAbsXBigY){
                   vPoint.push_back(cv::Point2d(i,j));
               }else{
                   vPoint.push_back(cv::Point2d(j,i));
               }

          }
      }
      else if(x1 < x2)
      {
          for(int i=x1+1;i<x2;++i)
          {
              int j=round (a*i+b);
              if(isAbsXBigY){
                  vPoint.push_back(cv::Point2d(i,j));
              }else{
                  vPoint.push_back(cv::Point2d(j,i));
              }
          }
      }
  }

  return true;

}

  /* 判断一个点是否在一个线段上*/
bool laserDataDeal::onePointIfInOneLine(const cv::Point2d &start, const cv::Point2d &end, const cv::Point2d &point){
   cv::Vec4i line(start.x,start.y,end.x,end.y);
   std::vector<cv::Point2d> vPoint2d;
   double angle;
   getLineAllPoint(line,vPoint2d,angle);

   if(vPoint2d.size()<1){
     return false;
   }

   for(int i=0;i<vPoint2d.size();i++){
      if((vPoint2d[i].x==point.x)&&(vPoint2d[i].y==point.y)){
        return true;
      }
   }
   return false;
}


/* 一个点到容器中所有点的最近距离  起点 存放所有点的容器  最近点的坐标*/
double laserDataDeal::onePointToVectPointNearPoint(const cv::Point2d & RobotPoint, std::vector<cv::Point2d>&
                                                   vPoint,cv::Point2d& RePoint ){

  cv::Point2d centerPoint(RobotPoint);
  RePoint=centerPoint;
  double len = 0;
  if(vPoint.size()>0){
   len=getTwoPointDis(centerPoint,vPoint[0]);
   RePoint = vPoint[0];
  }

  for(int i=0;i<vPoint.size();i++){
      if(getTwoPointDis(centerPoint,vPoint[i])<len){
          len = getTwoPointDis(centerPoint,vPoint[i]);
          RePoint =vPoint[i];
      }
  }
  return len;
}

/* 得到某点到多条线段最近点的位姿  多条线段 起点  输出点*/
bool laserDataDeal::getPose2dFromVet4iNearPoint(std::vector<cv::Vec4i>&vVect4i,const CPose2D &InPose, CPose2D &OutPose){

  if(vVect4i.size()<0){
      return true;
  }

  double vMinAngle,endAngle;
  double vMinLen,endLen;
  cv::Point2d vMinPoint,endPoint;

  std::vector<cv::Point2d> vPoint;


  if(getLineAllPoint(vVect4i[0],vPoint,vMinAngle)){

      endLen = onePointToVectPointNearPoint(InPose.toPoint(),vPoint,vMinPoint);

      endAngle = vMinAngle;
      endPoint = vMinPoint;
  }

  for(std::vector<cv::Vec4i>::iterator it = vVect4i.begin();it!=vVect4i.end();it++){
      cv::Vec4i l = *it ;

      if(getLineAllPoint(l,vPoint,vMinAngle)){

          vMinLen = onePointToVectPointNearPoint(InPose.toPoint(),vPoint,vMinPoint);

          if(vMinLen<endLen){
              endAngle = vMinAngle;
              endPoint = vMinPoint;
          }
      }
  }
  OutPose.x(endPoint.x);
  OutPose.y(endPoint.y);
  OutPose.phi_angle(endAngle);

  std::cout<<"OutPose:"<<OutPose<<std::endl;


}

 /**两张图片的异或**/
int laserDataDeal::twoMaps_xor(const cv::Mat &map1, const cv::Mat &map2, cv::Mat &dstMap, const int &level){

  if((map1.channels()!=1)||(map2.channels()!=1)){
    std::cerr<<"two map is not one channel"<<std::endl;
    return -1;
  }

  if(map1.empty()||map2.empty()){
    std::cerr<<"two map have empty map"<<std::endl;
    return -1;
  }

  if(map1.cols!=map2.cols||map1.rows!=map2.rows){
    std::cerr<<"two map size are not the same"<<std::endl;
    return -1;
  }
  dstMap = map1.clone();
  int height = map1.rows;
  int width = map1.cols;
  int Num1 = 0, Num2 = 0;
  for(int i=0;i<height;i++)
    for(int j=0;j<width;j++){
      if(map1.at<uchar>(i,j)>level) Num1 =1; else Num1 =0;
      if(map2.at<uchar>(i,j)>level) Num2 =1; else Num2 =0;
      if((Num1+Num2)==1)
        dstMap.at<uchar>(i,j) = 255;
      else
        dstMap.at<uchar>(i,j) = 0;
    }
  return 0;
}

int laserDataDeal::getPointToLineMsgs(const CPose2D &LinePose, const cv::Point2d &Point, bool &thisInPointReal){
//  double
  return 0;
}
/***计算点到直线的距离***/
int laserDataDeal::getPointToLineMsgs(const cv::Point2d &start, const cv::Point2d &end, const cv::Point2d &Point,double&dist,double&angle){

  double angleSTP,angleSTE;
  double distSTP;

  angleSTE = getTwoPointAngleInMap(start,end);
  angleSTP = getTwoPointAngleInMap(end,Point);

  angle =angleSTP-angleSTE;
  if(angle>180) angle -=360;
  if(angle<=-180) angle +=360;

  distSTP = getTwoPointDis(end,Point);
  dist = distSTP*sin(angle*3.14/180);
  //  std::cout<<"angleSTE:"<<angleSTE<<std::endl;
  //  std::cout<<"angleSTP:"<<angleSTP<<std::endl;
//  std::cout<<"angle:"<<angle<<std::endl;
//  std::cout<<"dist:"<<dist<<std::endl;

  return 0;
}
/***判断直线与线段的焦点是否在另一条线段上***
* @brief 判断两条直线的焦点 是否在第二条线段上
* @param start1 直线1起点
* @param end1 直线1终点
* @param start2 直线2起点
* @param end2 直线2终点
* return 在 or不在
*/
bool laserDataDeal::twoLinesIfHaveSamePointInSecondLines(const cv::Point2d&start1, const cv::Point2d&end1,const cv::Point2d&start2, const cv::Point2d&end2,cv::Point2d&dstPoint2d){

  cv::Point new_point(0,0);
  float k1=0,k2=0,b1=0,b2=0;
  if((end1.x - start1.x)!=0){          //前两个点的横坐标不一样时 求取这两点的 点斜式
    k1=(float)(end1.y - start1.y)/(float)(end1.x - start1.x);
    b1=(float) start1.y- k1*(float)start1.x;
  }
  if((end2.x - start2.x)!=0){       //后两个点的横坐标不一样时 求取这两点的 点斜式
    k2 = (float)(end2.y - start2.y)/(float)(end2.x - start2.x);
    b2 = (float)start2.y-k2*(float)start2.x;
  }
  if(((end1.x - start1.x)!=0)&&((end2.x - start2.x)!=0)){  //假设两条直线斜率都存在时求交点

      if(k1 ==k2){
          return false;
      }else{
        new_point.x=round((b2-b1)/(k1-k2));

        new_point.y=round((b2*k1-b1*k2)/(k1-k2));}

  }else  if(((end1.x - start1.x)==0)&&((end2.x - start2.x)!=0)){   //假设第一条直线斜率不存在第二条直线斜率存在时求交点

       new_point.x= round (start1.x);

       new_point.y= round (k2*start1.x+b2);
  }else  if(((end1.x - start1.x)!=0)&&((end2.x - start2.x)==0)){    //假设第二条直线斜率不存在第一条直线斜率存在时求交点

       new_point.x=start2.x;

       new_point.y= round (k1*start2.x+b1);
  }else if(((end1.x - start1.x)==0)&&((end2.x - start2.x)==0)){      //假设两条斜率都不存在
       return false;
  }

  dstPoint2d = new_point;

  if(onePointIfInOneLine(start2,end2,dstPoint2d))
    return true;
  else return false;
}
/***两条直线的相似度***/
double laserDataDeal::twoLinesSimilarity(const cv::Point2d &start1, const cv::Point2d &end1,const cv::Point2d &start2, const cv::Point2d &end2){

  if(start2!=end1){
    std::cerr<<"Line error"<<std::endl;
  }
  double angle,dist;
  getPointToLineMsgs(start1,end1,end2,angle,dist);
  if(fabs(angle)>90)  angle=angle-90; else angle=90-angle;

  double simlarity = dist*(cos(angle/90));
  return simlarity;
}


int laserDataDeal::laserDataTransformPointCloudData( const double&minAngle,const double&maxAngle,const std::vector<double> &vLaserLong,std::vector<cv::Point2d> &vLaserPoint)
{

  if(vLaserLong.size()<=0){
    std::cerr<<"laser is empty"<<std::endl;
    return -1;
  }
  int num = vLaserLong.size();
  double resolution = (maxAngle-minAngle)/num;
//  std::cout<<"laser data size:"<<len<<std::endl;
  vLaserPoint.clear();
  for(int i=0;i<num;i++){
//    vLaserPoint[i].x = vLaserLong[i]*sin((-(len-1)/2+i)*mLaserResolutionParam);
//    vLaserPoint[i].y = vLaserLong[i]*cos((-(len-1)/2+i)*mLaserResolutionParam);
  }
  return 0;
}

  /**最小二乘法求取直线**/
int laserDataDeal::getLinesPoint2dSlope(std::vector<cv::Point2d>&vLinePoint2d,double& k,double& b){
  // 构建最小二乘问题
  ceres::Problem problem;
  double abc[2] = {0,0};
  for ( int i=0; i<vLinePoint2d.size(); i++ )
  {
      problem.AddResidualBlock (     // 向问题中添加误差项
      // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
          new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 2> (
              new CURVE_FITTING_COST ( vLinePoint2d[i].x, vLinePoint2d[i].y )
          ),
          nullptr,            // 核函数，这里不使用，为空
          abc                 // 待估计参数
      );
  }
  // 配置求解器
  ceres::Solver::Options options;     // 这里有很多配置项可以填
  options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
//  options.minimizer_progress_to_stdout = true;   // 输出到cout
  ceres::Solver::Summary summary;                // 优化信息
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres::Solve ( options, &problem, &summary );  // 开始优化
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
  std::cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;
//  for ( auto a:abc ) std::cout<<a<<" ";
  k=abc[0];b=abc[1];
  return 0;
}

int laserDataDeal::getANumaverage(std::vector<double> &vValue, double &average){
  // 构建最小二乘问题
  ceres::Problem problem;
  double abc[1] = {0};
  for ( int i=0; i<vValue.size(); i++ )
  {
      problem.AddResidualBlock (     // 向问题中添加误差项
      // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
          new ceres::AutoDiffCostFunction<CURVE_FITTING_COST1, 1, 1> (
              new CURVE_FITTING_COST1 ( vValue[i])
          ),
          nullptr,            // 核函数，这里不使用，为空
          abc                 // 待估计参数
      );
  }
  // 配置求解器
  ceres::Solver::Options options;     // 这里有很多配置项可以填
  options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
//  options.minimizer_progress_to_stdout = true;   // 输出到cout
  ceres::Solver::Summary summary;                // 优化信息
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres::Solve ( options, &problem, &summary );  // 开始优化
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
  std::cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;
//  for ( auto a:abc ) std::cout<<a<<" ";
  average =abc[0];
  return 0;
}


/* 将激光数据投影到栅格图上  初始地图  投影地图  机器人位姿  激光数据  激光起始角度*/
void laserDataDeal::getMapByLasterDataMappingToMap(const cv::Mat &Map,cv::Mat & map1, const CPose2D &RobotPose,
                                                  std::vector<double> & vLen,const int& Begin){
  map1 = Map.clone();

  for(int i=0;i<map1.rows;i++){
      for(int j=0;j<map1.cols;j++)
      {
          map1.at<char>(i,j)= 255;
      }
  }

  cv::Mat Tcw(3,3,CV_64FC1);
  cv::Mat Tpc(3,1,CV_64FC1),Tpw(3,1,CV_64FC1);


      Tcw.at<double>(0,0) = RobotPose.phi_cos();  Tcw.at<double>(0,1) = -RobotPose.phi_sin();  Tcw.at<double>(0,2) =  RobotPose.x();
      Tcw.at<double>(1,0) = RobotPose.phi_sin();  Tcw.at<double>(1,1) = RobotPose.phi_cos();  Tcw.at<double>(1,2) = RobotPose.y();
      Tcw.at<double>(2,0) = 0;  Tcw.at<double>(2,1) = 0;  Tcw.at<double>(2,2) = 1;

  for(int i=0;i<vLen.size();i++ ){

     Tpc.at<double>(0,0)= vLen[i]*sin(  double(Begin+i)/180.0*3.14  );
     Tpc.at<double>(1,0)= -vLen[i]*cos(  double(Begin+i)/180.0*3.14  );
     Tpc.at<double>(2,0)=1;

     Tpw = Tcw*Tpc;


       cv::circle(map1,cv::Point2d(Tpw.at<double>(0,0),Tpw.at<double>(1,0)),1,cv::Scalar(0,0,0),2);

  }
  cv::circle(map1, RobotPose.toPoint(), 2, cv::Scalar(0,255,0), 2, 8, 0);


//           cv::namedWindow("test2");
//           cv::imshow("test2",map1);
//           cv::waitKey(1);

}

/* 霍夫变换得到多条直线  地图  线段存储器*/
void laserDataDeal::thoughHoughLineGetLinePoint(cv::Mat & Map,std::vector<cv::Vec4i> &lines){

    cv::Mat midImage,dstImage;

    if(Map.channels()==3)
     cv::cvtColor(Map,Map,CV_BGR2GRAY);

    cv::Canny(Map,midImage,50,200,3);
    cv::cvtColor(midImage,dstImage,CV_GRAY2BGR);

    cv::HoughLinesP(midImage,lines,2,CV_PI/180,20,50,10);
    std::cout<<"lines: "<< lines.size() <<std::endl;

    for(size_t i=0;i<lines.size();i++){
        cv::Vec4d l = lines[i];
        cv::line(dstImage,cv::Point(l[0],l[1]),cv::Point(l[2],l[3]),cv::Scalar(186,88,255),1,CV_AA);
    }

      cv::namedWindow("dstImage");
      cv::imshow("dstImage",dstImage);
      cv::waitKey(1);
}
