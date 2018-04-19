#ifndef CPOSE2D_H
#define CPOSE2D_H
#include <opencv/cv.hpp>
#include <iostream>
/* 机器人位姿的三维表示*/
class CPose2D
{
public:
  CPose2D();
  CPose2D( cv::Point2d );
  CPose2D(const double& ,const double& ,const double& = 0);
  CPose2D &operator =(const CPose2D &Right){
    this->x_value = Right.x();
    this->y_value = Right.y();
    this->phi_value = Right.phi();
    return *this;
  }
  virtual ~CPose2D();
  inline double x() 				const 	{		return x_value;}
  inline double y() 				const 	{		return y_value;	}
  inline double phi()			const 	{	return phi_value;}
  double phi_angle() const;
  inline double phi_cos() const	{ return (double)cos(phi_value);}
  inline double phi_sin()	const	{ return (double)sin(phi_value);}
  inline double phi_tan() const { return (double)tan(phi_value);}
  inline void x(const double & x_){				x_value = static_cast<double>(x_);}
  inline void y(const double & y_){				y_value = static_cast<double>(y_);	}
  inline void phi(const double&phi_){		phi_value = static_cast<double>(phi_);	}
  inline void phi_angle(const double&phi_angle_) { phi_value = static_cast<double>(phi_angle_/180*3.1415);}
  friend std::ostream & operator<<(std::ostream &o,const CPose2D &P);
  bool twoCPose2DEqual(const CPose2D &) const;
  double twoCPose2DDis(const CPose2D &) const;
  double twoCPose2DDisPoint(const cv::Point2d &) const;
  double twoCPose2DAngle(const CPose2D &) const;
  inline cv::Point2d toPoint()const { cv::Point2d point(round(this->x()),round(this->y()));return point;  }

private:
  double x_value;
  double y_value;
  double phi_value;//the angle to the x-axis (in radians)
};
  /*两点之间的距离*/
double getTwoPointDis(const cv::Point& a,const cv::Point& b);
double getTwoPointDis(const cv::Point2d& a,const cv::Point2d& b);
  /*两点之间的角度*/
double getTwoPointAngleInMap(const cv::Point& start,const cv::Point& end);
double getTwoPointAngleInMap(const cv::Point2d& start,const cv::Point2d& end);

double getTwoPointAngleInRobot(const cv::Point& start,const cv::Point& end);
double getTwoPointAngleInRobot(const cv::Point2d& start,const cv::Point2d& end);


namespace Enum {

  typedef enum modes{

    emIdle = 0,
    emWork  =1,
    emTest =2
  }modes;

  typedef enum states{
    esWalkByWall,       //沿墙走
    esWalkNearWall,     //靠近墙
    esParkingClean,     //清扫

    esNavigateOneToOne,   //一个点到一个点

  }states;

  typedef enum sstates{

    essGoStraight,
    essTurnLeft,
    essTurnRight,
    essStop,

    essCleanTurnIntoCar,
    essCleanGoIntoCar,
    essCleanTurnApeakCar,
    essCleanGoCleanDist,
    essCleanTurnBacktoCar,
    essCleanGoBacktoCar,
    essCleanTurntoStart,
    essCleanForwardCarDist,

  essCleanTurnIntoCar1,     /*旋转向车位*/
  essCleanGoIntoCar1,       /*进入车位*/
  essCleanGoBacktoCar1,     /*推出清扫*/
  essCleanNormalClean,      /*正常清扫 计算点*/
  essCleanNormalStart,      /*正常清扫  开始清扫*/

}sstates;


  typedef enum rControls{
    rStop =0,
    rFront = 1,
    rRear = 2,
    rLeft =3,
    rRight =4,
    rLeftRotate = 5,
    rRightRotate =6
  }rControls;

}

namespace Struct {

  typedef struct sDists{
    double L = 0;
    double R = 0;
    double F = 0;

  }sDists;

}

struct CURVE_FITTING_COST   /**最小二乘法求取直线**/
{
    CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const abc,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - ( abc[0]*T ( _x )  + abc[1] ); // y-exp(ax^2+bx+c)
        //        residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
        return true;
    }
    const double _x, _y;    // x,y数据
};
struct CURVE_FITTING_COST1    /**最小二乘法求取一个均值**/
{
    CURVE_FITTING_COST1 ( double x ) : _x ( x ){}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const abc,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _x ) - ( abc[0]); // y-exp(ax^2+bx+c)

        return true;
    }
    const double _x;    // x,y数据
};
#endif // CPOSE2D_H
