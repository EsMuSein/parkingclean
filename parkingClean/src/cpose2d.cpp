#include "cpose2d.h"


CPose2D::CPose2D() :x_value(0.), y_value(0.), phi_value(0.){
  // TODO Auto-generated constructor stub
}

CPose2D::CPose2D( cv::Point2d point2d){
   x_value = point2d.x;
   y_value = point2d.y;
   phi_value = 0.0;
}
double CPose2D:: phi_angle() const
{
     int angle =int (round(phi_value*180/3.14))%360;
     if(angle>180) angle -=360;
     if(angle<=-180) angle +=360;
      return angle;
}


std::ostream & operator<<(std::ostream &o,const  CPose2D &P){
    o<<"[ "<<P.x()<<",  "<<P.y()<<",  "<<P.phi_angle()<<" ]";
  return o;
}

double CPose2D::twoCPose2DDis(const CPose2D &CPose) const{

  return sqrt((this->x()- CPose.x())*(this->x()-CPose.x())+ (this->y()- CPose.y())*(this->y()-CPose.y()));
}

double CPose2D::twoCPose2DDisPoint(const cv::Point2d &CPoint2d) const{

  return sqrt((this->x()- CPoint2d.x)*(this->x()-CPoint2d.x)+ (this->y()- CPoint2d.y)*(this->y()-CPoint2d.y));
}
double CPose2D::twoCPose2DAngle(const CPose2D &b) const{
  int angle = fabs(this->phi_angle()-b.phi_angle());
  angle = angle%360;
  if(angle>180) angle -=360;
  if(angle<=-180) angle +=360;
   return angle;

}

bool CPose2D::twoCPose2DEqual(const CPose2D &CPose) const{

  if(this->x()!=CPose.x()){
    return false;
  }else if(this->y()!=CPose.y()){
    return false;
  }else if(this->phi_angle()!=CPose.phi_angle()){
    return false;
  }else {
    return true;
  }

}

CPose2D::CPose2D(const double& x_,const double& y_,const double& phi_){
   x_value = x_;
   y_value = y_;
   phi_value = phi_;
}

CPose2D::~CPose2D() {
  // TODO Auto-generated destructor stub
}

 /* 两点之间的距离*/
double getTwoPointDis(const cv::Point& a,const cv::Point& b){
//   std::cout<<" point"<<std::endl;
   return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}
double getTwoPointDis(const cv::Point2d& a,const cv::Point2d& b){
//   std::cout<<" point2d"<<std::endl;
   return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}
 /* 两点之间的角度*/
double getTwoPointAngleInMap(const cv::Point &start, const cv::Point &end){
//    std::cout<<" point"<<std::endl;
  double angle;
  if(start.x>end.x&&start.y>end.y)
      angle = -90+round((atan2(fabs(end.y-start.y),fabs(end.x-start.x)))/3.14*180);
  else if(start.x>end.x&&start.y<end.y)
      angle = -90-round((atan2(fabs(end.y-start.y),fabs(end.x-start.x)))/3.14*180);
  else if(start.x<end.x&&start.y>end.y)
      angle = 90-round((atan2(fabs(end.y-start.y),fabs(end.x-start.x)))/3.14*180);
  else if(start.x<end.x&&start.y<end.y)
      angle = 90+round((atan2(fabs(end.y-start.y),fabs(end.x-start.x)))/3.14*180);
  else if(start.x==end.x&&start.y<end.y)
      angle = -180;
  else if(start.x==end.x&&start.y>=end.y)
      angle = 0;
  else if(start.y==end.y&&start.x>end.x)
      angle = -90;
  else if(start.y==end.y&&start.x<end.x)
      angle = 90;
  return angle;
}
double getTwoPointAngleInMap(const cv::Point2d &start, const cv::Point2d &end){
//    std::cout<<" point2d"<<std::endl;
  double angle;
  if(start.x>end.x&&start.y>end.y)
      angle = -90+round((atan2(fabs(end.y-start.y),fabs(end.x-start.x)))/3.14*180);
  else if(start.x>end.x&&start.y<end.y)
      angle = -90-round((atan2(fabs(end.y-start.y),fabs(end.x-start.x)))/3.14*180);
  else if(start.x<end.x&&start.y>end.y)
      angle = 90-round((atan2(fabs(end.y-start.y),fabs(end.x-start.x)))/3.14*180);
  else if(start.x<end.x&&start.y<end.y)
      angle = 90+round((atan2(fabs(end.y-start.y),fabs(end.x-start.x)))/3.14*180);
  else if(start.x==end.x&&start.y<end.y)
      angle = -180;
  else if(start.x==end.x&&start.y>=end.y)
      angle = 0;
  else if(start.y==end.y&&start.x>end.x)
      angle = -90;
  else if(start.y==end.y&&start.x<end.x)
      angle = 90;
  return angle;
}
double getTwoPointAngleInRobot(const cv::Point &start, const cv::Point &end){
  double angle = getTwoPointAngleInMap(start, end);
   angle = - (angle -90);
  if(angle>180) angle -=360;
  if(angle<=-180) angle +=360;
  return angle;
}
double getTwoPointAngleInRobot(const cv::Point2d &start, const cv::Point2d &end){
  double angle = getTwoPointAngleInMap(start, end);
   angle = - (angle -90);
  if(angle>180) angle -=360;
  if(angle<=-180) angle +=360;
  return angle;
}
