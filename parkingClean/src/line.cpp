#include "line.h"

Line::Line():mStart(cv::Point2d(0,0)),mEnd(cv::Point2d(0,0))
{
  mLength = getTwoPointDis(mStart,mEnd);
  mAngle = getTwoPointAngleInMap(mStart,mEnd);

}
Line::Line(const cv::Point2d&start,const cv::Point2d&end):mStart(start),mEnd(end){
   mLength = getTwoPointDis(mStart,mEnd);
   mAngle = getTwoPointAngleInMap(mStart,mEnd);
}

int Line::setLine(const cv::Point2d &start, const cv::Point2d &end){
   mStart = start;
   mStart = end;
   mLength = getTwoPointDis(mStart,mEnd);
   mAngle = getTwoPointAngleInMap(mStart,mEnd);
  return 0;
}

std::ostream &operator<<(std::ostream&os, Line&L){

  os<<"start[ "<<L.start().x<<","<<L.start().y<<"],end["<<L.end().x<<","<<L.end().y<<" ]";
  return os;
}

