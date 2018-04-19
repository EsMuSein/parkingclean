#ifndef LINE_H
#define LINE_H

#include <opencv.hpp>
#include <iostream>
#include <vector>
#include <laserdatadeal.h>

class Line
{
public:
  Line();
  Line(const cv::Point2d&start,const cv::Point2d&end);

  inline double Length(){ return mLength;}
  inline double Angle(){ return mAngle;}
  inline cv::Point2d start(){ return mStart;}
  inline cv::Point2d end(){ return mEnd;}

  int setLine(const cv::Point2d&start,const cv::Point2d&end);
  friend std::ostream & operator<<(std::ostream &o, Line &L);

private:
  cv::Point2d mStart;
  cv::Point2d mEnd;
  double mLength;
  double mAngle;


};

#endif // LINE_H


//    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//    double deltT = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
//    std::cout<<"listen speed time:"<<deltT<<std::endl;
