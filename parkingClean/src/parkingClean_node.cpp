#include <parkingclean.h>
using namespace std;
using namespace cv;
using namespace sensor_msgs;

int main(int argc,char** argv){

  ros::init(argc,argv,"parkingClean");


  ROS_INFO_STREAM("Program starting");

  parkingClean parkingclean;

  if(parkingclean.init())
  {
    parkingclean.spin();
  }else
  {
    ROS_ERROR_STREAM("Couldn't initialise parkingClean!");
  }
  ROS_INFO_STREAM("Program exiting");
}
