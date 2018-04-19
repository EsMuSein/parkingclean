#include <ros/ros.h>
#include <cstring>
#include <stdio.h>
#include <iostream>
#include <linux/input.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <client/myMsg.h>
#include <string>
#include <std_msgs/String.h>

 //sudo chmod 777 /dev/input/event3
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client_node");
  ros::NodeHandle nh;

  ros::Publisher client_pub = nh.advertise<client::myMsg>("/client_chatter",10);
  ros::Publisher string_pub = nh.advertise<std_msgs::String>("/string_chatter",10);
  ros::Rate loop_rate(20);

  std_msgs::String str1;
  string str = "2ae";
  client::myMsg msg;
//  client::myMsg last_msg;
  struct input_event t;
  int keys_fd;
  keys_fd=open("/dev/input/event3",O_RDONLY);
  if(keys_fd<=0)
  {
    ROS_ERROR_STREAM("client node can not open keyboard device !");
  }
  string st1="我按的键盘是:";

  while (ros::ok()) {

  read(keys_fd,&t,sizeof(struct input_event));

  if(t.type==EV_KEY){  //键盘实践

        static int code;
        code = (int) t.code;
        switch (code) {


        case 2:
            msg.mode = 0;
            msg.mode_key =string(st1+"1");
            str = "1a1";
            break;

        case 3:
          msg.mode = 1;
          msg.mode_key =string(st1+"2");
           str = "1a2";
            break;

        case 4:
          msg.mode = 2;
          msg.mode_key =string(st1+"3");
          str = "1a3";
            break;
        case 5:
          msg.mode = 3;
          msg.mode_key =string(st1+"4");
          str = "1a4";
            break;

        case 16:
          msg.operate = 0;
          msg.operate_key = string(st1+"q");
          str = "2aq";
            break;

        case 17:
          msg.operate = 1;
          msg.operate_key = string(st1+"w");
          str = "2aw";
            break;

        case 18:
          msg.operate = 2;
          msg.operate_key = string(st1+"e");
          str = "2ae";
            break;

        case 19:
          msg.operate = 3;
          msg.operate_key = string(st1+"r");
          str = "2ar";
            break;
        case 20:
          msg.operate = 4;
          msg.operate_key = string(st1+"t");
          str = "2at";
            break;
        case 57:
          msg.operate = 4;
          msg.operate_key = string(st1+" ");
          str = "2a ";
            break;
        case 30:
          msg.operate = 4;
          msg.operate_key = string(st1+"a");
          str = "2aa";
            break;
        case 32:
          msg.operate = 4;
          msg.operate_key = string(st1+"d");
          str = "2ad";
            break;
        case 31:
          msg.operate = 4;
          msg.operate_key = string(st1+"s");
          str = "2as";
            break;
        default:           
//            ROS_ERROR_STREAM("key is invalid");
            break;
        }
      }
//        std::cout<<" "<<str.data<<std::endl;

        str1.data = str;

        string_pub.publish(str1);

        client_pub.publish(msg);

//      if((last_msg.mode!=msg.mode)||(last_msg.operate!=msg.operate)){
//        ROS_ERROR_STREAM("+++++++++++++++++++++!");
//        client_pub.publish(msg);
//        last_msg.mode = msg.mode;
//        last_msg.operate = msg.operate;
//      }else{
//        ROS_INFO_STREAM("mode and operate is not change!");
//      }

        ros::spinOnce();
        loop_rate.sleep();
  }
  close(keys_fd);
  ROS_INFO("KEY is exiting!");
}
