#include "mainwindow.h"
#include <QApplication>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdarg.h>
#include <stdio.h>
#include <sstream>
#include <map>
#include <string.h>
#include <errno.h>
#include <vector>
#include <set>
#include <fstream>
#include <android/log.h>

int loop_count_ = 0;
ros::Publisher chatter_pub;

// from android samples
/* return current time in seconds */
static double now(void) {

  struct timespec res;
  clock_gettime(CLOCK_REALTIME, &res);
  return res.tv_sec + (double) res.tv_nsec / 1e9;

}

#define LASTERR strerror(errno)

// %Tag(CHATTER_CALLBACK)%
void chatterCallback(const std_msgs::StringConstPtr& msg){
    ROS_INFO("%s", msg->data.c_str());
    loop_count_++;
    std_msgs::String msgo;
    std::stringstream ss;
    ss << "hello world from android ndk " << loop_count_;
    msgo.data = ss.str();
    chatter_pub.publish(msgo);
}
// %EndTag(CHATTER_CALLBACK)%

int main(int argc, char *argv[])
{
    int ros_argc = 3;
    char *ros_argv[] = {"nothing_important" , "__master:=http://10.0.0.147:11311", "__ip:=10.0.0.99"};
    // %EndTag(CONF_ARGS)%
    //strcpy(argv[0], 'nothing_important');
    //argv[1] = '__master:=http://10.52.90.103:11311';
    //argv[2] = '__ip:=10.52.90.246';
    //argv[3] = '__hostname:=10.52.90.246';

    // %Tag(ROS_INIT)%
    ros::init(ros_argc, &ros_argv[0], "android_ndk_native_cpp");


    // %Tag(ROS_MASTER)%
    std::string master_uri = ros::master::getURI();

    ros::NodeHandle n;
    // %EndTag(ROS_MASTER)%

    // %Tag(ROS_CONF_SUB_PUB)%
    chatter_pub = n.advertise<std_msgs::String>("a_chatter", 1000);
    //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    for (int i=0; i< 100000; ++i)
    {
        std_msgs::String msg;
        msg.data = "Hello Toby!!!!";
        chatter_pub.publish(msg);
        ros::spinOnce();
    }
    // %EndTag(ROS_CONF_SUB_PUB)%

    // %Tag(ROS_SPIN)%
   // while(ros::ok() && !papp->destroyRequested){
   //     ros::spinOnce();
    //    loop_rate.sleep();
 //   }
    // %EndTag(ROS_SPIN)%

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
