#include "roshandler.h"
#include <geometry_msgs/Twist.h>
#include <QTimer>

namespace
{
    void sendMsg(ros::Publisher& pub, double linear, double angular)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = linear;
        msg.angular.z = angular;

        pub.publish(msg);
    }

    void rosSpin()
    {
        ros::spinOnce();
    }
}

ROShandler::ROShandler(QObject *parent):
    QObject(parent)
{
    timerInit();
    rosInit();
}

ROShandler::ROShandler()
{
    timerInit();
    rosInit();
}

void ROShandler::timerInit()
{
    auto onTimeout = [this](){ ::sendMsg(vel_pub_, linear_, angular_);};
    QObject::connect(&pubTimer_, &QTimer::timeout, onTimeout);
    QObject::connect(&rosTimer_, &QTimer::timeout, rosSpin);
    pubTimer_.start(1000/20);
    rosTimer_.start(1000/60);
}

void ROShandler::rosInit()
{
    int ros_argc = 3;
    char *ros_argv[] = {"nothing_important" , "__master:=http://10.0.1.3:11311", "__ip:=10.0.1.4"};
    ros::init(ros_argc, &ros_argv[0], "android_ndk_native_cpp");
    ros::NodeHandle n;
    vel_pub_ = n.advertise<geometry_msgs::Twist>("/arty_navigation/main_js_cmd_vel", 1);
}

void ROShandler::setVelAng(double linear, double angular)
{
    linear_ = linear;
    angular_ = angular;
    emit log("Velocity set");
}
