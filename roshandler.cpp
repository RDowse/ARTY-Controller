#include "roshandler.h"
#include <geometry_msgs/Twist.h>
#include <QTimer>
#include <QHostAddress>
#include <QNetworkInterface>

namespace
{
void sendMsg(ros::Publisher& pub, double linear, double angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;

    pub.publish(msg);
}

void rosSpin() { ros::spinOnce(); }

}//End of anonymous namespace


ROShandler::ROShandler(QObject* parent)
    : QObject(parent)
{
    rosInit();
}

ROShandler::ROShandler() { rosInit(); }

void ROShandler::timerInit()
{
    auto onTimeout = [this]()
    {
        ::sendMsg(vel_pub_, linear_, angular_);
    };
    QObject::connect(&pubTimer_, &QTimer::timeout, onTimeout);
    QObject::connect(&rosTimer_, &QTimer::timeout, rosSpin);
    pubTimer_.start(1000 / 20);
    rosTimer_.start(1000 / 60);
}

void ROShandler::rosInit()
{
    qDebug() << "Hello World!";

    if (!ros::isInitialized())
    {
        int ros_argc = 3;

        // get the devices local ip
        QString localIP;
        foreach (const QHostAddress& address, QNetworkInterface::allAddresses())
        {
            if (address.protocol() == QAbstractSocket::IPv4Protocol
                && address != QHostAddress(QHostAddress::LocalHost))
                localIP = "__ip:=" + address.toString();
        }

        const char* ros_argv[] = { "nothing_important",
            "__master:=http://10.0.0.92:11311", localIP.toStdString().c_str() };
        ros::init(ros_argc, const_cast<char**>(&ros_argv[0]),
            "android_ndk_native_cpp");
        ros::master::setRetryTimeout(ros::WallDuration(0.1));

        qDebug() << "ROS initialised";
        if (checkTopics())
        {
            qDebug() << "Creating nodehandle";
            ros::NodeHandle n;
            vel_pub_ = n.advertise<geometry_msgs::Twist>(
                "/navigation/main_js_cmd_vel", 1);
            timerInit();
        } else{
            qDebug() << "Failed to create nodehandle";
        }
    }
}

bool ROShandler::checkTopics()
{
    using ::operator ==;
    // check topics to see if the master is correct.
    ros::master::V_TopicInfo topiclist;
    ros::master::getTopics(topiclist);
    ros::master::TopicInfo rosout;
    rosout.name = "/rosout";
    rosout.datatype = "rosgraph_msgs/Log";

    auto comparator = [&rosout](const ros::master::TopicInfo& t){
        return rosout.name == t.name && rosout.datatype == t.datatype;
    };

    return std::find_if(std::begin(topiclist), std::end(topiclist), comparator)
        != std::end(topiclist);
}

void ROShandler::setVelAng(double linear, double angular)
{
    linear_ = linear;
    angular_ = angular;
    QString s = "Velocity: " + QString::number(linear) + " Angle: " + QString::number(angular);
    emit log(s);
}

void ROShandler::shutdownROS()
{
    ros::shutdown();
    emit log("The ROS node has been terminated");
}
