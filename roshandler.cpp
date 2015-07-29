#include "roshandler.h"
#include <geometry_msgs/Twist.h>

#include <QTimer>
#include <QHostAddress>
#include <QNetworkInterface>
#include <string>
namespace
{
void sendMsgVel(ros::Publisher& pub, double linear, double angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;

    pub.publish(msg);
}

void sendMsgJoy(ros::Publisher& pub,  double linear, double angular){
    sensor_msgs::Joy msg;
    msg.axes.push_back(linear);
    msg.axes.push_back(angular);
    pub.publish(msg);
}

void rosSpin() { ros::spinOnce(); }

} // End of anonymous namespace

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
        //::sendMsgVel(vel_pub_, linear_, angular_);
        ::sendMsgJoy(joystick_pub_, linear_, angular_);
    };
    QObject::connect(&pubTimer_, &QTimer::timeout, onTimeout);
    QObject::connect(&rosTimer_, &QTimer::timeout, rosSpin);
    pubTimer_.start(1000 / 20);
    rosTimer_.start(1000 / 60);
}

void ROShandler::rosInit()
{
//    if (!ros::isInitialized())
//    {
        int ros_argc = 3;
        emit log("Attempting to connect");
        // get the devices local ip
        std::string localIP;
        foreach (const QHostAddress& address, QNetworkInterface::allAddresses())
        {
            if (address.protocol() == QAbstractSocket::IPv4Protocol
                && address != QHostAddress(QHostAddress::LocalHost))
                localIP = "__ip:=" + address.toString().toStdString();
        }
//localIP.c_str()
        const char* ros_argv[] = { "nothing_important",
            "__master:=http://10.0.1.3:11311", "__ip:=10.0.1.4"};
        ros::init(ros_argc, const_cast<char**>(&ros_argv[0]),
            "android_ndk_native_cpp");
        ros::master::setRetryTimeout(ros::WallDuration(5));
        qDebug() << "ROS initialised";
//    }

//    if (checkTopics())
//    {
        qDebug() << "Creating nodehandle";
        ros::NodeHandle n;

//        vel_pub_ = n.advertise<geometry_msgs::Twist>(
//            "/navigation/main_js_cmd_vel", 1);
        joystick_pub_ = n.advertise<sensor_msgs::Joy>(
                    "/arty/joystick", 1);
        map_sub_ = n.subscribe("/arty_navigation/map", 1, &ROShandler::callback, this);
        //map_meta_sub_ = n.subscribe("/arty_navigation/amcl_pose", 1, &ROShandler::callbackPos, this);
        timerInit();
        emit log("Successfully connect ROS node");
//    }
//    else
//    {
//        qDebug() << "Failed to create nodehandle";
//        emit log("Failed to create nodehandle");
//    }
}

bool ROShandler::checkTopics()
{
    using ::operator==;
    // check topics to see if the master is correct.
    ros::master::V_TopicInfo topiclist;
    ros::master::getTopics(topiclist);
    ros::master::TopicInfo rosout;
    rosout.name = "/rosout";
    rosout.datatype = "rosgraph_msgs/Log";

    auto comparator = [&rosout](const ros::master::TopicInfo& t)
    {
        return rosout.name == t.name && rosout.datatype == t.datatype;
    };

    return std::find_if(std::begin(topiclist), std::end(topiclist), comparator)
        != std::end(topiclist);
}

void ROShandler::setVelAng(double linear, double angular)
{
    linear_ = linear;
    angular_ = angular;
    QString s = "Velocity: " + QString::number(linear) + " Angle: "
        + QString::number(angular);
    emit log(s);
}

void ROShandler::restartROS()
{
    // attempt to initialise ros again
    rosInit();
}

void ROShandler::shutdownROS()
{
    ros::shutdown();
    emit log("The ROS node has been terminated");
}

void ROShandler::setMasterIP(QString masterIP) { masterIP_ = masterIP; }

QString ROShandler::getMasterIP() { return masterIP_; }

void ROShandler::callback(const nav_msgs::OccupancyGrid& msg)
{
    size_t dataSize = msg.info.width * msg.info.height;
    auto buffer = new uchar[dataSize];

    for (size_t i = 0; i < dataSize; ++i)
    {
        if (msg.data[i] == -1)
        {
            buffer[i] = 127.5;
        }
        else
        {
            buffer[i] = (100-msg.data[i]) * 2.55;
        }
    }

    auto deleter = [](void* info)
    {
        delete[] static_cast<std::vector<uchar>*>(info);
    };

    QImage map(buffer, msg.info.width, msg.info.height,
        QImage::Format_Grayscale8, deleter);
    map = map.mirrored();

    emit mapUpdate(map);
}

//void ROShandler::callbackPos(const geometry_msgs::PoseWithCovarianceStamped& msg){
//    emit posUpdate(msg.pose.position);
//}
