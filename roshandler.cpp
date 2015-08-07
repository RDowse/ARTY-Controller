#include "roshandler.h"

ROSHandler::ROSHandler() {
    start();
}

void ROSHandler::run() {}

void ROSHandler::start() { initROS(); }

void ROSHandler::stop()
{
    deletePublishers();
    ros::shutdown();
}

void ROSHandler::initTimers()
{
    auto rosSpin = []
    {
        ros::spinOnce();
    };
    QObject::connect(&rosSpinTimer_, &QTimer::timeout, rosSpin);
    rosSpinTimer_.start(1000 / 60);
}

void ROSHandler::initROS()
{
    qDebug()<<"Attempting to init ros";
    if (!ros::isInitialized())
    {
        int ros_argc = 3;
        // get the devices local ip
        std::string localIP;
        foreach (const QHostAddress& address, QNetworkInterface::allAddresses())
        {
            if (address.protocol() == QAbstractSocket::IPv4Protocol
                && address != QHostAddress(QHostAddress::LocalHost))
                localIP = "__ip:=" + address.toString().toStdString();
        }
        qDebug()<<"found local IP";
        const char* ros_argv[] = { "nothing_important",
            "__master:=http://10.0.0.131:11311", localIP.c_str() };

        ros::init(ros_argc, const_cast<char**>(&ros_argv[0]),
            "android_ndk_native_cpp");
        ros::master::setRetryTimeout(ros::WallDuration(5));
        // reconnectTimerInit();
        qDebug() << "ROS initialised";
    }
}

void ROSHandler::deletePublishers() { publishers_.clear(); }

bool ROSHandler::isConnected() { return ros::master::check(); }
