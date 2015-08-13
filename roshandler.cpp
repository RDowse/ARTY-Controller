#include "roshandler.h"

ROSHandler::ROSHandler()
{
    qDebug() << "Creating ROSHandler";
    connected_ = true;
    running_ = true;
    masterIP_ = "10.0.0.127";
    initSpinTimer();

    ros::master::setRetryTimeout(ros::WallDuration(5));

    start();
}

ROSHandler::~ROSHandler()
{
    qDebug() << "Destroying ROSHandler";
    running_ = false;
    thread_.join();
}

void ROSHandler::start() { thread_ = std::thread(&ROSHandler::run, this); }

void ROSHandler::run()
{

    while (running_)
    {

        {
            std::lock_guard<decltype(mtx_)> lock(mtx_);
            connected_ = isConnected();

            if (!node_)
            {
                qDebug() << "--Connecting ROS--";
                startROS();
            }
            else if ((!connected_ && node_)
                || ros::master::getHost() != masterIP_)
            {
                stopROS();
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}

void ROSHandler::stopROS()
{
    std::lock_guard<decltype(mtx_)> lock(mtx_);
    // Delete the publishers and the nodehandle.
    qDebug() << "Stopping ROS";
    deletePublishers();
    ros::shutdown();
    ros::waitForShutdown();

    node_.reset();

    qDebug() << "Node successfully shutdown";
}

void ROSHandler::initSpinTimer()
{
    auto onTimeout = [this]()
    {
        if (connected_ && node_)
        {
            ros::spinOnce();
        }
    };

    QObject::connect(&rosSpinTimer_, &QTimer::timeout, onTimeout);

    rosSpinTimer_.start(1000 / 60);
}

void ROSHandler::startROS()
{
    std::lock_guard<decltype(mtx_)> lock(mtx_);
    qDebug() << "Attempting to init ros";

    int ros_argc = 3;
    // get the devices local ip
    std::string localIP;
    foreach (const QHostAddress& address, QNetworkInterface::allAddresses())
    {
        if (address.protocol() == QAbstractSocket::IPv4Protocol
            && address != QHostAddress(QHostAddress::LocalHost))
            localIP = "__ip:=" + address.toString().toStdString();
    }
    qDebug() << "Found local IP " << localIP.data();

    std::string tempMasterIP = "__master:=http://" + masterIP_ + ":11311";
    const char* ros_argv[]
        = { "nothing_important", tempMasterIP.c_str(), localIP.c_str() };
    //TODO: look at how to change the master URI at runtime
    ros::init(ros_argc, const_cast<char**>(&ros_argv[0]), "android_controller",
        ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);

    qDebug() << "Creating nodehandle";
    if (isConnected())
    {
        node_.reset(new ros::NodeHandle());
        qDebug() << "Nodehandle created";
        qDebug() << "Node ok? " << node_->ok();
    }
    else
    {
        qDebug() << "Unable to create nodehandle";
    }

    qDebug() << "ROS initialised";
}

void ROSHandler::deletePublishers()
{
    std::lock_guard<decltype(mtx_)> lock(mtx_);
    qDebug() << "Deleting publishers";
    publishers_.clear();
}

bool ROSHandler::isConnected() { return ros::master::check(); }

void ROSHandler::setMasterIP(std::string masterIP) { masterIP_ = masterIP; }

std::string ROSHandler::getMasterIP() const { return masterIP_; }
