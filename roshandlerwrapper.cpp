#include "roshandlerwrapper.h"
#include <QDebug>
ROSHandlerWrapper::ROSHandlerWrapper()
{
    addPublishers();
    addSubscribers();
}

ROSHandlerWrapper::ROSHandlerWrapper(QObject* parent)
    : QObject(parent)
{
    addPublishers();
    addSubscribers();
}

void ROSHandlerWrapper::addPublishers()
{
    //Publish at 60hz.
    qDebug()<<"Adding publishers";
    auto onTimeout = [this]
    {
        roshandler_.publish("/arty/joystick", twistMsg_);
    };
    QObject::connect(&pubTimer_, &QTimer::timeout, onTimeout);
    pubTimer_.start(1000/60);
}

void ROSHandlerWrapper::addSubscribers() {}

void ROSHandlerWrapper::setTwistMsg(const double& linear, const double& angular)
{
    twistMsg_.linear.x = linear;
    twistMsg_.angular.z = angular;
}

void ROSHandlerWrapper::setMasterIP(const QString& masterIP){
    roshandler_.setMasterIP(masterIP.toStdString());
}

QString ROSHandlerWrapper::getMasterIP() const{
    QString tempString = QString::fromStdString(roshandler_.getMasterIP());
    return tempString;
}
