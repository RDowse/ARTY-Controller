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
    auto onTimeout = [this]
    {
        roshandler_.publish("/arty/joystick", twistMsg_);
    };
    QObject::connect(&pubTimer_, &QTimer::timeout, onTimeout);
}

void ROSHandlerWrapper::addSubscribers() {}

void ROSHandlerWrapper::setTwistMsg(double linear, double angular)
{
    twistMsg_.linear.x = linear;
    twistMsg_.angular.z = angular;
}