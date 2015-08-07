#ifndef ROSHANDLERWRAPPER_H
#define ROSHANDLERWRAPPER_H

#include <roshandler.h>

#include <QObject>

#include <geometry_msgs/Twist.h>

class ROSHandlerWrapper : public QObject
{
    Q_OBJECT
    geometry_msgs::Twist twistMsg_;
    ROSHandler roshandler_;
    QTimer pubTimer_;

    void addPublishers();
    void addSubscribers();

public:
    ROSHandlerWrapper();
    explicit ROSHandlerWrapper(QObject* parent);

    Q_INVOKABLE void setTwistMsg(double linear, double angular);

signals:
    void log(QString msg);
};

#endif // ROSHANDLERWRAPPER_H
