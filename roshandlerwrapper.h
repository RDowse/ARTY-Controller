#ifndef ROSHANDLERWRAPPER_H
#define ROSHANDLERWRAPPER_H

#include <roshandler.h>

#include <QObject>

#include <geometry_msgs/Twist.h>

class ROSHandlerWrapper : public QObject
{
    Q_OBJECT
    ROSHandler roshandler_;
    QTimer pubTimer_;

    //MESSAGES
    geometry_msgs::Twist twistMsg_;

    void addPublishers();
    void addSubscribers();

public:
    ROSHandlerWrapper();
    explicit ROSHandlerWrapper(QObject* parent);

    Q_INVOKABLE void setTwistMsg(const double& linear, const double& angular);
    Q_INVOKABLE void setMasterIP(const QString& masterIP);
    Q_INVOKABLE QString getMasterIP() const;

signals:
    void log(QString msg);
};

#endif // ROSHANDLERWRAPPER_H
