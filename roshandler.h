#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <ros/ros.h>

#include <stdarg.h>
#include <stdio.h>
#include <map>
#include <errno.h>
#include <vector>
#include <set>
#include <fstream>
#include <android/log.h>
#include <QObject>
#include <qtimer.h>
#include <QString>

class ROShandler : public QObject
{
    Q_OBJECT
private:
    double linear_, angular_;
    ros::Publisher vel_pub_;
    QString masterIP_;
    QString logMsg_;
    QTimer pubTimer_;
    QTimer rosTimer_;
    void timerInit();
    void rosInit();
    bool checkTopics();
public:
    explicit ROShandler(QObject *parent);
    ROShandler();
    ~ROShandler(){}
    Q_INVOKABLE void restartROS();
    Q_INVOKABLE void setVelAng(double linear, double angular);
    Q_INVOKABLE void shutdownROS();
    Q_INVOKABLE void setMasterIP(QString masterIP);
    Q_INVOKABLE QString getMasterIP();
signals:
    void log(QString msg);
};

#endif // ROSHANDLER_H
