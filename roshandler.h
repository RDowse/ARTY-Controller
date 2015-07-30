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
#include <QImage>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class ROShandler : public QObject
{
    Q_OBJECT
private:
    double linear_, angular_;
    ros::Publisher vel_pub_;
    ros::Publisher joystick_pub_;
    ros::Subscriber map_sub_;
    ros::Subscriber map_meta_sub_;
    QString masterIP_;
    QString logMsg_;
    QTimer pubTimer_;
    QTimer rosSpinTimer_;
    QTimer rosMasterTimer_;
    void timerInit();
    void reconnectTimerInit();
    void rosInit();
    bool checkTopics();

public:
    explicit ROShandler(QObject* parent);
    ROShandler();
    ~ROShandler() {}
    Q_INVOKABLE void restartROS();
    Q_INVOKABLE void setVelAng(double linear, double angular);
    Q_INVOKABLE void shutdownROS();
    Q_INVOKABLE void setMasterIP(QString masterIP);
    Q_INVOKABLE QString getMasterIP();
    void callback(const nav_msgs::OccupancyGrid &msg);
    void callbackPos(const geometry_msgs::PoseWithCovarianceStamped& msg);
signals:
    void log(QString msg);
    void mapUpdate(QImage map);
    void posUpdate(geometry_msgs::Pose position);
};

#endif // ROSHANDLER_H
