#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <subscriberwrapper.h>
#include <publisherwrapper.h>
#include <QDebug>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QTimer>
#include <Any.h>
#include <sensor_msgs/Joy.h>
#include <unordered_map>
#include <thread>
#include <ros/ros.h>

class ROSHandler
{
private:
    std::thread thread_;
    std::vector<types::Any> subscribers_;
    std::unordered_map<std::string, types::Any> publishers_;
    QTimer pubTimer_;
    QTimer rosSpinTimer_;
    QTimer rosMasterTimer_;
    void initROS();
    void initTimers();
    void run();

public:
    ROSHandler();
    void start();
    void stop();
    bool isConnected();
    template <typename MsgType>
    void subscribe(std::string& topic, std::function<void()> callback)
    {
        subscribers_.emplace(subscribers_.end(),new SubscriberWrapper<MsgType>(topic));
        // subscribers_.front().connect()
    }
    template <typename MsgType>
    bool publish(const std::string& topic, const MsgType& msg, std::function<void()> getMsg)
    {
        if (!isConnected())
        {
            return false;
        }
        if (publishers_.find(topic) == std::end(publishers_))
        {
            publishers_.emplace(topic, PublisherWrapper<MsgType>(topic));
        }
        publishers_.at(topic).as<PublisherWrapper<MsgType> >().publish(msg);
        QObject::connect(&pubTimer_, &QTimer::timeout, getMsg);
        return true;
    }
    void deletePublishers();
};

#endif // ROSHANDLER_H
