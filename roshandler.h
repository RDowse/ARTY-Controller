#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <subscriberwrapper.hxx>
#include <publisherwrapper.hxx>
#include <Any.h>

#include <QDebug>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QTimer>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

#include <unordered_map>
#include <thread>

class ROSHandler
{
private:
    std::thread thread_;
    std::vector<types::Any> subscribers_;
    std::unordered_map<std::string, types::Any> publishers_;
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
        if (!isConnected())
        {
            return;
        }
        subscribers_.emplace(
            subscribers_.end(), new SubscriberWrapper<MsgType>(topic));
        // subscribers_.front().connect()
    }
    template <typename MsgType>
    bool publish(const std::string& topic, const MsgType& msg)
    {

        if (!isConnected())
        {
            return false;
        }
        if (publishers_.find(topic) == std::end(publishers_))
        {
            ros::NodeHandle n;
            publishers_.emplace(topic, PublisherWrapper<MsgType>(topic));
            publishers_.at(topic).as<PublisherWrapper<MsgType> >().connect(n);
        }
        publishers_.at(topic).as<PublisherWrapper<MsgType> >().sendMsg(msg);
        return true;
    }
    void deletePublishers();
};

#endif // ROSHANDLER_H
