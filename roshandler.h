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
    bool running_;
    bool connected_;
    std::string masterIP_;

    std::recursive_mutex mtx_;
    std::thread thread_;
    std::vector<types::Any> subscribers_;
    std::unordered_map<std::string, types::Any> publishers_;
    std::unique_ptr<ros::NodeHandle> node_;

    QTimer rosSpinTimer_;
    QTimer rosMasterTimer_;

    void start();
    void startROS();
    void stopROS();
    void run();
    void deletePublishers();
    void initSpinTimer();

public:
    ROSHandler();
    ~ROSHandler();

    bool isConnected();
    void setMasterIP(std::string masterIP);
    std::string getMasterIP() const;
    template <typename MsgType>
    void subscribe(std::string& topic, std::function<void()> callback)
    {
        std::lock_guard<decltype(mtx_)> lock(mtx_);

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
        std::lock_guard<decltype(mtx_)> lock(mtx_);

        if (!connected_ || !node_)
        {
            return false;
        }
        if (publishers_.find(topic) == std::end(publishers_))
        {
            //Add the publisher to the map if it does not exist and advertise it.
            publishers_.emplace(topic, PublisherWrapper<MsgType>(topic));
            publishers_.at(topic).as<PublisherWrapper<MsgType> >().connect(*node_);
        }
        publishers_.at(topic).as<PublisherWrapper<MsgType> >().sendMsg(msg);
        return true;
    }
};

#endif // ROSHANDLER_H
