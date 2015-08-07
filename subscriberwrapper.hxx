#ifndef SUBSCRIBERWRAPPER_H
#define SUBSCRIBERWRAPPER_H

#include <ros/ros.h>
#include <mutex>

template <typename MsgType>
class SubscriberWrapper
{
private:
    std::unique_ptr<ros::Subscriber> subscriber_;
    std::mutex subscriberMutex_;
    std::string topic_;
public:
    SubscriberWrapper(const std::string& topic): topic_(topic){}
    void connect(const ros::NodeHandle& n,
        std::function<void(const MsgType&)> callback){
        std::lock_guard<std::mutex> lock(subscriberMutex_);
        (void*)lock;

        auto safeCallback = [this, callback](const MsgType& msg)
        {
            std::lock_guard<std::mutex> lock(subscriberMutex_);
            (void*)lock;

            callback(msg);
        };

        subscriber_.reset(
            new ros::Subscriber(n.subscribe(topic_, 1, &safeCallback, this)));
    }
    void disconnect(){
        std::lock_guard<std::mutex> lock(subscriberMutex_);
        //(void*)lock;

        subscriber_.reset();
    }
};

#endif // SUBSCRIBERWRAPPER_H
