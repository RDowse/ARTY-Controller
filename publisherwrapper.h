#ifndef PUBLISHERWRAPPER_H
#define PUBLISHERWRAPPER_H

#include <ros/ros.h>
#include <mutex>

template <typename MsgType>
class PublisherWrapper
{
private:
    std::unique_ptr<ros::Publisher> publisher_;
    std::mutex publisherMutex_;
    std::string topic_;

public:
    PublisherWrapper(const std::string& topic):topic_(topic){}

    void connect(ros::NodeHandle& n){
        std::lock_guard<std::mutex> lock(publisherMutex_);
        (void*)&lock;

        publisher_.reset(new ros::Publisher(n.advertise<MsgType>(topic_, 1)));
    }
    void sendMsg(MsgType msg){
        std::lock_guard<std::mutex> lock(publisherMutex_);
        (void*)&lock;

        publisher_->publish(msg);
    }
    void disconnect(){
        std::lock_guard<std::mutex> lock(publisherMutex_);
        (void*)&lock;

        publisher_.reset();
    }
};

#endif // PUBLISHERWRAPPER_H

