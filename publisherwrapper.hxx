#ifndef PUBLISHERWRAPPER_H
#define PUBLISHERWRAPPER_H

#include <ros/ros.h>
#include <mutex>

template <typename MsgType> class PublisherWrapper
{
private:
    std::unique_ptr<ros::Publisher> publisher_;
    std::mutex mtx_;
    std::string topic_;

    static ros::Publisher* copyIfValid(const std::unique_ptr<ros::Publisher>& obj)
    {
        if (obj)
        {
            return new ros::Publisher(*obj);
        }
        else
        {
            return nullptr;
        }
    }

public:
    PublisherWrapper(const std::string& topic)
        : topic_(topic)
    {
    }

    void connect(ros::NodeHandle& n)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        (void*)&lock;

        publisher_.reset(new ros::Publisher(n.advertise<MsgType>(topic_, 1)));
    }
    void sendMsg(MsgType msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        (void*)&lock;

        publisher_->publish(msg);
    }
    void disconnect()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        (void*)&lock;

        publisher_.reset();
    }

    PublisherWrapper(const PublisherWrapper &obj)
        : publisher_( PublisherWrapper::copyIfValid(obj.publisher_) )
        , topic_(obj.topic_)
    {
    }

};

#endif // PUBLISHERWRAPPER_H
