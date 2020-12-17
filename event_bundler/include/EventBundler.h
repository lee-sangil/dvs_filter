#ifndef EVENTBUNDLER_H
#define EVENTBUNDLER_H
// ros
#include <ros/ros.h>

// opencv
#include <opencv2/core.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

class EventBundler
{
public:
    struct Parameter{
        bool check_time_interval = true;
        bool check_number_events = true;
    };
    EventBundler(std::string yaml);

    void GrabEvent(dvs_msgs::EventArrayConstPtr msg);
    void setEventPublisher(ros::Publisher &pub);

private:
    std::vector<dvs_msgs::Event> vec_msg;
    dvs_msgs::EventArrayPtr bundle_msg;

    int max_events;
    double delta_time;

    double next_send_time = -1;
    double time_curr = -1;

    ros::Publisher event_array_pub;
    EventBundler::Parameter _param;
};

#endif // EVENTBUNDLER_H