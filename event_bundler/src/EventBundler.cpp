#include "EventBundler.h"
EventBundler::EventBundler(std::string yaml)
{
    // read yaml file
    cv::FileStorage fSettings(yaml, cv::FileStorage::READ);
    if (!fSettings.isOpened()){
        throw std::runtime_error(std::string("Could not open file: ") + yaml);
    }

    delta_time = fSettings["delta_time"];
    max_events = fSettings["max_events"];
    if (delta_time == 0)
        _param.check_time_interval = false;
    if (max_events == 0)
        _param.check_number_events = false;

    bundle_msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
    vec_msg.reserve(max_events);
}

void EventBundler::GrabEvent(dvs_msgs::EventArrayConstPtr msg)
{
    if (time_curr < 0){
        next_send_time = msg->events[0].ts.toSec() + delta_time;
        bundle_msg->height = msg->height;
        bundle_msg->width = msg->width;
    }

    for (int i = 0, len = msg->events.size(); i < len; ++i){
        time_curr = msg->events[i].ts.toSec();
        vec_msg.emplace_back(msg->events[i]);

        if ( (time_curr > next_send_time & _param.check_time_interval) || (vec_msg.size() == max_events & _param.check_number_events))
        {
            // publish event
            bundle_msg->header.stamp = vec_msg[0].ts;
            bundle_msg->events.assign(vec_msg.begin(), vec_msg.end());
            event_array_pub.publish(bundle_msg);

            next_send_time = time_curr + delta_time;
            vec_msg.clear();
        }
    }
}

void EventBundler::setEventPublisher(ros::Publisher &pub)
{
    this->event_array_pub = pub;
}