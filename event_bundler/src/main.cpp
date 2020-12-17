#include <ros/ros.h>


#include "EventBundler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rvio");
    ros::NodeHandle nh("~");

    std::string yaml;
    nh.param<std::string>("yaml", yaml, "");

    EventBundler eventBundler(yaml);

    ros::Publisher event_publisher = nh.advertise<dvs_msgs::EventArray>("/bundle_events", 10);
    ros::Subscriber event_sub = nh.subscribe("/events", 10, &EventBundler::GrabEvent, &eventBundler);
    eventBundler.setEventPublisher(event_publisher);

    ros::spin();
    ros::shutdown();

    return 0;
}
