#ifndef __DVS_FILTER_H__
#define __DVS_FILTER_H__

#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

namespace dvs_filter
{
    class Filter
    {
    public:
        struct Parameter{
            double max_interval;
            int search_radius;
            int min_flicker_hz;
        };

        Filter(ros::NodeHandle &nh, ros::NodeHandle nh_private);
        virtual ~Filter();

    private:
        std::string _ns;

        ros::NodeHandle _nh;
        ros::Subscriber _camera_info_sub;
        ros::Subscriber _event_sub;
        ros::Publisher _event_pub;

        dvs_msgs::EventArrayPtr _events_msg;

        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);
        void eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
        void grabEvent(const dvs_msgs::Event &ev);
        void lookupNeighber(const dvs_msgs::Event &ev, bool &isAdjacency);
        void filkerCounter(const dvs_msgs::Event &ev, bool &isFlicker);

        bool _is_camera_info_got, _is_ts_init;
        double _ts_init;
        int _width, _height;
        
        double *_sae_p, *_sae_n;

        uint8_t *_counter;
        int _stack_depth;
        bool *_fully_stacked;
        double *_stack;

        Parameter _param;
    };
} // namespace dvs_filter

#endif