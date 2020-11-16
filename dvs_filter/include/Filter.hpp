#ifndef __DVS_FILTER_H__
#define __DVS_FILTER_H__

#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

namespace dvs_filter
{
    class Filter
    {
    public:
        struct Parameter{
            float max_interval;
            int search_radius;
            int min_flicker_hz;
            float stack_time_resolution;
        };

        Filter(ros::NodeHandle &nh, ros::NodeHandle nh_private);
        virtual ~Filter();
        void publishCameraInfo();

    private:
        std::string _ns;

        ros::NodeHandle _nh;
        ros::Subscriber _camera_info_sub;
        ros::Subscriber _event_sub;
        ros::Publisher _camera_info_pub;
        ros::Publisher _event_pub;

        dvs_msgs::EventArrayPtr _events_msg;
        sensor_msgs::CameraInfoPtr _camera_info_msg;

        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);
        void eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
        bool grabEvent(const dvs_msgs::Event &ev);
        bool lookupAdjacency(const dvs_msgs::Event &ev);
        bool flickerCounter(const dvs_msgs::Event &ev);

        bool _is_camera_info_got, _is_ts_init;
        double _ts_init;
        int _width, _height;
        
        float *_sae_p, *_sae_n;
        
        int _stack_depth;
        float *_stack;
        bool * _stack_polarity;
        int8_t *_counter;

        Parameter _param;
    };
} // namespace dvs_filter

#endif