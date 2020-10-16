#include "Filter.hpp"
#include <algorithm>

namespace dvs_filter
{
    Filter::Filter(ros::NodeHandle &nh, ros::NodeHandle nh_private) : _nh(nh)
    {
        _ns = ros::this_node::getNamespace();
        if (_ns == "/")
            _ns = "/filter";

        _is_camera_info_got = false;
        _is_ts_init = false;

        // Setup subscribers and publishers
        _camera_info_sub = _nh.subscribe("camera_info", 1, &Filter::cameraInfoCallback, this);
        _event_sub = _nh.subscribe("events", 10, &Filter::eventsCallback, this);
        _event_pub = _nh.advertise<dvs_msgs::EventArray>("event_filter", 10);
        _events_msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
    }

    Filter::~Filter()
    {
        delete[] _sae_p;
        delete[] _sae_n;
        delete[] _fully_stacked;
        delete[] _stack;
        delete[] _counter;
    }

    /*** Callback functions ***/
    void Filter::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        if (_is_camera_info_got == false)
        {
            _width = msg->width;
            _height = msg->height;

            _sae_p = new double[_width * _height];
            _sae_n = new double[_width * _height];

            _counter = new uint8_t[_width * _height];
            _stack_depth = std::ceil(_param.min_flicker_hz*.1);
            _fully_stacked = new bool[_width * _height];
            _stack = new double[_stack_depth * _width * _height];

            _events_msg->height = _height;
            _events_msg->width = _width;

            _is_camera_info_got = true;
        }
    }

    void Filter::eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
    {
        if (_is_ts_init == false)
        {
            _ts_init = msg->events[0].ts.toSec();
            _is_ts_init = true;
        }

        if (_is_camera_info_got)
        {
            bool isAdjacency, isFlicker;

            _events_msg->events.clear();
            for (uint32_t i = 0; i < msg->events.size(); ++i)
            {
                grabEvent(msg->events[i]);
                lookupNeighber(msg->events[i], isAdjacency);
                filkerCounter(msg->events[i], isFlicker);

                if (isAdjacency & not isFlicker)
                    _events_msg->events.emplace_back(msg->events[i]);
            }
            _event_pub.publish(_events_msg);
        }
    }

    void Filter::grabEvent(const dvs_msgs::Event &ev)
    {
        double ts = ev.ts.toSec();
        int idx_ev = ev.y + _height * ev.x;

        if (ev.polarity > 0)
            _sae_p[idx_ev] = ts;
        else
            _sae_n[idx_ev] = ts;
        
        _stack[(_counter[idx_ev]++) + _stack_depth*idx_ev] = ts;
    }

    void Filter::lookupNeighber(const dvs_msgs::Event &ev, bool &isAdjacency)
    {
        isAdjacency = true;
    }

    void Filter::filkerCounter(const dvs_msgs::Event &ev, bool &isFlicker)
    {
        isFlicker = false;
    }

} // namespace dvs_filter