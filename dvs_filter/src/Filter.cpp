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

        nh_private.param<double>("max_interval", _param.max_interval, 0.01);
        nh_private.param<int>("search_radius", _param.search_radius, 1);
        nh_private.param<int>("min_flicker_hz", _param.min_flicker_hz, 60);
        _param.stack_time_resolution = 0.1;

        // Setup subscribers and publishers
        _camera_info_sub = _nh.subscribe("camera_info", 1, &Filter::cameraInfoCallback, this);
        _event_sub = _nh.subscribe("events", 10, &Filter::eventsCallback, this);
        _event_pub = _nh.advertise<dvs_msgs::EventArray>(_ns + "/events", 10);
        _events_msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
    }

    Filter::~Filter()
    {
        delete[] _sae_p;
        delete[] _sae_n;
        delete[] _stack;
        delete[] _stack_polarity;
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

            _stack_depth = std::ceil(2.*_param.min_flicker_hz*_param.stack_time_resolution);
            _stack = new double[_stack_depth * _width * _height];
            _stack_polarity = new bool[_width * _height];
            _counter = new int8_t[_width * _height];

            for (int i = 0; i < _width*_height; i++)
                _counter[i] = -1;

            _events_msg->height = _height;
            _events_msg->width = _width;

            _is_camera_info_got = true;
        }
    }

    void Filter::eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
    {
        static uint32_t s_count;

        if (_is_ts_init == false)
        {
            _ts_init = msg->events[0].ts.toSec();
            s_count = msg->header.seq;
            _is_ts_init = true;
        }

        if (_is_camera_info_got)
        {
            if (msg->header.seq > ++s_count)
            {
                std::cout << "Missing events [seq=" << s_count << "..." << msg->header.seq-1 << ']' << std::endl;
                s_count = msg->header.seq;
            }

            bool isOverlap, isAdjacency, isFlicker;

            _events_msg->events.clear();
            for (uint32_t i = 0; i < msg->events.size(); ++i)
            {
                grabEvent(msg->events[i], isOverlap);
                lookupAdjacency(msg->events[i], isAdjacency);
                flickerCounter(msg->events[i], isFlicker);

                if (not isOverlap & isAdjacency & not isFlicker)
                    _events_msg->events.emplace_back(msg->events[i]);
            }
            _event_pub.publish(_events_msg);
        }
    }

    void Filter::grabEvent(const dvs_msgs::Event &ev, bool &isOverlap)
    {
        isOverlap = true;
        const double ts = ev.ts.toSec();
        const int idx_ev = ev.y + _height * ev.x;

        if (ev.polarity > 0)
        {
            if (_sae_p[idx_ev] <= _sae_n[idx_ev])
            {
                _sae_p[idx_ev] = ts;
                isOverlap = false;
            }
        }
        else
        {
            if (_sae_p[idx_ev] >= _sae_n[idx_ev])
            {
                _sae_n[idx_ev] = ts;
                isOverlap = false;
            }
        }
        
        if (_counter[idx_ev] >= 0 && _stack_polarity[idx_ev] != ev.polarity || _counter[idx_ev] < 0)
        {
            if (_counter[idx_ev] < 0)
                _counter[idx_ev] = 0;

            _stack_polarity[idx_ev] = ev.polarity;
            _stack[_counter[idx_ev] + _stack_depth*idx_ev] = ts;
            _counter[idx_ev]++;

            if (_counter[idx_ev] == _stack_depth)
                _counter[idx_ev] = 0;
        }
    }

    void Filter::lookupAdjacency(const dvs_msgs::Event &ev, bool &isAdjacency)
    {
        const double * sae;
        if (ev.polarity > 0)
            sae = _sae_p;
        else
            sae = _sae_n;
            
        const double ts = ev.ts.toSec();
        const double th_ts = std::max(ts - _param.max_interval, 0.);

        const int min_col = std::max(ev.x-_param.search_radius,0);
        const int max_col = std::min(ev.x+_param.search_radius,_width);
        const int min_row = std::max(ev.y-_param.search_radius,0);
        const int max_row = std::min(ev.y+_param.search_radius,_height);

        isAdjacency = false;
        for (int c = min_col; c < max_col; c++)
        {
            for (int r = min_row; r < max_row; r++)
            {
                if (sae[r + _height * c] > th_ts && sae[r + _height * c] < ts)
                {
                    isAdjacency = true;
                    break;
                }
            }
        }
    }

    void Filter::flickerCounter(const dvs_msgs::Event &ev, bool &isFlicker)
    {
        const double ts = ev.ts.toSec();
        const double th_ts = std::max(ts - _param.stack_time_resolution, 0.);
        
        const int idx_ev = ev.y + _height * ev.x;

        isFlicker = false;

        if (_stack[_counter[idx_ev] + _stack_depth*idx_ev] > th_ts)
            isFlicker = true;
    }

} // namespace dvs_filter  