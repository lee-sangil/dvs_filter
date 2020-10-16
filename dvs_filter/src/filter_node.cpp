#include "Filter.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dvs_filter");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	dvs_filter::Filter Filter(nh, nh_private);

	ros::spin();
	ros::shutdown();

	return 0;
}