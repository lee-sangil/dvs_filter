#include "Filter.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dvs_filter");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	dvs_filter::Filter filter(nh, nh_private);
	ros::Rate loop_rate(200);

	while(ros::ok())
	{
		ros::spinOnce();
		filter.publishCameraInfo();
		loop_rate.sleep();
	}
	ros::shutdown();

	return 0;
}