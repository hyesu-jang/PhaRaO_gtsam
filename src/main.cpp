#include <ros/ros.h>

#include <PhaRaO.hpp>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "radar_rot_node");

	ros::NodeHandle nh("~");

	PhaRaO pr(nh);

	image_transport::ImageTransport it(nh);

	string param_radarSubTopic_ = "/radar/polar";
	nh.getParam("sub_topic", param_radarSubTopic_);
	image_transport::Subscriber sub = it.subscribe(param_radarSubTopic_, 1, &PhaRaO::callback, &pr);

	
	while (true)
	{
		ros::spinOnce();
	}
	

	return 0;
}
