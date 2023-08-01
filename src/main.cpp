#include <ros/ros.h>

#include "radar_odom.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_rot_node");

    ros::NodeHandle nh("~");

    radarOdom cf(nh);

    image_transport::ImageTransport it(nh);
  	
	image_transport::Subscriber sub = it.subscribe("/radar/polar", 1, &radarOdom::callback, &cf);
    //image_transport::Subscriber sub = it.subscribe("/radar_image_inrange", 1, &radarOdom::callback, &cf);

    ros::spin();

    return 0;
}
