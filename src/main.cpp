#include <ros/ros.h>

#include "radar_odom.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_rot_node");

    ros::NodeHandle nh("~");

    radarOdom cf(nh);

    image_transport::ImageTransport it(nh);
  	
    string param_radarSubTopic_;
    nh.getParam("sub_topic", param_radarSubTopic_);
	image_transport::Subscriber sub = it.subscribe(param_radarSubTopic_, 1, &radarOdom::callback, &cf);
    //image_transport::Subscriber sub = it.subscribe("/radar_image_inrange", 1, &radarOdom::callback, &cf);

    ros::spin();

    return 0;
}
