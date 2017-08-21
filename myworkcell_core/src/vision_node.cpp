/** Simple ROS Node **/

#include<ros/ros.h>
 
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;

	ROS_INFO_STREAM("Hello World!");

	ros::spin();
}
