#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_num_gen");
  ros::NodeHandle nh;

  ros::Publisher number_generator = nh.advertise<std_msgs::Int8>("random_number", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Int8 msg;
    msg.data = rand() % 100 + 1;

    number_generator.publish(msg);
    ROS_INFO("Random number published is %d", msg.data);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
