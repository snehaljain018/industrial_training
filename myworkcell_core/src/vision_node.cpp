/** Simple ROS Node **/

#include<ros/ros.h>
#include<fake_ar_publisher/ARMarker.h>
#include<myworkcell_core/LocalizePart.h>
 
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;

	class Localizer
	{
	public:
		Localizer(ros::NodeHandle& nh)
		{
			ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, &Localizer::visionCallback, this);
      server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
    }

		void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
		{
			last_msg_ = msg;
      //ROS_INFO_STREAM(last_msg_->pose.pose);
		}

    bool localizePart(myworkcell_core::LocalizePart::Request& req,
                      myworkcell_core::LocalizePart::Response& res)
    {
      fake_ar_publisher::ARMarkerConstPtr p = last_msg_;
      if (!p) return false;

      res.pose = p->pose.pose;
      return true;
    }

		ros::Subscriber ar_sub_;
		fake_ar_publisher::ARMarkerConstPtr last_msg_;
    ros::ServiceServer server_;
	};


	Localizer localizer(nh);

	ROS_INFO_STREAM("Starting vision node!");

	ros::spin();
}



