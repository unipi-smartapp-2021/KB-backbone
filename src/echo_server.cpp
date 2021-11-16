#include "backbone/ThrottleTopic.h"
#include "ros/ros.h"

auto handle(backbone::ThrottleTopic::Request& req, backbone::ThrottleTopic::Response& res) -> bool {
  auto throttle_rate = req.rate;
  auto new_topic = "test";
  ROS_INFO("Serving new topic %s at rate %d", new_topic, throttle_rate);
  return true;
}

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "throttle_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ThrottleTopic", handle);
  ROS_INFO("Ready to throttle topic");
  ros::spin();

  return 0;
}