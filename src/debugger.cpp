#include <ros/init.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <std_msgs/Duration.h>

#include <cstdlib>

#include "config.hpp"

auto Callback(boost::shared_ptr<std_msgs::Duration const> const& message) -> void {
  ROS_INFO("Debugger) Delay is \"%ld\" microseconds", message->data.toNSec() / 1'000);
}

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "Debugger");

  auto handle = ros::NodeHandle();
  auto client = handle.subscribe(std::string(sa::kb::kTestTopic) + "RatedDelay", 1, Callback);
  ros::spin();

  return EXIT_SUCCESS;
}