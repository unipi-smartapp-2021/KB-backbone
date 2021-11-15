#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/String.h>

#include <cstdlib>

#include "config.hpp"

namespace {

auto PrintMessageBody(std_msgs::String::ConstPtr const& message) -> void {
  ROS_INFO("%s", message->data.c_str());
}

} // namespace

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "Listener");

  auto node = ros::NodeHandle();
  auto subscriber = node.subscribe(sa::kTestTopic, sa::kChatterQueueSize, PrintMessageBody);
  if (subscriber == nullptr) {
    std::cerr << "Failed subscription on \"" << sa::kTestTopic << "\"\n";
    std::_Exit(EXIT_FAILURE);
  }

  ros::spin();

  return EXIT_SUCCESS;
}