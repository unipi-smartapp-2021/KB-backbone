#include <ros/ros.h>
#include <std_msgs/String.h>

#include "config.hpp"
#include <chrono>
#include <ratio>
#include <sstream>
#include <thread>
#include <topic_tools/shape_shifter.h>

auto main(int argc, char *argv[]) -> int {
  ros::init(argc, argv, "Talker");

  auto handle = ros::NodeHandle();
  auto publisher =
      handle.advertise<std_msgs::String>(sa::kTestTopic, sa::kChatterQueueSize);
  if (publisher == nullptr) {
    std::cerr << "Failed advertisement on \"" << sa::kTestTopic << "\"\n";
    std::_Exit(EXIT_FAILURE);
  }

  while (publisher.getNumSubscribers() == 0) {
    // Wait for connections
    std::this_thread::sleep_for(
        std::chrono::milliseconds(sa::kSubscriptionWaitTime));
  }

  auto frequency = ros::Rate(sa::kLoopFrequency);
  auto messages_sent = 0UL;
  while (ros::ok()) {
    auto body_buffer = std::stringstream();
    body_buffer << "Hello, I'm message number " << messages_sent;

    auto message = std_msgs::String();
    message.data = body_buffer.str();
    publisher.publish(message);
    ++messages_sent;

    ros::spinOnce();
    frequency.sleep();
  }

  return EXIT_SUCCESS;
}
