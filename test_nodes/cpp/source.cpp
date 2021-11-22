#include <ros/ros.h>
#include <std_msgs/String.h>

#include <thread>

#include "config.hpp"

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "Source");

  auto handle = ros::NodeHandle();
  auto publisher = handle.advertise<std_msgs::String>(sa::kb::kTestTopic, 1);
  if (publisher == nullptr) {
    std::cerr << "Failed advertisement on \"" << sa::kb::kTestTopic << "\"\n";
    std::_Exit(EXIT_FAILURE);
  }

  while (publisher.getNumSubscribers() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(sa::kb::kSubscriptionWaitTimeMs));
  }

  auto frequency = ros::Rate(sa::kb::kSourceRate);
  auto messages_sent = 0UL;
  while (ros::ok()) {
    auto body_buffer = std::stringstream();
    body_buffer << "Message " << messages_sent;

    auto message = std_msgs::String();
    message.data = body_buffer.str();
    publisher.publish(message);
    ++messages_sent;

    frequency.sleep();
  }

  return EXIT_SUCCESS;
}