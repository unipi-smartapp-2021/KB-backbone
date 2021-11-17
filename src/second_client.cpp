#include <ros/init.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cstdlib>

#include "backbone/ThrottleTopic.h"
#include "config.hpp"

auto Callback(boost::shared_ptr<std_msgs::String const> const& message) -> void {
  ROS_INFO("SecondClient) Received \"%s\"", message->data.c_str());
}

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "SecondClient");

  auto handle = ros::NodeHandle();
  auto client = handle.serviceClient<backbone::ThrottleTopic>(std::string(sa::kb::kTestTopic) +
                                                              "Multiplexer");
  auto service = backbone::ThrottleTopic();
  service.request.rate = sa::kb::kSecondClientRate;
  if (!client.call(service)) {
    ROS_ERROR("SecondClient) Failed to call service");
    return EXIT_FAILURE;
  }

  ROS_INFO("SecondClient) Receiving on \"%s\"", service.response.topic_name.c_str());
  auto subscriber = handle.subscribe(service.response.topic_name, 1, Callback);
  ros::spin();

  return EXIT_SUCCESS;
}