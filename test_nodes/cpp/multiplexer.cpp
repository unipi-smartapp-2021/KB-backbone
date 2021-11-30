#include <std_msgs/String.h>

#include "config.hpp"
#include "rated_topic.hpp"
#include "ros/init.h"

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "Multiplexer");

  auto rates = std::vector<unsigned>({1,5});
  auto multiplexer = sa::kb::RatedTopic<std_msgs::String>(sa::kb::kTestTopic, rates);
  ros::spin();

  return EXIT_SUCCESS;
}