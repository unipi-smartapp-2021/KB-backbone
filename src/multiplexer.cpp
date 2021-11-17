#include <std_msgs/String.h>

#include "config.hpp"
#include "frequency_multiplexer.hpp"

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "Multiplexer");

  auto multiplexer = sa::kb::TopicFrequencyMultiplexer<std_msgs::String>(sa::kb::kTestTopic);
  multiplexer.Run();

  return EXIT_SUCCESS;
}