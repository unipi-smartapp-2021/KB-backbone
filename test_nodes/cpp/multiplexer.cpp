#include <std_msgs/String.h>

#include "config.hpp"
#include "rated_topic.hpp"

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "Multiplexer");

  auto rates = std::vector<unsigned>({5,10});
  auto multiplexer = sa::kb::RatedTopic<std_msgs::String>(sa::kb::kTestTopic, rates);
  multiplexer.Run();

  return EXIT_SUCCESS;
}