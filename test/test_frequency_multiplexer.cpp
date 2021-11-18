#include <gtest/gtest.h>
#include <string>
#include "config.hpp"
#include "frequency_multiplexer.hpp"
#include "std_msgs/String.h"



TEST(FrequencyMultiplexerTests, TestCall) //NOLINT
{
  auto s = sa::kb::TopicFrequencyMultiplexer<std_msgs::String>(sa::kb::kTestTopic);
  s.Run();
  
  

  
}


auto main(int argc, char** argv) -> int {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_talker");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}