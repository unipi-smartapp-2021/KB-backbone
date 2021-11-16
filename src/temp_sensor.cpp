#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <cstdlib>

#include "config.hpp"
#include "ros/node_handle.h"

namespace {

constexpr auto kDummyTemperature = 30.0;
constexpr auto kReadFrequency = 10.0F;
constexpr auto kPublishFrequency = 5.0F;

} // namespace

class TemperatureSensor {
 public:
  TemperatureSensor(ros::NodeHandle& handle)
      : publisher_{handle.advertise<std_msgs::Float64>(sa::kTestTopic, sa::kChatterQueueSize)} {}

  auto ReadTemperatureSensorData() const -> double {
    ROS_INFO("Reading Sensor");
    return temperature_;
  }

  auto PublishTemperature() -> void {
    ROS_INFO("Publishing Temperature");
    auto message = std_msgs::Float64();
    message.data = this->ReadTemperatureSensorData();
    publisher_.publish(message);
  }

 private:
  double         temperature_{kDummyTemperature};
  ros::Publisher publisher_;
};

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "TemperatureSensorNode");

  auto handle = ros::NodeHandle();
  auto temperature_sensor = TemperatureSensor(handle);
  auto read_temperature_timer = handle.createTimer(
      ros::Duration(1.0 / kReadFrequency),
      std::bind(&TemperatureSensor::ReadTemperatureSensorData, &temperature_sensor));

  auto temperature_publish_timer =
      handle.createTimer(ros::Duration(1.0 / kPublishFrequency),
                         std::bind(&TemperatureSensor::PublishTemperature, &temperature_sensor));

  ros::spin();

  return EXIT_SUCCESS;
}
