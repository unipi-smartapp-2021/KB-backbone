#include <ros/init.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <cinttypes>
#include <tuple>

#include "config.hpp"

namespace sa {

namespace kb {

template<typename I, typename T, typename U>
class TopicFrequencyMultiplexer {
 public:
  TopicFrequencyMultiplexer(std::string const& topic)
      : TopicFrequencyMultiplexer(ros::NodeHandle(), topic) {}

  TopicFrequencyMultiplexer(ros::NodeHandle const& handle, std::string const& topic)
      : name_{topic + "Multiplexer"},
        handle_{handle},
        service_{handle_.advertiseService(name_, &TopicFrequencyMultiplexer::RegisterToNode, this)},
        input_{handle_.subscribe(topic, 1, &TopicFrequencyMultiplexer::ForwardMessage, this)} {}

  auto Run() -> void {
    ros::spin();
  }

 private:
  auto RegisterToNode(T& request, U& response) -> bool {
    auto publish_interval = ros::Duration(1.0 / request.rate);
    auto new_topic = name_ + "_" + std::to_string(subscribed_);
    outputs_.emplace_back(handle_.advertise<I>(new_topic, 1), ros::Time::now(), publish_interval);
    ++subscribed_;

    ROS_INFO("Multiplexer) Created \"%s\" for rate \"%u\"", new_topic.c_str(), request.rate);
    response.topic_name = new_topic;
    return true;
  }

  auto ForwardMessage(boost::shared_ptr<I const> const& message) -> void {
    ROS_INFO("Multiplexer) Received \"%s\"", message->data.c_str());
    for (auto& tuple : outputs_) {
      auto& publish_time_guard = std::get<1>(tuple);
      auto  now = ros::Time::now();
      if (now >= publish_time_guard) {
        auto& publisher = std::get<0>(tuple);
        publisher.publish(*message);
        publish_time_guard = now + std::get<2>(tuple);
      }
    }
  }

  std::string const                                                 name_;
  ros::NodeHandle                                                   handle_;
  ros::ServiceServer                                                service_;
  ros::Subscriber                                                   input_;
  std::vector<std::tuple<ros::Publisher, ros::Time, ros::Duration>> outputs_{};
  std::uint16_t                                                     subscribed_{};
};

} // namespace kb

} // namespace sa