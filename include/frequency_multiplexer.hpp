#include <ros/ros.h>

#include <cinttypes>

#include "backbone/ThrottleTopic.h"
#include "config.hpp"
#include "rated_publisher.hpp"

namespace sa {

namespace kb {

template<typename T>
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
  using Request = backbone::ThrottleTopic::Request;
  using Response = backbone::ThrottleTopic::Response;

  auto RegisterToNode(Request& request, Response& response) -> bool {
    auto new_topic = name_ + std::to_string(subscribed_);
    publishers_.emplace_back(handle_.advertise<T>(new_topic, 1), request.rate);
    ++subscribed_;

    ROS_INFO("Multiplexer) Created \"%s\" for \"%uHz (%ums)\"",
             new_topic.c_str(),
             request.rate,
             1'000 / request.rate);
    response.topic_name = new_topic;
    return true;
  }

  auto ForwardMessage(boost::shared_ptr<T const> const& message) -> void {
    ROS_INFO("Multiplexer) Received \"%s\"", message->data.c_str());
    static auto val = 0UL;
    for (auto& publisher : publishers_) {
      if (publisher.Publish(message)) {
        ROS_INFO("Multiplexer%s) Average Delay: %" PRId64 "ms",
                 publisher.TopicName().c_str(),
                 publisher.MovingAverageDelay().toNSec() / 1'000'000);
      }
    }
  }

  std::string const                      name_;
  ros::NodeHandle                        handle_;
  ros::ServiceServer                     service_;
  ros::Subscriber                        input_;
  std::vector<sa::kb::RatedPublisher<T>> publishers_{};
  std::uint16_t                          subscribed_{};
};

} // namespace kb

} // namespace sa