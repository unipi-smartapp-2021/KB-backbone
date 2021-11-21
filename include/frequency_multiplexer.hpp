#include <ros/ros.h>

#include <cinttypes>

#include "backbone/ThrottleTopic.h"
#include "config.hpp"
#include "rated_publisher.hpp"

namespace sa {

namespace kb {

template<typename T>
/** \brief generic wrapper class for Multiplexer
    expose a service for the arrival of a certain type of messages from a topic with a fixed frequency
 */
class TopicFrequencyMultiplexer {
 public:
  /** \brief base constructor of the class
     
      \param topic the topic of the service
   */
  TopicFrequencyMultiplexer(std::string const& topic)
      : TopicFrequencyMultiplexer(ros::NodeHandle(), topic) {}

  /** \brief advanced constructor of the class

      \param handle the handler of the Node
      \param topic the topic of the service
   */
  TopicFrequencyMultiplexer(ros::NodeHandle const& handle, std::string const& topic)
      : name_{topic + "Multiplexer"},
        handle_{handle},
        service_{handle_.advertiseService(name_, &TopicFrequencyMultiplexer::RegisterToNode, this)},
        input_{handle_.subscribe(topic, 1, &TopicFrequencyMultiplexer::ForwardMessage, this)} {}

  /** \brief The run method that start the topic
     
   */
  auto Run() -> void {
    ros::spin();
  }

 private:
  using Request = backbone::ThrottleTopic::Request; //!< Variable that identify the Request Topic
  using Response = backbone::ThrottleTopic::Response; //!< Variable that identify the Responce Topic

  /** \brief Function that register a Node to the Service

      \param request Request Handler of the Node
      \param responce Responce Handler of the Node
      
      \return True if successfully subscribed
   */
  auto RegisterToNode(Request& request, Response& response) -> bool {
    auto new_topic = name_ + std::to_string(subscribed_);
    publishers_.emplace_back(handle_.advertise<T>(new_topic, 1), request.rate);
    ++subscribed_;

    ROS_INFO("Multiplexer) Created \"%s\" for \"%uHz (%ums)\"",
             new_topic.c_str(),
             request.rate,
             1000 / request.rate);
    response.topic_name = new_topic;
    return true;
  }

  /** \brief Function that Forward the message to every Node subscribed

      \param message The message that need to be sent
   */
  auto ForwardMessage(boost::shared_ptr<T const> const& message) -> void {
    ROS_INFO("Multiplexer) Received \"%s\"", message->data.c_str());
    static auto val = 0UL;
    for (auto& publisher : publishers_) {
      if (publisher.Publish(message)) {
        ROS_INFO("Multiplexer%s) Average Delay: %" PRId64 "ms",
                 publisher.TopicName().c_str(),
                 publisher.MovingAverageDelay().toNSec() / 1000000);
      }
    }
  }

  std::string const                      name_; //!< the name of the Service
  ros::NodeHandle                        handle_; //!< the handler of the node
  ros::ServiceServer                     service_; //!< the service server we are multiplexxing
  ros::Subscriber                        input_; //!< the subscriber 
  std::vector<sa::kb::RatedPublisher<T>> publishers_{}; //!< array of who is subscribed to us
  std::uint16_t                          subscribed_{}; //!< total number of the subscribed we have
};

} // namespace kb

} // namespace sa
