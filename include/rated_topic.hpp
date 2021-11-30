/// \file rated_topic.hpp
///
/// File containing both definition and implementation of the [`RatedTopic`] class.
///
/// [`RatedTopic`]: ::RatedTopic
///
/// \author backbone_team
#ifndef SA_KB_INCLUDE_RATED_TOPIC_H_
#define SA_KB_INCLUDE_RATED_TOPIC_H_

#include <ros/duration.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/Duration.h>

#include <algorithm>

#include "backbone/RateTopic.h"
#include "config.hpp"
#include "rated_publisher.hpp"

/** \addtogroup Smart_Application 
    @{ 
*/
namespace sa {

/** \addtogroup Knowledge_Base 
    @{ 
*/
namespace kb {

namespace {

constexpr auto kMessagesKept = 1UL; /// Number of incoming/outgoing messages to keep

} // namespace

/// A frequency multiplexer of a [topic ros] that allows the forwarding of messages received on it to
/// an arbitrary number of subscribers, with different update rates.
///
/// [ros topic]: http://wiki.ros.org/Topics
///
/// # Description
/// This class is particularly useful when there is a need to **limit** the communication overhead in
/// case many nodes are interested in the same topic, but with different update rates. At the time of
/// creation it is possible to specify a set of update frequencies that will be used to create as many
/// new topics on which the subscribers will be distributed according to their preference.
///
/// To be able to subscribe to the multiplexed topic it is sufficient to invoke the service identified as
/// `"TopicName" + "Rated"` providing an appropriate `RateTopic` request message. The response from the
/// receiving node will contain the name that identifies **the first topic that has an update frequency
/// greater or equal than the one specified** if such exists, otherwise the call fails, `false` is
/// returned and the associated response **must not be used**.
///
/// If the frequencies are known in advance, it is possible to directly connect to the associated topics
/// using as name `"TopicName" + "Rated" + rate + "Hz"`.
///
/// If a node that is used for debugging/logging purposes is interested in knowing the publish delay of
/// the multiplexer, it can subscribe to the topic `"TopicName" + "Rated" + "Delay".
template<typename T>
class RatedTopic {
 public:
  using DebugMessage = std_msgs::Duration; ///< Shorthand for the message used to profile delays

  /// \brief Multiplexes the `topic` specified with the `rates` provided.
  ///
  /// \param topic Name of the topic to which subscribe
  /// \param rates The ordered set of rates to support
  /// \param handle Handle of the ros node owning the new object
  ///
  /// ### Failures
  /// If `rates` is empty, is not sorted or contains duplicates, the procedure aborts
  /// calling [`Fail()`].
  ///
  /// [`Fail()`]: sa::kb::Fail
  RatedTopic(std::string const&           topic,
             std::vector<unsigned> const& rates,
             ros::NodeHandle const&       handle = ros::NodeHandle())
      : rates_{rates},
        handle_{handle},
        input_{handle_.subscribe(topic, kMessagesKept, &RatedTopic::ForwardMessage, this)} {
    if (rates.empty()) {
      sa::kb::Fail("No rate provided");
    }

    if (!std::is_sorted(rates.begin(), rates.end())) {
      sa::kb::Fail("Wrong rates provided");
    }
    auto const name = topic + "Rated";
    subscribe_ = handle_.advertiseService(name, &RatedTopic::HandleSubscribe, this);
    debugger_ = handle_.advertise<DebugMessage>(name + "Delay", kMessagesKept);
    publishers_.reserve(rates.size());
    for (auto rate : rates) {
      publishers_.emplace_back(
          handle_.advertise<T>(name + std::to_string(rate) + "Hz", kMessagesKept), rate);
    }
  }

  /// \brief Executes the [`ros::spin()`] loop.
  ///
  /// [`ros::spin()`]: ros::spin
  inline auto Run() -> void {
    ros::spin();
  }

 private:
  using Request = backbone::RateTopic::Request;   /// Shorthand for the request message type
  using Response = backbone::RateTopic::Response; /// Shorthand for the response message type

  /// \brief Handles subscription requests, returning the name of the first topic
  /// that has an update rate that is **greater or equal than** that provided.
  ///
  /// If there is no such topic, the function returns `false` and the responde message
  /// **must not be used**.
  ///
  /// \param in Incoming request message
  /// \param out Outgoing response message
  ///
  /// \return True if successfully subscribed
  /// \return False if there is no such topic to subscribe
  auto HandleSubscribe(Request& in, Response& out) -> bool {
    if (in.rate <= 0){
      ROS_ERROR("Got request rate %d Hz, please send a positive value", in.rate);
      return true;
    }
    else {
      auto upper_bound = std::upper_bound(
          rates_.begin(), rates_.end(), in.rate);
      if (upper_bound == rates_.begin()) {
        ROS_ERROR("Got request rate %d Hz: the minimum frequency supported is %d Hz",in.rate,*rates_.begin());
        return false;
      }
    upper_bound--;
    out.topic = publishers_[std::distance(rates_.begin(), upper_bound)].TopicName();
    return true;
  }
  }
  /// \brief Tries to forward the received message on the source topic to all the rated ones.
  ///
  /// If there is **at least** one node subscribed to the debug topic, than it also computes
  /// the actual delay for each publish performed and sends the average on it.
  ///
  /// \param to_forward The incoming message to forward to the rated topics
  auto ForwardMessage(boost::shared_ptr<T const> const& to_forward) -> void {
    if (debugger_.getNumSubscribers() == 0) {
      for (auto& publisher : publishers_) {
        publisher.Publish(to_forward);
      }
      return;
    }

    auto total_delay = ros::Duration(0, 0);
    auto forwarded = 0;
    for (auto& publisher : publishers_) {
      auto delay = publisher.PublishAndDelay(to_forward);
      if (delay.toNSec() >= 0) {
        total_delay += delay;
        ++forwarded;
      }
    }

    if (forwarded > 0) {
      auto to_debug = std_msgs::Duration();
      to_debug.data = ros::Duration(0, 0).fromNSec(total_delay.toNSec() / forwarded);
      debugger_.publish(to_debug);
    }
  }
  std::vector<unsigned> const            rates_;        /// Set of ordered rates to support
  ros::NodeHandle                        handle_;       /// Handle of the owner ros node
  ros::ServiceServer                     subscribe_;    /// Subscribe service provider
  ros::Subscriber                        input_;        /// Source topic
  std::vector<sa::kb::RatedPublisher<T>> publishers_{}; /// Set of rated topics
  ros::Publisher                         debugger_;     /// Debug topic on which to send delays
};

} // namespace kb
/** @} Knowledge_Base */

} // namespace sa
/** @} Smart_Application */

#endif // SA_KB_INCLUDE_RATED_TOPIC_H_
