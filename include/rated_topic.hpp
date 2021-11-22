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

/** \addtogroup Smart_Application @{ */
namespace sa {

/** \addtogroup Knowledge_Base @{ */
namespace kb {

namespace {

constexpr auto kMessagesKept = 1UL; /// Number of incoming/outgoing messages to keep

} // namespace

/// TODO
/// Wraps a topic, allowing its frequency multiplexing.
///
/// # Description
/// This class can be *very* useful for limiting the amount of bandwidth used for passing messages
/// on a particular topic, in case many different subscribers are interested in it but with
/// different refresh rates.
template<typename T>
class RatedTopic {
 public:
  using DebugMessage = std_msgs::Duration; //!< How much delay a message had when we send it

  /// TODO
  ///
  /// # Arguments
  ///
  /// # Failures
  ///
  RatedTopic(std::string const&           topic,
             std::vector<unsigned> const& rates,
             ros::NodeHandle const&       handle = ros::NodeHandle())
      : name_{topic + "Rated"},
        rates_{rates},
        handle_{handle},
        subscribe_{handle_.advertiseService(name_, &RatedTopic::HandleSubscribe, this)},
        input_{handle_.subscribe(topic, kMessagesKept, &RatedTopic::ForwardMessage, this)},
        debugger_{handle_.advertise<DebugMessage>(name_ + "Delay", kMessagesKept)} {
    if (rates.empty()) {
      sa::kb::Fail("No rate provided");
    }

    if (!std::is_sorted(rates.begin(), rates.end())) {
      sa::kb::Fail("Wrong rates provided");
    }

    publishers_.reserve(rates.size());
    for (auto rate : rates) {
      publishers_.emplace_back(
          handle_.advertise<T>(name_ + std::to_string(rate) + "Hz", kMessagesKept), rate);
    }
  }

  /// TODO
  ///
  inline auto Run() -> void {
    ros::spin();
  }

 private:
  using Request = backbone::RateTopic::Request;
  using Response = backbone::RateTopic::Response;

  /// TODO
  ///
  /// # Arguments
  ///
  auto HandleSubscribe(Request& in, Response& out) -> bool {
    auto upper_bound = std::upper_bound(
        rates_.begin(), rates_.end(), in.rate, [](auto const& a, auto const& b) { return a <= b; });
    if (upper_bound == rates_.end()) {
      return false;
    }

    out.topic = publishers_[std::distance(rates_.begin(), upper_bound)].TopicName();
    return true;
  }

  /// TODO
  ///
  /// # Arguments
  ///
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

  std::string const                      name_;         /// TODO
  std::vector<unsigned> const            rates_;        /// TODO
  ros::NodeHandle                        handle_;       /// TODO
  ros::ServiceServer                     subscribe_;    /// TODO
  ros::Subscriber                        input_;        /// TODO
  std::vector<sa::kb::RatedPublisher<T>> publishers_{}; /// TODO
  ros::Publisher                         debugger_;     /// TODO
};

} // namespace kb
/** @} Knowledge_Base */

} // namespace sa
/** @} Smart_Application */

#endif // SA_KB_INCLUDE_RATED_TOPIC_H_
