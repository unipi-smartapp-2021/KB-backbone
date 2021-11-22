/** \file rated_topic.hpp

    File that contains the class for the Topic with a frequency rate

    \author backbone_team
 */
#ifndef SA_KB_INCLUDE_RATED_TOPIC_H_
#define SA_KB_INCLUDE_RATED_TOPIC_H_

#include <ros/duration.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/Duration.h>

#include <algorithm>

#include "backbone/RateTopic.h"
#include "config.hpp"
#include "fail.hpp"
#include "rated_publisher.hpp"

/**
   \addtogroup Smart_Application
   @{
 */

//! Namespace that identify the whole Smart Application project
namespace sa {

/**
    \addtogroup Knowledge_Base
    @{
  */

//! Namespace that identify the Knowledge Base part of the project
namespace kb {

namespace {

constexpr auto kMessagesKept = 1UL; //!< Number of incoming/outcoming messages to keep

} // namespace

/** \brief Wraps a topic, allowing its frequency multiplexing.

    ## Description
    This class can be *very* useful for limiting the amount of bandwidth used for passing messages
    on a particular topic, in case many different subscribers are interested in it but with
    different refresh rates.
*/
template<typename T>
class RatedTopic {
 public:
  using DebugMessage = std_msgs::Duration; //!< How much delay a message had when we send it

  /** \brief Constructor of the class

      \param topic the topic of interest that we are subscribing to
      \param rates the rate at wich we want to be updated
      \param handle the NodeHandle we will refer to
   */
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

  /** \brief Function that start the ros service

   Handles properly both incoming requests and messages.
  */
  inline auto Run() -> void {
    ros::spin();
  }

 private:
  using Request = backbone::RateTopic::Request;
  using Response = backbone::RateTopic::Response;

  auto HandleSubscribe(Request& in, Response& out) -> bool {
    auto upper_bound = std::upper_bound(
        rates_.begin(), rates_.end(), in.rate, [](auto const& a, auto const& b) { return a <= b; });
    if (upper_bound == rates_.end()) {
      return false;
    }

    out.topic = publishers_[std::distance(rates_.begin(), upper_bound)].TopicName();
    return true;
  }

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

  std::string const                      name_;         //!< the name of the Service
  std::vector<unsigned> const            rates_;        //!< array of who is subscribed to us
  ros::NodeHandle                        handle_;       //!< the handler of the node
  ros::ServiceServer                     subscribe_;    //!<
  ros::Subscriber                        input_;        //!< the subscriber
  std::vector<sa::kb::RatedPublisher<T>> publishers_{}; //!< array of who is subscribed to us
  ros::Publisher                         debugger_;     //!<
};

} // namespace kb
/** @} End of KB group*/
} // namespace sa
/** @} End of SA group*/
#endif // SA_KB_INCLUDE_RATED_TOPIC_H_
