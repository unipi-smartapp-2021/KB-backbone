/// \file rated_publisher.hpp
///
/// File containing both definition and implementation of the [`RatedPublisher`] class.
///
/// [`RatedPublisher`]: ::RatedPublisher
///
/// \author backbone_team
#ifndef SA_KB_INCLUDE_RATED_PUBLISHER_H_
#define SA_KB_INCLUDE_RATED_PUBLISHER_H_

#include <cstdlib>

#include "fail.hpp"

/** \addtogroup Smart_Application 
    @{ 
*/
namespace sa {

/** \addtogroup Knowledge_Base 
    @{ 
*/
namespace kb {

/// A wrapper for a [`ros::Publisher`] that allows to specify a **maximum rate** of updates on the
/// associated topic.
///
/// [`ros::Publisher`]: ros::Publisher
///
/// # Description
/// This class is particularly useful when there is a need to **limit** the amount of messages sent
/// on a given topic in case the sending frequency is much higher than that of the processing nodes.
/// In this way, whenever an attempt is made to publish a message on the wrapped topic, if the minimum
/// waiting time has not yet elapsed, the operation is **not** performed.
///
/// The minimum time between two successive publications is defined as `1 / rate` milliseconds.
template<typename T>
class RatedPublisher {
 public:
  /// \brief Wraps the provided `publisher` and limits its maximum `rate` at the desired level.
  ///
  /// \param publisher Publisher whom rate must be limited.
  /// \param rate Maximum frequency of publishing.
  ///
  /// ### Failures
  /// If `publisher == nullptr` or `rate == 0` the constructor aborts calling [`Fail()`].
  ///
  /// [`Fail()`]: sa::kb::Fail
  RatedPublisher(ros::Publisher const& publisher, unsigned rate)
      : publisher_{publisher},
        update_gap_{rate == 0 ? 1.0 : 1.0 / rate} {
    if (publisher_ == nullptr) {
      sa::kb::Fail("Invalid publisher provided");
    }

    if (rate == 0) {
      sa::kb::Fail("Rate cannot be `0`");
    }
  }

  /// \brief Checks whether the wrapped topic has **at least** 1 subscriber.
  ///
  /// \return True if subscribers > 0
  /// \return False if subscribers == 0
  [[nodiscard]] inline auto HasSubscribers() const -> bool {
    return publisher_.getNumSubscribers() > 0;
  }

  /// \brief Retrieves the name of the wrapped topic.
  ///
  /// \return the name of the topic in std::string format
  [[nodiscard]] inline auto TopicName() const -> std::string {
    return publisher_.getTopic();
  }

  /// \brief Attempts to publish a `message` on the wrapped topic.
  ///
  /// In case the minimum waiting time has not yet elapsed, or if nobody is subscribed to the topic,
  /// `false` is returned, otherwise `true`.
  ///
  /// \param message The message to publish on the wrapped topic.
  ///
  /// \return a boolean describing if we published the message or not
  auto Publish(boost::shared_ptr<T const> const& message) -> bool {
    auto now = ros::Time::now();
    if (!this->HasSubscribers() || now < next_update_) {
      return false;
    }

    publisher_.publish(message);
    next_update_ = now + update_gap_;
    return true;
  }

  /// \brief Attempts to publish a `message` on the wrapped topic,
  /// while also computing the delay between `expected` and `real` publish times.
  ///
  /// In case the minimum waiting time has not yet elapsed, or if nobody is subscribed to the topic,
  /// a [`ros::Duration`] with **negative** nanoseconds is returned, otherwise `expected - real`.
  /// Please, note that *on the very first publish, the returned duration will be `0`*.
  ///
  /// [`ros::Duration`]: ros::Duration
  ///
  /// \param message The message to publish on the wrapped topic.
  ///
  /// \return The delay between the last update and this message being published
  [[nodiscard]] auto PublishAndDelay(boost::shared_ptr<T const> const& message) -> ros::Duration {
    auto now = ros::Time::now();
    if (!this->HasSubscribers() || now < next_update_) {
      return {0, -1};
    }

    publisher_.publish(message);
    auto delay = next_update_.toNSec() == 0 ? ros::Duration(0, 0) : now - next_update_;
    next_update_ = now + update_gap_;
    return delay;
  }

 private:
  ros::Publisher const publisher_;   /// Publisher whom rate must be enforced
  ros::Duration const  update_gap_;  /// Minimum time between two consecutive publish
  ros::Time            next_update_; /// Time instant from which a publish can succeed
};

} // namespace kb
/** @} Knowledge_Base */

} // namespace sa
/** @} Smart_Application */

#endif // SA_KB_INCLUDE_RATED_PUBLISHER_H_
