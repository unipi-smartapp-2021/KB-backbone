#ifndef SA_KB_INCLUDE_RATED_PUBLISHER_H_
#define SA_KB_INCLUDE_RATED_PUBLISHER_H_

#include <ros/assert.h>

#include <cstdlib>

#include "fail.hpp"

namespace sa {

namespace kb {

namespace {

constexpr auto kDelayHistorySize = 8UL;

}

  /** \brief Class that save the history of the messages arrived with delays

   */
class DelayHistory {
 public:
  /** \brief base construct of the class

      \param the max capacity of the stored delayed messages
   */
  DelayHistory(std::uint32_t capacity)
      : capacity_{capacity},
        durations_{std::make_unique<ros::Duration[]>(capacity_)} {
    if (capacity_ == 0 || durations_ == nullptr) {
      sa::kb::Fail("Cannot create DelayHistory");
    }
  }

  /** \brief Function that save a delay  

      \param value the delay of the currently pushed message
   */
  inline auto Push(ros::Duration const& value) -> void {
    durations_[pushed_ % capacity_] = value;
    ++pushed_;
  }

  /** \brief Function that save a delay

      \param value the delay of the currently pushed message
   */
  inline auto Push(ros::Duration&& value) -> void {
    durations_[pushed_ % capacity_] = value;
    ++pushed_;
  }

  /** \brief Function that calculate the mean of the delays

      \return the mean of all the currents durations
   */
  [[nodiscard]] auto Mean() const -> ros::Duration {
    if (pushed_ == 0) {
      return {0, 0};
    }

    auto const size = std::min(capacity_, pushed_);
    auto       total = ros::Duration(0, 0);
    for (auto i = 0UL; i < size; ++i) {
      total += durations_[i];
    }
    return {0, static_cast<int32_t>(total.toNSec() / size)};
  }

 private:
  std::uint32_t const              capacity_; //!< the total capacity of the History
  std::unique_ptr<ros::Duration[]> durations_;//!< the array of Duration of the pushed messages
  std::uint32_t                    pushed_{}; //!< the number of total pushed messages
};

template<typename T>
/** \brief Class for a publisher with a specific frequency rate

 */
class RatedPublisher {
 public:
  /** \brief base constructor

      \param publisher the publisher we use
      \param rate the rate at witch we want to update our subscribers
   */
  RatedPublisher(ros::Publisher const& publisher, std::uint32_t rate)
      : publisher_{publisher},
        update_gap_{1.0 / rate},
        expected_next_update_{ros::Time::now()},
        delays_{sa::kb::kDelayHistorySize} {
    if (publisher_ == nullptr) {
      sa::kb::Fail("Invalid publisher provided");
    }
  }

  /** \brief Function that ches if we have subscribers

      \return True if we have >0 subscribers
      \return False if we have ==0 subscribers
   */
  [[nodiscard]] inline auto HasSubscribers() const -> bool {
    ROS_ASSERT(publisher_ != nullptr);
    return publisher_.getNumSubscribers() > 0;
  }

  /** \brief Function that return the name of the Topic we are publishing

      \return the name of the topic we are publishing
   */
  [[nodiscard]] inline auto TopicName() const -> std::string {
    return publisher_.getTopic();
  }

  /** \brief Function that tell what is the average duration of the messages

      \return the mean of the delays of the current messages
   */
  [[nodiscard]] auto MovingAverageDelay() const -> ros::Duration {
    return delays_.Mean();
  }

  /** \brief Function that publish a message and send it to all the subscribers

      \param message the message we want to publish

      \return True if we publish the message
      \return False if we don't have subscribers to sent the message
      \return False if there's an expected update coming soon
   */
  auto Publish(boost::shared_ptr<T const> const& message) -> bool {
    ROS_ASSERT(publisher_ != nullptr && message != nullptr);

    auto now = ros::Time::now();
    if (!this->HasSubscribers() || now < expected_next_update_) {
      return false;
    }
    publisher_.publish(message);

    delays_.Push(now - expected_next_update_);
    expected_next_update_ = now + update_gap_;
    return true;
  }

 private:
  ros::Publisher const publisher_; //!< the publisher we are ratoing
  ros::Duration const  update_gap_; //!< the rate at wich we want to update
  ros::Time            expected_next_update_; //!< the time of the next update
  sa::kb::DelayHistory delays_; //!< the array of delays
};

} // namespace kb

} // namespace sa

#endif // SA_KB_INCLUDE_RATED_PUBLISHER_H_
