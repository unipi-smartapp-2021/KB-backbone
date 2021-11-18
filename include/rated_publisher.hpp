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

class DelayHistory {
 public:
  DelayHistory(std::uint32_t capacity)
      : capacity_{capacity},
        durations_{std::make_unique<ros::Duration[]>(capacity_)} {
    if (capacity_ == 0 || durations_ == nullptr) {
      sa::kb::Fail("Cannot create DelayHistory");
    }
  }

  inline auto Push(ros::Duration const& value) -> void {
    durations_[pushed_ % capacity_] = value;
    ++pushed_;
  }

  inline auto Push(ros::Duration&& value) -> void {
    durations_[pushed_ % capacity_] = value;
    ++pushed_;
  }

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
  std::uint32_t const              capacity_;
  std::unique_ptr<ros::Duration[]> durations_;
  std::uint32_t                    pushed_{};
};

template<typename T>
class RatedPublisher {
 public:
  RatedPublisher(ros::Publisher const& publisher, std::uint32_t rate)
      : publisher_{publisher},
        update_gap_{1.0 / rate},
        expected_next_update_{ros::Time::now()},
        delays_{sa::kb::kDelayHistorySize} {
    if (publisher_ == nullptr) {
      sa::kb::Fail("Invalid publisher provided");
    }
  }

  [[nodiscard]] inline auto HasSubscribers() const -> bool {
    ROS_ASSERT(publisher_ != nullptr);
    return publisher_.getNumSubscribers() > 0;
  }

  [[nodiscard]] inline auto TopicName() const -> std::string {
    return publisher_.getTopic();
  }

  [[nodiscard]] auto MovingAverageDelay() const -> ros::Duration {
    return delays_.Mean();
  }

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
  ros::Publisher const publisher_;
  ros::Duration const  update_gap_;
  ros::Time            expected_next_update_;
  sa::kb::DelayHistory delays_;
};

} // namespace kb

} // namespace sa

#endif // SA_KB_INCLUDE_RATED_PUBLISHER_H_