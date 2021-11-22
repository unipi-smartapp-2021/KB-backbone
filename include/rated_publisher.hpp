/** \file rated_publisher.hpp

    File that contains the class for the realization of a Publisher that send messages at a certain frequency

    \author backbone_team
 */
#ifndef SA_KB_INCLUDE_RATED_PUBLISHER_H_
#define SA_KB_INCLUDE_RATED_PUBLISHER_H_

#include <ros/assert.h>

#include <cstdlib>

#include "fail.hpp"

/**
   \addtogroup Smart_Application
   @{
 */

//! Namespace that identify the whole Smart Application project.
namespace sa {

  /**
     \addtogroup Knowledge_Base
     @{
   */
  //! Namespace that identify the Knowledge Base part of the project.
namespace kb {

/** \brief Class for a publisher with a specific frequency rate

 */
template<typename T>
class RatedPublisher {
 public:
  /** \brief base constructor

      \param publisher the publisher we use
      \param rate the rate at witch we want to update our subscribers
   */
  RatedPublisher(ros::Publisher const& publisher, unsigned rate)
      : publisher_{publisher},
        update_gap_{1.0 / rate} {
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

  /** \brief Function that publish a message and send it to all the subscribers

      \param message the message we want to publish

      \return True if we publish the message
      \return False if we don't have subscribers to sent the message
      \return False if there's an expected update coming soon
   */
  auto Publish(boost::shared_ptr<T const> const& message) -> bool {
    ROS_ASSERT(publisher_ != nullptr && message != nullptr);

    auto now = ros::Time::now();
    if (!this->HasSubscribers() || now < next_update_) {
      return false;
    }

    publisher_.publish(message);
    next_update_ = now + update_gap_;
    return true;
  }

  /** \brief Function that publish a message to all the subscribers returning how much delayed it was

      \param message the message we want to publish

      \return {0, -1} if we couldn't publish the message
      \return ros::Duration how many milliseconds of delay there was between the message and the last update
   */
  [[nodiscard]] auto PublishAndDelay(boost::shared_ptr<T const> const& message) -> ros::Duration {
    ROS_ASSERT(publisher_ != nullptr && message != nullptr);

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
  ros::Publisher const publisher_;   //!< the publisher we are ratoing
  ros::Duration const  update_gap_;  //!< the rate at wich we want to update
  ros::Time            next_update_; //!< the time of the next update
};

} // namespace kb
  /** @} End of the kb group*/
} // namespace sa
/** @} End of the sa group*/
#endif // SA_KB_INCLUDE_RATED_PUBLISHER_H_
