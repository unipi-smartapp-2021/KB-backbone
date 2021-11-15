#pragma once

#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/subscription_callback_helper.h"
namespace sa {
namespace kb {
class Subscriber {
 private:
  ros::NodeHandle                    topic;
  ros::Rate                          target_freq;
  ros::SubscriptionCallbackHelperPtr callback;
};
} // namespace kb
} // namespace sa