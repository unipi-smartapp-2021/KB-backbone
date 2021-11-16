#ifndef SA_KB_INCLUDE_RATED_SUBSCRIBE_H_
#define SA_KB_INCLUDE_RATED_SUBSCRIBE_H_

#include <ros/node_handle.h>

namespace sa {

namespace kb {

class RatedSubscribe {
 public:
  virtual auto SubscribeWithRate(ros::NodeHandle handle, ros::Rate) -> void = 0;
};

} // namespace kb

} // namespace sa

#endif // SA_KB_INCLUDE_RATED_SUBSCRIBE_H_