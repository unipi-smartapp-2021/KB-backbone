// #ifndef SA_KB_INCLUDE_RATE_MULTIPLEXER_H_
// #define SA_KB_INCLUDE_RATE_MULTIPLEXER_H_

// #include <ros/init.h>
// #include <ros/node_handle.h>
// #include <ros/time.h>

// #include <queue>
// #include <utility>

// #include "rated_subscribe.hpp"

// namespace sa {

// namespace kb {

// namespace {

// class RatedSubscriber {
//  public:
//   RatedSubscriber(ros::NodeHandle destination, ros::Duration interval)
//       : destination_{destination},
//         interval_{interval} {
//     destination.
//   }

//  private:
//   ros::Publisher destination;
// };

// } // namespace

// template<typename T>
// class RateMultiplexer : public RatedSubscribe {
//  public:
//   /// Creates a multiplexer node of a given `topic`.
//   RateMultiplexer(std::string const& topic)
//       : source_{ros::NodeHandle().subscribe(topic, 1, [&](T message) { message_ = message; })} {}

//   auto SubscribeWithRate(ros::NodeHandle destination, ros::Rate frequency) -> void override {
//     subscribers_.emplace_back(destination, frequency.cycleTime());
//   }

//   auto Run() -> void {
//     while (ros::ok()) {
//     }
//   }

//  private:
//   ros::Subscriber                                        source_;
//   T                                                      message_;
//   std::vector<std::pair<ros::NodeHandle, ros::Duration>> subscribers_;
//   RateMultiplexer<T>::MinHeap                            update_times_;
// };

// } // namespace kb

// } // namespace sa

// #endif // SA_KB_INCLUDE_RATE_MULTIPLEXER_H_