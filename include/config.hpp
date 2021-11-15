#ifndef SA_INCLUDE_CONFIG_H_
#define SA_INCLUDE_CONFIG_H_

namespace sa {

constexpr auto kLoopFrequency = 5UL;
constexpr auto kChatterQueueSize = 128UL;
constexpr auto kSubscriptionWaitTime = 125UL;
constexpr auto kTestTopic = "Hello_World";

} // namespace sa

#endif // SA_INCLUDE_CONFIG_H_