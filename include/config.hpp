#ifndef SA_KB_INCLUDE_CONFIG_H_
#define SA_KB_INCLUDE_CONFIG_H_

namespace sa {

namespace kb {

constexpr auto kSourceRate = 30UL;
constexpr auto kFirstClientRate = 1UL;
constexpr auto kSecondClientRate = 4UL;
constexpr auto kSubscriptionWaitTimeMs = 125UL;
constexpr auto kTestTopic = "SensorTopic";
constexpr auto kMillisInSecond = 1'000UL;

} // namespace kb

} // namespace sa

#endif // SA_KB_INCLUDE_CONFIG_H_
