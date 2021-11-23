#ifndef SA_KB_INCLUDE_CONFIG_H_
#define SA_KB_INCLUDE_CONFIG_H_

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

constexpr auto kSourceRate = 30UL;              //!< Test rate for the source
constexpr auto kFirstClientRate = 1UL;          //!< Test rate for first client
constexpr auto kSecondClientRate = 4UL;         //!< Test rate for second client
constexpr auto kSubscriptionWaitTimeMs = 125UL; //!< Subscription cooldown
constexpr auto kTestTopic = "SensorTopic";      //!< Test topic name
constexpr auto kMillisInSecond = 1000UL;        //!< The amount of milliseconds in a second

} // namespace kb
/** @}*/
} // namespace sa
/** @}*/
#endif // SA_KB_INCLUDE_CONFIG_H_
