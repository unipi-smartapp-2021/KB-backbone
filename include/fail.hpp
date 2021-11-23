#ifndef SA_KB_INCLUDE_FAIL_H_
#define SA_KB_INCLUDE_FAIL_H_

#include <ros/ros.h>

/** \addtogroup Smart_Application @{ */
namespace sa {

/** \addtogroup Knowledge_Base @{ */
namespace kb {

/// Aborts the program, printing the error `message` provided using [`ROS_FATAL`].
///
/// [`ROS_FATAL`]: ROS_FATAL
///
/// # Arguments
/// - `message`: The error message to print
[[noreturn]] auto Fail(std::string const& message) -> void {
  ROS_FATAL("%s", message.c_str());
  std::_Exit(EXIT_FAILURE);
}

} // namespace kb
/** @} Knowledge_Base */

} // namespace sa
/** @} Smart_Application */

#endif // SA_KB_INCLUDE_FAIL_H_
