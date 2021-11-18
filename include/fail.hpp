#ifndef SA_KB_INCLUDE_FAIL_H_
#define SA_KB_INCLUDE_FAIL_H_

#include <ros/ros.h>

namespace sa {

namespace kb {

[[noreturn]] auto Fail(std::string const& message) -> void {
  ROS_FATAL("%s", message.c_str());
  std::_Exit(EXIT_FAILURE);
}

} // namespace kb

} // namespace sa

#endif // SA_KB_INCLUDE_FAIL_H_