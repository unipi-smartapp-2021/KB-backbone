#ifndef SA_KB_INCLUDE_FAIL_H_
#define SA_KB_INCLUDE_FAIL_H_

#include <ros/ros.h>

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

  /** \brief Function that send an error message after an error

      \param message the error message we are going to send
   */
[[noreturn]] auto Fail(std::string const& message) -> void {
  ROS_FATAL("%s", message.c_str());
  std::_Exit(EXIT_FAILURE);
}

} // namespace kb
  /** @} */
} // namespace sa
/** @} */
#endif // SA_KB_INCLUDE_FAIL_H_
