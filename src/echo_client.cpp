#include <cstdlib>

#include "backbone/ThrottleTopic.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "throttle_client");

  ros::NodeHandle         n;
  ros::ServiceClient      client = n.serviceClient<backbone::ThrottleTopic>("ThrottleTopic");
  backbone::ThrottleTopic srv;
  srv.request.rate = 10;
  auto rate = ros::Rate(10);
  while (ros::ok()) {
    if (client.call(srv)) {
      ROS_INFO("new topic name %s ", srv.response.new_topic);
    } else {
      ROS_ERROR("Failed to call throttletopic");
      return 1;
    }
    rate.sleep();
  }
  return 0;
}