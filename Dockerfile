FROM ros:noetic-ros-base

ENV ROS_WS=/home/ws

RUN mkdir -p $ROS_WS/src

COPY ./ $ROS_WS/src
WORKDIR $ROS_WS

RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && cd $ROS_WS && catkin_make \
  && sed --in-place --expression \ 
  '$isource "$ROS_WS/devel/setup.bash"' \
  /ros_entrypoint.sh

CMD [ "catkin_make"]