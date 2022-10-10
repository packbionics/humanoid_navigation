FROM osrf/ros:kinetic-desktop

RUN apt-get update && apt-get upgrade -y

RUN mkdir -p /home/catkin_ws/src

WORKDIR /home/catkin_ws/src
RUN git clone https://github.com/ROBOTIS-GIT/humanoid_navigation

WORKDIR /home/catkin_ws/src/humanoid_navigation
RUN git checkout kinetic-devel

WORKDIR /home/catkin_ws
RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash; rosdep install -i --from-path src --rosdistro kinetic -y; catkin_make"

CMD ["/bin/bash", "-c", ". devel/setup.bash; roslaunch footstep_planner footstep_planner_complete.launch"]
