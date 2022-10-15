FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get upgrade -y
RUN apt-get install ros-foxy-navigation2 -y

RUN mkdir -p /home/dev-ws/src

WORKDIR /home/dev-ws/src

RUN git clone https://github.com/packbionics/humanoid_navigation
RUN git clone https://github.com/packbionics/humanoid_msgs
RUN git clone https://github.com/sbpl/sbpl

WORKDIR /home/dev-ws

RUN /bin/bash -c ". /opt/ros/foxy/setup.bash; colcon build --packages-select sbpl gridmap_2d humanoid_nav_msgs"
RUN /bin/bash -c ". /opt/ros/foxy/setup.bash; . install/setup.bash; colcon build --packages-select footstep_planner"

CMD ["/bin/bash", "-c", ". install/setup.bash; ros2 launch footstep_planner footstep_planner_complete.launch.py"]
