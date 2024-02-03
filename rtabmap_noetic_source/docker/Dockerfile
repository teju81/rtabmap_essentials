FROM osrf/ros:noetic-desktop-full

## Set environment variable to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
	mlocate \
	gedit \
	git

RUN mkdir ~/catkin_ws/src -p
RUN cd ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; catkin_init_workspace'
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /root/catkin_ws; catkin_make'
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN apt install -y ros-$ROS_DISTRO-rtabmap*
RUN cd ~
RUN git clone https://github.com/introlab/rtabmap.git rtabmap
RUN cd rtabmap/build
#RUN cmake ..
#RUN make -j6
#RUN make install
#RUN cd ~/catkin_ws
#RUN git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
#RUN catkin_make -j4
WORKDIR /rtabmap/build