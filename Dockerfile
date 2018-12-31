FROM rrdockerhub/ros-base-kinetic-amd64

LABEL authors = "Yu Okamoto <yu.okamoto@rapyuta-robotics.com>"

ENV ROS_HOME=/catkin_ws

# install base software
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    ros-kinetic-rospy \
    ros-kinetic-roslaunch \
    build-essential \
    ros-kinetic-actionlib-msgs \
    ros-kinetic-image-transport-plugins \
    python-pip \
    python-wheel \
    python-snappy

RUN pip install -U catkin_tools

# create workspace and clone source
RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src  && \ 
	git clone https://github.com/yuokamoto/ros_tello.git

# submodule update
RUN cd /catkin_ws/src/ros_tello && \
	git submodule init && git submodule update

# install and prep
RUN cd /catkin_ws/src/ros_tello && \
	sed -i -e "s/sudo//g" scripts/tellolib/Tello_Video/install/Linux/linux_install.sh &&  \
	/bin/bash -c "source setup.sh"

# SHELL ["/bin/bash", "-c", "/catkin_ws/src/ros_tello/setup.sh"]

# rosdep install
RUN cd /catkin_ws && \
	rosdep update && \ 
	rosdep install -y --from-paths src --ignore-src --rosdistro kinetic


# build package
RUN	/bin/bash -c ". /opt/ros/kinetic/setup.bash && \ 
	cd /catkin_ws && catkin init && catkin build -j4 ros_tello"

# execute roslaunch
CMD ["roslaunch ros_tello tello.launch"]

# add entrypoint
RUN cp /catkin_ws/src/ros_tello/ros_entrypoint.sh .
ENTRYPOINT ["/ros_entrypoint.sh"]