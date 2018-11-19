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
    ros-kinetic-image-view \
    python-pip \
    python-wheel \
    python-snappy

RUN pip install -U catkin_tools

# create workspace and clone source
RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && \
	git clone https://github.com/anqixu/TelloPy.git && \
	git clone https://github.com/anqixu/h264_image_transport.git && \
	git clone https://github.com/anqixu/tello_driver.git  && \
	git clone https://github.com/yuokamoto/ros_tello.git

# install TelloPy
RUN cd /catkin_ws/src && \
	cd TelloPy && pip2 install -e .

# rosdep install
RUN cd /ros/catkin_ws && \
	rosdep update && \ 
	rosdep install -y --from-paths src --ignore-src --rosdistro kinetic

# install ffmpeg and av package
RUN apt-get install software-properties-common -y && \
	add-apt-repository ppa:jonathonf/ffmpeg-3 && \
	apt update && \
	apt-get install -y ffmpeg libav-tools x264 x265 libavdevice-dev && \
	pip install av

# build package
RUN	/bin/bash -c ". /opt/ros/kinetic/setup.bash && \ 
	cd /catkin_ws && catkin init && catkin build -j4 tello_driver"

# execute roslaunch
CMD /bin/bash -c ". /catkin_ws/devel/setup.bash && \
	 roslaunch tello_driver tello_node.launch"

# add entrypoint
RUN cp /catkin_ws/src/ros_tello/ros_entrypoint.sh .
ENTRYPOINT ["/ros_entrypoint.sh"]