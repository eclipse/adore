FROM ubuntu:focal
ARG USER
ARG UID
ENV LANG C.UTF-8
RUN apt-get update && \
	apt-get install -y sudo vim apt-transport-https ca-certificates gnupg software-properties-common wget apt-utils && \
	export DEBIAN_FRONTEND=noninteractive && \
	apt-get install -y \
	cmake \
	libboost-dev \
	libboost-date-time-dev \
	libboost-system-dev \
	libboost-chrono-dev \
	libboost-thread-dev \
	libblas-dev \
	libopenblas-dev \
	liblapack-dev \
	libzmq5 \
	libzmq3-dev \
	freeglut3-dev \
	libxerces-c-dev \
	libcurl4-openssl-dev \
	libdlib-dev \
	screen \
	doxygen \
	git \
	apt-utils \
	lsb-release \
	gnupg \
	wget \
	software-properties-common \
	unzip \
	wget \
	xsdcxx && \
	sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
	apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
	apt-get update && \
	apt-get install -y ros-noetic-desktop-full python3-catkin-tools python3-roslaunch python3-pip python3-rosdep psmisc ros-noetic-ibeo-msgs && \
	rosdep init && \
	rosdep update && \
	apt-get install autoconf automake libtool curl make g++ unzip 
RUN cd /usr/bin && ln -s python3 python
RUN echo "UID=${UID}, USER=${USER}"
RUN useradd -rm -d "/home/${USER}" -s /bin/bash -g root -G sudo -u "${UID}" "${USER}" 
RUN echo "${USER}:ubuntu">asdf && cat asdf|chpasswd 
USER $USER
WORKDIR /home/$USER
RUN pip3 install pyyaml && \
	pip3 install rospkg && \
	pip3 install osrf-pycommon && \
	pip3 install pydot && \
	pip3 install graphviz && \
	pip3 install tf-agents 
RUN	echo "source /opt/ros/noetic/setup.bash" >> "/home/${USER}/.bashrc" 
RUN	echo "source /home/${USER}/catkin_ws/install/setup.bash" >> "/home/${USER}/.bashrc" 
RUN echo "cd /home/${USER}/catkin_ws/" >> "/home/${USER}/.bashrc" 
RUN cd "/home/${USER}/catkin_ws/" && catkin config --install

FROM alpine:3.14

ARG PROJECT
COPY --from=sumo_builder /tmp/${PROJECT}/.build /tmp/${PROJECT}/.build

