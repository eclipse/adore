ARG PROJECT=adore_if_ros
ARG DEBUG

ARG ADORE_IF_ROS_MSG_TAG=latest
ARG LIBADORE_TAG=latest
ARG PLOTLABLIB_TAG=latest

FROM adore_if_ros_msg:${ADORE_IF_ROS_MSG_TAG} AS adore_if_ros_msg
FROM libadore:${LIBADORE_TAG} AS libadore
FROM plotlablib:${PLOTLABLIB_TAG} AS plotlablib


FROM ros:noetic-ros-core-focal AS adore_if_ros_requirements_base

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.build.ubuntu20.04.system"
ARG REQUIREMENTS_FILE_PIP3="requirements.${PROJECT}.pip3"

RUN mkdir -p /tmp/${PROJECT}/${PROJECT}
COPY files/${REQUIREMENTS_FILE} /tmp/${PROJECT}
COPY files/${REQUIREMENTS_FILE_PIP3} /tmp/${PROJECT}
WORKDIR /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y $(sed '/^#/d' ${REQUIREMENTS_FILE}) && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install -r ${REQUIREMENTS_FILE_PIP3}

COPY ${PROJECT} /tmp/${PROJECT}/${PROJECT}


FROM adore_if_ros_requirements_base AS adore_if_ros_external_library_requirements_base

ARG INSTALL_PREFIX=/tmp/${PROJECT}/${PROJECT}/build/install
RUN mkdir -p "${INSTALL_PREFIX}"

ARG LIB=adore_if_ros_msg
COPY --from=adore_if_ros_msg /tmp/${LIB} /tmp/${LIB}
WORKDIR /tmp/${LIB}/${LIB}/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

ARG LIB=libadore
COPY --from=libadore /tmp/${LIB} /tmp/${LIB}
WORKDIR /tmp/${LIB}/${LIB}/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

ARG LIB=plotlablib
COPY --from=plotlablib /tmp/${LIB} /tmp/${LIB}
WORKDIR /tmp/${LIB}/${LIB}/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 


FROM adore_if_ros_external_library_requirements_base AS adore_if_ros_builder

ARG PROJECT

SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/${PROJECT}/build
RUN source /opt/ros/noetic/setup.bash && \
    cmake .. \
             -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
             -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX="install" && \
    cmake --build . -v --config Release --target install -- -j $(nproc) 


#FROM alpine:3.14 AS adore_if_ros_package

#ARG PROJECT

#COPY --from=adore_if_ros_builder /tmp/${PROJECT}/build /tmp/${PROJECT}/build

