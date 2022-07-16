

ARG PROJECT
ARG DEBUG

FROM plotlab:latest AS plotlab

FROM csaps-cpp:latest AS csaps-cpp
FROM xodr:latest AS xodr
FROM dlib:v19.19 AS dlib
FROM qpoases:latest AS qpoases
FROM libadore:latest AS libadore
FROM adore_if_ros_msg:latest AS adore_if_ros_msg
FROM libzmq:latest AS libzmq
FROM cppzmq:latest AS cppzmq

FROM ros:noetic-ros-core-focal AS adore_if_ros_requirements_base

ARG PROJECT
ARG DEBUG
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

RUN mkdir -p /tmp/external
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/external
COPY --from=csaps-cpp /tmp/csaps-cpp/build/*.deb .
COPY --from=xodr /tmp/xodr/build/*.deb .
COPY --from=libzmq /tmp/libzmq /tmp/libzmq
COPY --from=cppzmq /tmp/cppzmq /tmp/cppzmq
#COPY --from=dlib /tmp/dlib/build/*.deb .
#COPY --from=qpoases /tmp/qpOASES/build/*.deb .
RUN dpkg -i *.deb 

WORKDIR /tmp/libzmq/build
RUN cmake --install . && ldconfig

WORKDIR /tmp/cppzmq/build
RUN cmake --install . && ldconfig

COPY --from=qpoases /tmp/qpOASES /tmp/qpOASES
WORKDIR /tmp/qpOASES/build
RUN cmake --install . && ldconfig

COPY ${PROJECT} /tmp/${PROJECT}/${PROJECT}

COPY --from=plotlab /tmp/plotlab /tmp/plotlab
COPY --from=libadore /tmp/libadore /tmp/libadore/libadore
COPY --from=adore_if_ros_msg /tmp/adore_if_ros_msg /tmp/adore_if_ros_msg

WORKDIR /tmp/adore_if_ros_msg/build
RUN cmake --install . && ldconfig


FROM adore_if_ros_requirements_base AS adore_if_ros_builder


ARG PROJECT
ARG DEBUG

RUN mkdir -p /tmp/${PROJECT}/${PROJECT}/build

SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/${PROJECT}/build


RUN source /opt/ros/noetic/setup.bash && \
    cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="install" && \
    cmake --build . -v --config Release --target install -- -j $(nproc)

# || [ $DEBUG == 1 ] && true
    #cmake --build . -- -j $(nproc)
    #cmake --build . --config Release --target install -- -j $(nproc)
    #make -j$(nproc) || true

#FROM alpine:3.14 AS adore_if_ros_package

#ARG PROJECT

#COPY --from=adore_if_ros_builder /tmp/${PROJECT}/build /tmp/${PROJECT}/build

