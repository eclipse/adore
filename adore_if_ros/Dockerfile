ARG PROJECT
ARG DEBUG


FROM adore_if_ros_msg:latest AS adore_if_ros_msg
FROM libadore:latest AS libadore

FROM plotlablib:latest AS plotlablib
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

COPY ${PROJECT} /tmp/${PROJECT}/${PROJECT}

FROM adore_if_ros_requirements_base AS adore_if_ros_external_library_requirements_base

ARG INSTALL_PREFIX=/tmp/${PROJECT}/${PROJECT}/build/install
RUN mkdir -p "${INSTALL_PREFIX}"

ARG LIB=adore_if_ros_msg
COPY --from=adore_if_ros_msg /tmp/${LIB} /tmp/${PROJECT}/${LIB}/${LIB}
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/${LIB}/${LIB}/build
RUN source /opt/ros/noetic/setup.bash && \
    cmake .. > /dev/null 2>&1 || true && \
    cmake --install . --prefix ${INSTALL_PREFIX}
RUN dpkg -i *.deb ## TODO fix adore_if_ros cmake library so this is not necessary

ARG LIB=libadore
COPY --from=libadore /tmp/${LIB} /tmp/${LIB}
WORKDIR /tmp/${LIB}/${LIB}/build
RUN cmake .. -DCMAKE_PREFIX_PATH=`realpath install` && \
    cmake --install . --prefix ${INSTALL_PREFIX}

ARG LIB=plotlablib
COPY --from=plotlablib /tmp/${LIB} /tmp/${PROJECT}/${LIB}
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/${LIB}/${LIB}/build
RUN cmake .. -DCMAKE_PREFIX_PATH=`realpath install` && \
    cmake --install . --prefix ${INSTALL_PREFIX}
RUN dpkg -i *.deb

ARG LIB=libzmq
COPY --from=libzmq /tmp/${LIB} /tmp/${LIB}
WORKDIR /tmp/${LIB}/build
RUN cmake --install . --prefix ${INSTALL_PREFIX}

ARG LIB=cppzmq
COPY --from=cppzmq /tmp/${LIB} /tmp/${LIB}
WORKDIR /tmp/${LIB}/build
RUN cmake --install . --prefix ${INSTALL_PREFIX}


FROM adore_if_ros_external_library_requirements_base AS adore_if_ros_builder

ARG PROJECT
ARG DEBUG

SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/${PROJECT}/build
RUN source /opt/ros/noetic/setup.bash && \
    cmake .. \
             -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
             -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX="install" && \
    cmake --build . -v --config Release --target install -- -j $(nproc) 


#ARG CMAKE_PREFIX_PATH="/opt/ros/noetic;/tmp/${PROJECT}/${PROJECT}/build/install;/tmp/libadore/libadore/build/install;/tmp/plotlablib/plotlablib/build/install"
#             -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH} && \

#FROM alpine:3.14 AS adore_if_ros_package

#ARG PROJECT

#COPY --from=adore_if_ros_builder /tmp/${PROJECT}/build /tmp/${PROJECT}/build

