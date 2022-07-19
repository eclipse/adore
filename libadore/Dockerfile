ARG PROJECT



FROM plotlab:latest as plotlab
FROM csaps-cpp:latest as csaps-cpp
FROM qpoases:latest as qpoases
FROM dlib:v19.19 as dlib
FROM xodr:latest as xodr
FROM catch2:v2.10.2 as catch2
FROM osqp:latest as osqp

FROM ubuntu:20.04 as libadore_builder

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"

RUN mkdir -p /tmp/${PROJECT}
COPY files/${REQUIREMENTS_FILE} /tmp/${PROJECT}
WORKDIR /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    DEBIAN_FRONTEND=noninteractive xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/external
WORKDIR /tmp/external
COPY --from=csaps-cpp /tmp/csaps-cpp/build/*.deb .
COPY --from=xodr /tmp/xodr/build/*.deb .
COPY --from=catch2 /tmp/Catch2/build/*.deb .
#COPY --from=plotlab /tmp/plotlab/lib/build/*.deb .
SHELL ["/bin/bash", "-c"]
RUN dpkg -i *.deb 


COPY ${PROJECT} /tmp/${PROJECT}

COPY --from=plotlab /tmp/plotlab /tmp/plotlab
WORKDIR /tmp/plotlab/lib/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

RUN mkdir -p /tmp/${PROJECT}/build && mkdir -p /tmp/${PROJECT}/build/install

WORKDIR /tmp/${PROJECT}/build

COPY --from=dlib /tmp/${PROJECT}/external/dlib /tmp/${PROJECT}/external/dlib
WORKDIR /tmp/${PROJECT}/external/dlib/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

COPY --from=xodr /tmp/xodr /tmp/xodr
WORKDIR /tmp/xodr/build
COPY --from=dlib /tmp/${PROJECT}/external/dlib /tmp/${PROJECT}/external/dlib
WORKDIR /tmp/${PROJECT}/external/dlib/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

COPY --from=qpoases /tmp/qpOASES /tmp/qpOASES
WORKDIR /tmp/qpOASES/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

WORKDIR /tmp/libadore/build
RUN cmake .. -DBUILD_adore_TESTING=ON -DCMAKE_PREFIX_PATH=install -DCMAKE_INSTALL_PREFIX:PATH=install && \
    cmake --build . --config Release --target install -- -j $(nproc)

WORKDIR /tmp/osqp/build
RUN cmake .. -DBUILD_adore_TESTING=ON -DCMAKE_PREFIX_PATH=install -DCMAKE_INSTALL_PREFIX:PATH=install && \
    cmake --build . --config Release --target install -- -j $(nproc) 


#FROM alpine:3.14 AS libadore_package

#ARG PROJECT

WORKDIR /tmp/${PROJECT}
#COPY --from=libadore_builder /tmp/${PROJECT} /tmp/${PROJECT}

