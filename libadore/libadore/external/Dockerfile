ARG PROJECT

FROM ubuntu:20.04 as libadore_external_library_builder

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"


RUN mkdir -p /tmp/${PROJECT}
COPY files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

WORKDIR /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    DEBIAN_FRONTEND=noninteractive xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY ${PROJECT} /tmp/${PROJECT}

WORKDIR /tmp/${PROJECT}
RUN mkdir -p build
WORKDIR /tmp/${PROJECT}/build
#RUN rm -rf build && \
#    mkdir -p build && \
#    cd build && \
#    cmake .. && \
#    make -j$(nproc)
RUN cmake .. -DCMAKE_INSTALL_PREFIX:PATH=install && \
    cmake --build . -- -j $(nproc)
    #cmake --build . --config Release --target install -- -j $(nproc)


WORKDIR /tmp/${PROJECT}/build
RUN cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . || true

FROM alpine:3.14

ARG PROJECT

COPY --from=libadore_external_library_builder /tmp/${PROJECT} /tmp/${PROJECT}

