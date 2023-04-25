FROM ubuntu:22.04

LABEL maintainer="alexandre.abadie@inria.fr"

# Install tools required for the build
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        git \
        libfontconfig1 \
        libfreetype6 \
        libuuid1 \
        libxcb1 \
        libxext6 \
        libxrender1 \
        wget \
        && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ARG SEGGER_STUDIO_VERSION=v712a
ARG SEGGER_STUDIO_ARCHIVE=Setup_EmbeddedStudio_ARM_${SEGGER_STUDIO_VERSION}_linux_x64.tar.gz
ARG SEGGER_STUDIO_URL=https://www.segger.com/downloads/embedded-studio/${SEGGER_STUDIO_ARCHIVE}
RUN echo 'Installing Segger Studio' >&2 && \
    mkdir -p /opt && \
    curl -L -o /opt/${SEGGER_STUDIO_ARCHIVE} ${SEGGER_STUDIO_URL} && \
    tar -C /opt -zxf /opt/${SEGGER_STUDIO_ARCHIVE} && \
    rm -f /opt/${SEGGER_STUDIO_ARCHIVE} && \
    /opt/arm_segger_embedded_studio_${SEGGER_STUDIO_VERSION}_linux_x64/install_segger_embedded_studio --accept-license --copy-files-to /opt/segger && \
    rm -rf /opt/arm_segger_embedded_studio_${SEGGER_STUDIO_VERSION}_linux_x64 && \
    rm -f /opt/${SEGGER_STUDIO_ARCHIVE}

RUN /opt/segger/bin/pkg update -packagesdir /opt/segger/packages
RUN /opt/segger/bin/pkg upgrade -packagesdir /opt/segger/packages
RUN /opt/segger/bin/pkg install -yes -packagesdir /opt/segger/packages CMSIS-CORE_V5 CMSIS-DSP_V5 nRF

RUN mkdir /dotbot
RUN git config --global --add safe.directory /dotbot

WORKDIR /dotbot
