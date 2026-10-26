# eSync Linux SDK container
# Build: podman build -t esync-sdk .
# Run:   podman run -d --name esync esync-sdk qcar123
# Run:   podman run -d --name esync -v ~/Downloads/esync-campaigns:/downloads esync-sdk qcar123
# Run:   podman run -d --name esync --net=host --cap-add=NET_ADMIN -v ~/Downloads/esync-campaigns:/downloads esync-sdk qcar123
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Added can-utils to the package installation list
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo tini ca-certificates procps iproute2 iputils-ping jq zip unzip openssl \
        libssl-dev libsqlcipher-dev libsqlite3-dev libcurl4-openssl-dev libxml2-dev \
        libtspi-dev libzip-dev libncurses-dev libzstd-dev libproc2-dev can-utils \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu && chmod 0440 /etc/sudoers.d/ubuntu

COPY --chown=ubuntu:ubuntu . /opt/esync-sdk
RUN chmod +x /opt/esync-sdk/*.sh

WORKDIR /home/ubuntu

ENTRYPOINT ["/usr/bin/tini", "--", "/opt/esync-sdk/docker-entrypoint.sh"]
