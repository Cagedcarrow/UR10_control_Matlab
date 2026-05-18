# hash:sha256:42994865528339eeba2c44183a9ee9b7863a813c7c3ce21b904795c8338a18a5
FROM registry.codeocean.com/codeocean/ubuntu-desktop:ubuntu-22.04-guacamole-1.5.2

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        apt-utils=2.4.12 \
        curl=7.81.0-1ubuntu1.16 \
        locales=2.35-0ubuntu3.8 \
        software-properties-common=0.99.22.9 \
    && rm -rf /var/lib/apt/lists/*

COPY postInstall /
RUN /postInstall
