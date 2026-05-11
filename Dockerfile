# Stage 1: build
FROM debian:trixie AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake make gcc \
    libbluetooth-dev \
    libnl-3-dev libnl-genl-3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY CMakeLists.txt .
COPY src/ src/
COPY lib/ lib/
COPY tests/ tests/

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j$(nproc)

# Stage 2: runtime
FROM debian:trixie

RUN apt-get update && apt-get install -y --no-install-recommends \
    libbluetooth3 \
    libnl-3-200 libnl-genl-3-200 \
    rfkill \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /src/build/odid-daemon /usr/local/bin/odid-daemon
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["--no-wifi"]
