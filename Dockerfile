# Stage 1: build
FROM debian:bookworm-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake make gcc \
    libbluetooth-dev \
    libnl-3-dev libnl-genl-3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j$(nproc)

# Stage 2: runtime
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    libbluetooth3 \
    libnl-3-200 libnl-genl-3-200 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /src/build/odid-daemon /usr/local/bin/odid-daemon

ENTRYPOINT ["/usr/local/bin/odid-daemon"]
CMD ["--no-wifi"]
