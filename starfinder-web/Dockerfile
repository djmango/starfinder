# Build stage
FROM rust:1.92-bookworm AS builder

WORKDIR /app

# Copy the entire workspace since starfinder-web depends on starfinder
COPY Cargo.toml Cargo.lock ./
COPY src ./src
COPY starfinder-web ./starfinder-web

# Build the release binary
WORKDIR /app/starfinder-web
RUN cargo build --release

# Copy the downloaded catalog from build cache
RUN mkdir -p /catalog && \
    cp -r /root/.cache/starfinder /catalog/

# Runtime stage
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy the binary from builder
COPY --from=builder /app/starfinder-web/target/release/starfinder-web .

# Copy static files
COPY starfinder-web/static ./static

# Copy the pre-downloaded catalog
COPY --from=builder /catalog/starfinder /root/.cache/starfinder

EXPOSE 8080

ENV BIND_ADDR=0.0.0.0:8080
ENV RUST_LOG=info

CMD ["./starfinder-web"]
