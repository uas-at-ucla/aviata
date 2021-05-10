# Production Dockerfile. Should only be built in CI after code is built.
FROM ghcr.io/uas-at-ucla/aviata-dev
COPY . /aviata
