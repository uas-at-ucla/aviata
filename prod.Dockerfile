# Production Dockerfile. Should only be built in Travis after code is built.
FROM uasatucla/aviata-dev
COPY . /workspace
