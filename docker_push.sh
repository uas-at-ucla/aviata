#!/bin/bash
docker-compose build prod
echo "$DOCKER_PASSWORD" | docker login -u uasatucla --password-stdin
docker-compose push dev
docker-compose push prod
