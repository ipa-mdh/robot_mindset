#!/bin/bash


docker build --file .dev-setup/docker/Dockerfile.base \
    --secret id=gitcreds,src=$HOME/.git-credentials \
    --tag robot_mindset_base:1.0 \
    .
docker build --file .dev-setup/docker/Dockerfile.run \
    --secret id=gitcreds,src=$HOME/.git-credentials \
    --tag robot_mindset_run:1.0 \
    .