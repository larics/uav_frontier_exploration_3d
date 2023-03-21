#!/bin/bash

CONTAINER_NAME=$1
[ -z "$CONTAINER_NAME" ] && CONTAINER_NAME=frontier_exp

ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
docker start $CONTAINER_NAME
docker attach $CONTAINER_NAME
