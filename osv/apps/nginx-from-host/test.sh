#!/bin/bash

THIS_DIR=$(readlink -f $(dirname $0))
CMDLINE=$($THIS_DIR/../cmdline.sh $THIS_DIR)

$THIS_DIR/../../scripts/tests/test_http_app.py \
  -e "$CMDLINE" \
  --http_path "/nginx.conf" \
  --guest_port 8000 \
  --host_port 8000 \
  --line 'epoll_wait' \
  --http_line 'worker_connections' \
  --count 10000
