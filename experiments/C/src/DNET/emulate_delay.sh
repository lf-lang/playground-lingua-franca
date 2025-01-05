#!/bin/bash

sudo dnctl -f flush
sudo pfctl -F all

sudo dnctl pipe 1 config delay 2.5
sudo pfctl -f /dev/stdin <<EOF
dummynet-anchor "mytest"
anchor "mytest"
EOF
sudo pfctl -a mytest -f /dev/stdin <<EOF
dummynet out proto {tcp,udp,icmp} from any to 127.0.0.1 pipe 1
dummynet in proto {tcp,udp,icmp} from 127.0.0.1 to any pipe 1
EOF
sudo pfctl -E