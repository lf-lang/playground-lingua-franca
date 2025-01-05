#!/bin/bash

# Array of test delays focusing around 1ms
delays=(0.25 0.5 0.75 0.8 0.9 1.0 1.1 1.2 1.5 2.0)

for delay in "${delays[@]}"; do
    sudo dnctl -f flush
    sudo pfctl -F all
    sudo dnctl pipe 1 config delay $delay
    sudo pfctl -f /dev/stdin <<EOF
dummynet-anchor "mytest"
anchor "mytest"
EOF
    sudo pfctl -a mytest -f /dev/stdin <<EOF
dummynet out proto {tcp,udp,icmp} from any to 127.0.0.1 pipe 1
dummynet in proto {tcp,udp,icmp} from 127.0.0.1 to any pipe 1
EOF
    sudo pfctl -E
    
    echo "Testing delay of ${delay}ms:"
    ping -c 10 127.0.0.1
    echo "------------------------"
done