#!/bin/bash

cleanup() {
    echo ""
    echo ""
    echo "..........Cleaning up............."
    echo ""
    kill -- -$$
}
trap cleanup SIGINT

mkdir -p build && cd build

cmake .. 2>/dev/null
cmake --build . 2>/dev/null

cd ..

echo ""
echo ""
echo "Running simple_ekf..."
echo "----------------------------------------"
echo ""

java -cp /usr/share/java/lcm.jar lcm.lcm.TCPService 7700 &>/dev/null & 
sleep 0.5

lcm-logger -f -q --lcm-url="tcpq://localhost:7700" flightData/output_log.lcmlog 2>/dev/null &

build/src/simple_ekf 2>/dev/null &

lcm-logplayer-gui -p -l tcpq://localhost:7700 $1 &>/dev/null

wait