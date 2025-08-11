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

cmake .. 2> /dev/null
cmake --build . 2> /dev/null
sudo cmake --install . --prefix /usr/local > /dev/null

cd ..

echo ""
echo ""
echo "Running simple_ekf..."
echo "----------------------------------------"
echo ""

build/src/myApp &

wait