#!/bin/bash

cleanup() {
    echo ""
    echo ""
    echo "..........Cleaning up............."
    echo ""
    kill -- -$$
}
trap cleanup SIGINT

# Define the name of the virtual environment directory
VENV_NAME=".venv"

# Check if the virtual environment directory exists
if [ -d "$VENV_NAME" ]; then
    echo "Virtual environment already exists."
else
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_NAME" --prompt="mekf"
fi

# Activate the virtual environment
echo "Activating virtual environment..."
source "$VENV_NAME/bin/activate"

# Run the Python script to check and install dependencies
echo "Checking and installing dependencies..."
python3 setup_dependencies.py

echo "Setup complete. Virtual environment is active."


mkdir -p build && cd build

cmake .. 2>/dev/null
cmake --build . 2>/dev/null

cd ..

java -cp /usr/local/share/java/lcm.jar lcm.lcm.TCPService 7700 &>/dev/null & 
sleep 0.5

if [ "$1" == "live" ]; then
    build/src/simple_ekf 2>/dev/null &

    lcm-logger -f -q --lcm-url="tcpq://localhost:7700" flightData/output_log.lcmlog 2>/dev/null &

    lcm-spy -l tcpq://localhost:7700 &>/dev/null &

    python3 src/publish_imu.py
else
    lcm-logger -f -q --lcm-url="tcpq://localhost:7700" flightData/output_log.lcmlog 2>/dev/null &

    build/src/simple_ekf 2>/dev/null &

    lcm-logplayer-gui -p -l tcpq://localhost:7700 $1 &>/dev/null

    wait
fi