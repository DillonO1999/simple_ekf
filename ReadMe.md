# Simple EKF Implementation

## Required dependencies

Cmake 
- Install Glib2 with `sudo apt update && sudo apt install libglib2.0-dev`
- Install Cmake with `sudo apt install cmake`

Java
- Install java with `sudo apt install default-jdk`.

LCM (lightweight communications & Marshalling)  
- git clone LCM (lightweight communications & Marshalling) from [source](https://github.com/lcm-proj/lcm.git).
- Run these commands in the lcm directory:
```bash
mkdir build && cd build
cmake -DLCM_ENABLE_JAVA=ON ..
make && sudo make install
```

Eigen 
- install Eigen using `sudo apt install libeigen3-dev`.

yaml-cpp
- install yaml-cpp with `sudo apt install libyaml-cpp-dev`.

## How to run

### From flight log

From project root directory (simple_ekf), run:

```bash
./start <path/to/lcm_flight_log> 

# Example
./start flightData/my_flight_log.lcmlog
```

### Live flight data

From project root directory (simple_ekf), run:

```bash
./start live
```

## How to plot

```bash
./plot.sh
```

## Optional Steps

Add Java and messages jar file to your path in bashrc file to see data streams in lcm-spy (double click datatype)
- The bottom of your bashrc should look something like:
```bash
export JAVA_HOME="/usr/lib/jvm/java-8-openjdk-amd64"
export JDK_HOME="/usr/lib/jvm/java-8-openjdk-amd64"
export CLASSPATH="/home/dillon/lcm_messages.jar:$CLASSPATH"
```