# Simple EKF Implementation

## Required dependencies

LCM (lightweight communications & Marshalling)  
- Install LCM (lightweight communications & Marshalling) from the instructions [here](https://lcm-proj.github.io/lcm/content/install-instructions.html#installing-lcm).

Eigen 
- install Eigen using `sudo apt update && sudo apt install libeigen3-dev` or install from [source](https://gitlab.com/libeigen/eigen).

yaml-cpp
- install yaml-cpp for reading yaml config file from the ReadMe instructions [here](https://github.com/jbeder/yaml-cpp).

#### For plotting

You will need python and the following packages:

- numpy 
- matplotlib
- lcm (might already be included in newer python versions)
- yaml
- scipy

## How to run

From project root directory (simple_ekf), run:

```bash
./start <path/to/lcm_flight_log> 

# Example
./start flightData/my_flight_log.lcmlog
```

## Optional Steps

Add Java and messages jar file to your path in bashrc file to see data streams in lcm-spy (double click datatype)
- The bottome of your bashrc should look something like:
```bash
export JAVA_HOME="/usr/lib/jvm/java-8-openjdk-amd64"
export JDK_HOME="/usr/lib/jvm/java-8-openjdk-amd64"
export CLASSPATH="/home/dillon/lcm_messages.jar:$CLASSPATH"
```