# Extended Kalman Filter  
## Pedestrian Tracking using measurements from RADAR and LIDAR  
Self-Driving Car Engineer Nanodegree Program  
Term 2. Project 1

---
Kalman Filter is a mathematical approach of determining the state of the system.  
It calculates the system's state using mathematical model of the process and clarifies the state using the measurement information.  
  
Sensor fusion - is a process of combining measurements from different sensors to get one accurate picture.  
Sensor Fusion for Pedestrian Tracking using RADAR and LIDAR sensors is an actual task for the self-driving car. In this project we use Extended Kalman Filter to combine data from RADAR and LIDAR to track pedestrian position and velocity.  
  
![alt tag](https://github.com/olegleyz/SDCND-T2P1-EKF/blob/master/process-flow.png)
Image: Udacity Self-Driving Car Nanodegree  
    
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Usage

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake ../src && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./kalman2D path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./kalman2D ../data/sample-laser-radar-measurement-data-1.txt ../data/sample-laser-radar-measurement-data-1-output.txt`

