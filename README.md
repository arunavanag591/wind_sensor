# Wind Tunnel Fixture

This package includes python nodes than drives the wind sensor in a x-y fixture

## Table of Contents

1. [System Requirement](#system-requirements)
2. [Package Overview](#package-overview)
    
    2a. [Directory Structure](#directory-structure)

3. [Usage](#usage)



## System Requirements

The following software needs the below mentioned setup:
* <a href="http://wiki.ros.org/melodic/Installation/Ubuntu">ROS Melodic</a>
* <a href = "https://www.phidgets.com/docs/OS_-_Linux"> LibPhidgets22</a>
* <a href = "https://www.phidgets.com/docs/Language_-_Python#Install_Phidget_Python_Module_for_Linux"> Python Phidgets Library</a>


## Package Overview

This is the general overview of the structure of this Library, and the listed files are described.

### Directory Structure
Below is the directory structure of the validation package, including the general function of certain scripts.

    |--Wind_Sensor
        |-- limit_switch                     
            |-- nodes           
                |-- mag_switch.py           #Inits the magnetometers and returns status message       
                |-- stepper_operation                   
                    |-- stepper_main.py     #Main node to initiate the motors
                    |-- stepper_class.py    #Class file containing all the functions that takes mag_switch status as input to driver the motors in x and y    
            |-- launch
                |-- stepper.launch
        |-- CMakeLists.txt
        |-- package.xml
                
                
                

## Usage

1. Git clone the package inside  ` ~/catkin_ws/src/ ` :
    
    ``` 
    git clone https://github.com/arunavanag591/wind_sensor.git
    ```

2. Once you have installed all the dependencies mentioned in [System Requirments](#system-requirements) run `catkin build` from catkin source folder.
3. Copy and paste in your terminal the following

    ``` 
    roslaunch limit_switch stepper.launch 
    ```
4. To view the mag switch status, on a separate terminal

    ``` 
    rostopic echo /mag_switch 
    ```



### **TODO: Implemet Kill Switch / Emergency Switch**
