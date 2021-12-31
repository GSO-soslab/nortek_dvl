# ROS driver for Nortek DVL 1Mhz and pressure 300m (still in progress, basic is working)
This version tested in ubuntu 18.04

## Licence: MIT License

## Installation
- Install dependencies
    - Install ros serial, please check [here](http://wjwwood.io/serial/doc/1.1.0/index.html) for more information. For ubuntu 18.04/ROS melodic, just using command below  
        ```
        $ sudo apt-get install ros-melodic-serial 

        ```
- Install DVL driver
    ```
    $ cd ~/your_path_catkin_ws/src
    $ git clone https://github.com/GSO-soslab/nortek_dvl.git
    ```

## Build
```
$ cd ~/your_path_catkin_ws/
$ catkin_make --pkg nortek_dvl
```

## Usage
- Fix USB permission: 
    ```shell
    $ ls -l /dev/ttyUSB* # check your permissions
    $ id -Gn {usrname} # replace {usrname} with your usrname, check if "dialout" is there
    $ sudo usermod -a G dialout {usrname} # add dialout in your username
    ```
- Setup
    - select **PNORBT7** for Bottom Track
    - select **PNORI1/PNORS1/PNORC1** for Current Profile

- Launch
    ```shell
    $ roslaunch nortek_dvl star_dvl.launch
    ```

- Change usb port: 
    - go to /nortek_dvl/launch/star_dvl.launch
    - change "port_name" as you find the DVL

## Current setup:

- if you enabled current profile with 2 and bottom track with the max 8hz. you will receive bottom track at 4hz and current profile at 2 hz in ROS.
- if you only enable bottom track with max 8hz, you will receive 8hz in ROS.


## Todos:

- send serial back, setup DVL property: change hz, enable currect profile......
- apply ros cfg
- close thread appropriately  
