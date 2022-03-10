# ROS driver for Nortek DVL 1Mhz and pressure 300m (still in progress, basic is working)
This version tested in ubuntu 18.04

## Licence: MIT License

## Installation
- Install dependencies
    - Install ros serial, please check [here](http://wiki.ros.org/serial) for more information. For ubuntu 18.04/ROS melodic, just using command below  
        ```
        $ sudo apt-get install ros-melodic-serial 

        ```
- Install DVL driver
    ```
    $ cd ~/your_path_catkin_ws/src
    $ git clone https://github.com/URIsoslab/nortek_dvl.git
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
    - select **PNORBT7** for Botton Track
    - select **PNORI1/PNORS1/PNORC1** for Current Profile

- Launch
    ```shell
    $ roslaunch nortek_dvl star_dvl.launch
    ```

- Change usb port: 
    - go to /nortek_dvl/launch/star_dvl.launch
    - change "port_name" as you find the DVL

## TODO:

- change bottom track to **PNORBT9** so that it includes pressure
- apply ros cfg
- close thread appropriately  
