# KVRC 2022: KHNP Virtual Robot Challenge 2022
## Hosted by Korea Hydro & Nuclear Power
+ Quadrotor UAV is used for this year

<br>


## Homepage - [click](http://kvrc2022.com/)
## Promotion Video [click](https://youtu.be/usKW3UG9wpc)
<!-- ## NEWS article [click] -->


 <br> 

<p align="left"> 
<img src="main_code/resources/poster.jpg" width="400"/> 
</p> 


<br>


## ● Requirements & Installation
+ Mainly tested on `ROS Melodic` version
+ Make sure that you installed `ROS desktop full` version - refer the [wiki page](https://wiki.ros.org/ROS/Installation)
  + It comes with `Qt5`, `Gazebo`, `OpenCV` version 3.2, `cv_bridge`
  + In other words, this repo depends on `QT5`, `Gazebo`, `OpenCV`, `cv_bridge`.
+ Make sure that you installed `PX4-SITL` and `mavros`
  + **Note, we are using commit `96c7fe4978bab2af970a097f4898e024c2d33440` instead of the latest version**
  + **Note, `Lockstep` should be disabled by changing the `boards/px4/sitl/default.cmake` file**
  
    <details><summary>[Unfold to see how to install PX4-SITL properly]</summary>

    ~~~shell
    $ sudo apt-get install ros-<distro>-mavros ros-<distro>-mavros-extras
    $ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    $ chmod +x install_geographiclib_datasets.sh
    $ sudo ./install_geographiclib_datasets.sh

    $ cd ~/ && git clone https://github.com/PX4/PX4-Autopilot.git
    $ cd PX4-Autopilot
    $ git reset --hard 96c7fe4978bab2af970a097f4898e024c2d33440
    $ git submodule update --init --recursive

    $ source PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
    ~~~

    + **Note, `Lockstep` should be disabled by changing the `boards/px4/sitl/default.cmake` file as follows:**

    ```shell
    Open the file
    $ cd ~/PX4-Autopilot

    $ gedit boards/px4/sitl/default.cmake
    ```

    + Edit the last line's `ENALBE_LOCKSTEP_SCHEDULER` as `no`

    ~~~python
    if(REPLAY_FILE)
      message(STATUS "Building with uorb publisher rules support")
      add_definitions(-DORB_USE_PUBLISHER_RULES)

      message(STATUS "Building without lockstep for replay")
      set(ENABLE_LOCKSTEP_SCHEDULER no)
    else()
      #set(ENABLE_LOCKSTEP_SCHEDULER yes)
      set(ENABLE_LOCKSTEP_SCHEDULER no)
    endif()
    ~~~

    + Then build `PX4-SITL`

    ~~~shell
    $ cd ~/PX4-Autopilot

    # important!!
    $ sudo apt install ros-<distro>-gazebo-plugins

    $ export LANG=C.UTF-8
    $ export LC_ALL=C.UTF-8
    $ DONT_RUN=1 make px4_sitl_default gazebo
          When undefined iginition error occurs: (gazebo: symbol lookup error: /usr/lib/x86_64-linux-gnu/libgazebo_common.so.9: undefined symbol: ZN8ignition10fuel_tools12ClientConfi..........
          $ sudo apt upgrade libignition-math4
    ~~~


    </details>
  
+ Make sure that your `Cmake` version is more recent than 3.19
  + Check it with `$ cmake --version`
    <details><summary>[Unfold to see how to upgrade Cmake]</summary>
    
      ~~~shell
      $ wget https://github.com/Kitware/CMake/releases/download/v3.19.8/cmake-3.19.8.tar.gz
      $ tar zxf cmake-3.19.8.tar.gz && cd cmake-3.19.8
      $ ./bootstrap
      $ make
      $ sudo make install

      $ sudo reboot
      $ cmake --version 
      ~~~
      
    </details>
+ Make sure that your `Gazebo` version is at least `9.19.0`
  <details><summary>[Unfold to see how to upgrade Gazebo]</summary>
  
    + Updating `Gazebo` - [reference link](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install#Alternativeinstallation:step-by-step)
    ~~~shell
    $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    $ cat /etc/apt/sources.list.d/gazebo-stable.list
    $ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    $ sudo apt-get update
    $ sudo apt-get upgrade
    ~~~
    
  </details>

+ Finally, install this git repo
  <details><summary>[Unfold to see how to build this repo]</summary>
  
    ```shell
    $ cd <your_workspace>
    $ catkin build
    $ . devel/setup.bash

    ## It is better to save sourcing your workspace within .bashrc
    $ echo "source <your_workspace>/devel/setup.bash" >> ~/.bashrc

    $ cd <your_workspace>/src
    $ git clone --recursive https://github.com/Woojin-Seol/KVRC2022

    !!!Add Gazebo and ROS Path **ONLY ONCE, do NOT run below block again!!!!**
    $ cd KVRC2022/gazebo_map_for_khnp
    $ echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/season2:/home/$(whoami)/PX4-Autopilot/Tools/sitl_gazebo/models" >> ~/.bashrc
    $ echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/$(whoami)/PX4-Autopilot/build/px4_sitl_default/build_gazebo" >> ~/.bashrc
    $ echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$(whoami)/PX4-Autopilot/build/px4_sitl_default/build_gazebo" >> ~/.bashrc
    $ echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$(whoami)/PX4-Autopilot:/home/$(whoami)/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc

    $ . ~/.bashrc

    $ cd <your_workspace>/src
    $ catkin build
    $ . ~/.bashrc
    ```
    
  </details>

+ Your `~/.bashrc` should be look like
  ```shell
  ....

  source ~/khnp_ws/devel/setup.bash # order is important, workspace's setup.bash should be ahead of ROS_PACKAGE_PATH

  export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/mason/PX4-Autopilot/build/px4_sitl_default/build_gazebo
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/mason/khnp_ws/src/KVRC2022/gazebo_map_for_khnp/season2:/home/mason/PX4-Autopilot/Tools/sitl_gazebo/models
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/mason/PX4-Autopilot/build/px4_sitl_default/build_gazebo
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/mason/PX4-Autopilot:/home/mason/PX4-Autopilot/Tools/sitl_gazebo # order is important, workspace's setup.bash should be ahead of ROS_PACKAGE_PATH

  ....
  ```
---

<br>

## ● How to use
+ Run launch file
  ~~~shell
  $ roslaunch khnp_competition gazebo.launch
  $ roslaunch khnp_competition main.launch

  For real competition (maintainer only)
  $ roslaunch khnp_competition gazebo.launch course:=real
  $ roslaunch khnp_competition main.launch course:=real
  ~~~

<br>


## ● What you can change / what you have to do 
+ You can edit codes related to control the UAV. Currently, temporal example controller is included in `main.launch` file.
+ Add your autonomous navigation algorithm code and controller in `main.launch`.
  + Open `main.launch` to edit
    ```xml
    <?xml version="1.0"?>
    <launch>

    <!-- Edit this part with your own algorithms -->



    <!-- Do not touch below!!!!!!!! -->
      ............

    </launch>
    ```
