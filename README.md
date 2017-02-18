# icub-moveit

URDF iCub model ready for MoveIt! in ROS Hydro

## Environment

All the functionality was tested on machines configured with:

+ Ubuntu 14.04 LTS
+ ROS Hydro (install it [here](http://wiki.ros.org/hydro/Installation/Ubuntu))

## Download and Setup

Open a terminal and navigate to the folder where you want to download the package.

Now, run:

    git clone https://github.com/vislab-tecnico-lisboa/icub-moveit.git

After completing the download, you need to add the new folder to the $ROS_PACKAGE_PATH, run:

    sudo gedit ~/.bashrc

At the end of the file, add the following line:

    export ROS_PACKAGE_PATH=[Path to the downloaded folder]:$ROS_PACKAGE_PATH
    
Save the file and run:

    source ~/.bashrc

## Compilation



## Running


This repo was inspired from the work of [vislab-tecnico-lisboa](https://github.com/vislab-tecnico-lisboa/icub-moveit.git)
