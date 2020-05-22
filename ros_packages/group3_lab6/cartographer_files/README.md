## HOW TO CONFIGURE AND RUN THE GOOGLE CARTOGRAPHER

For the installation follow the instructions here:
<https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html>

### After the installation is complete do the following steps:

 1.  Source the **setup.bash** file of the cartographers workspace 
 
    `source ~/catkin_ws/install_isolated/setup.bash`

 1.  Copy the folder "f110_description" to the folder "~/catkin_ws/install_isolated/share" 
 
    `cp -r f110_description ~/catkin_ws/install_isolated/share`
 
 1.  Copy the launch-files to "~/catkin_ws/install_isolated/share/cartographer_ros/launch" and adjust the paths inside of them for your system.
 
     The launch-files are using bag-files as input for the cartographer, 
     so you need to provide a bagfile with the suiting topics **/scan** and **/tf** (without transformations from the frame "base_link" to "map")
 
    `cp  *.launch ~/catkin_ws/install_isolated/share/cartographer_ros/launch`

 1.  Copy the f110_2d.lua file to the folder configured in the launch-files, for example

    `cp f110_2d.lua ~/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files`

 1. With the following command you can check if your bagfile is suitable for the cartographer
    
    `cartographer_rosbag_validate -bag_filename your_bag.bag`

 1.  Then simply call roslaunch and provide the path to the bagfile like

    `roslaunch cartographer_ros f110_2d.launch bag_filename:=your_bag.bag`

 1. Now we can save the maps published by the cartographer at the topic **/map** with

    `rosrun map_server map_saver  -f map_name`

    And then convert it to different colors to see the details better

    `convert map_name.pgm -fuzz 34% -fill black -opaque gray converted_map_name.pgm`
