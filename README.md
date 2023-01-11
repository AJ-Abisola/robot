# This is a work on autonomous grape counting in a vineyard using the thorvald robot.

The work in this repo works in the simulation world called vineyard_small_s4_coarse, it can work in other world with some tweaks to the topological map.

#TO RUN
- start simulation world
- include config file topo_nav.rviz in rviz
- run roslaunch aj topo_nav.launch
- run rosrun aj grapecounter.py or navigate to folder /aj/scripts and run python grapecounter.py
