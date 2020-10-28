# uav_frontier_exploration_3d

# A Multi-Resolution Frontier-Based Planner for Autonomous 3D Exploration

Proposed multi-resolution frontier-based planner is a real-time capable exploration planner.
The planner consists of the algorithm for detecting frontier points, followed by clustering of frontier points and selecting the best
frontier point to be explored. Performance is achieved by not relying on data obtained directly 
from the 3D sensor, but on data obtained by a mapping algorithm. In order to cluster the frontier points,
the properties of the Octree environment representation are used, which allows easy analysis with
different resolutions. 

This README gives a short overview. For more information refer to the [wiki](https://github.com/larics/uav_frontier_exploration_3d/wiki). 

Video recordings of frontier-based exploration for both simulation and experimental scenarios can be found at [YouTube](https://www.youtube.com/playlist?list=PLC0C6uwoEQ8a88D6cKfa81Hfo_s_qVZxf).

## Exploration planner installation and execution - quick guide

In order to run the current version of 3D exploration planner, you should compile the package uav_frontier_exploration_3d. 
First of all, navigate to the source folder of your ros workspace:

```sh
git clone https://github.com/larics/uav_frontier_exploration_3d.git
catkin build uav_frontier_exploration_3d
```

Start a simulation after compilation:

```sh
roslaunch uav_frontier_exploration_3d frontier_server.launch
```
In order to start autonomous UAV exploration, call service:

```sh
rosservice call /red/exploration/toggle "data: true"
```
Tested under ROS Melodic.

Further instructions for the installation, visualization of the exploration progress,
as well as more demo scenarios and descriptions of the parameters can be found in the [wiki](https://github.com/larics/uav_frontier_exploration_3d/wiki).

## Credits

This algorithm was developed by [Ana Batinovic](mailto:ana.batinovic@fer.hr) 
with the help and support of the members of the [LARICS](https://larics.fer.hr/).

## Contact

You can contact [Ana Batinovic](mailto:ana.batinovic@fer.hr) for any question or remark. 

