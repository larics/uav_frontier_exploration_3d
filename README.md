# uav_frontier_exploration_3d

# A Multi-Resolution Frontier-Based Planner for Autonomous 3D Exploration

This repository contains the code for frontier-based exploration, presented in the paper:

**A Multi-Resolution Frontier-Based Planner for Autonomous 3D Exploration**\
Ana Batinovic, Tamara Petrovic, Antun Ivanovic, Frano Petric and Stjepan Bogdan (IEEE Robotics and Automation Letters 2021)\
If you use this code in a scholarly work, please cite our [journal paper](https://ieeexplore.ieee.org/document/9387089):

```
@ARTICLE{Batinovic-RAL-2021,
  author={Batinovic, Ana and Petrovic, Tamara and Ivanovic, Antun and Petric, Frano and Bogdan, Stjepan},
  journal={IEEE Robotics and Automation Letters}, 
  title={A Multi-Resolution Frontier-Based Planner for Autonomous 3D Exploration}, 
  year={2021},
  volume={6},
  number={3},
  pages={4528-4535},
  doi={10.1109/LRA.2021.3068923}}
```


Proposed multi-resolution frontier-based planner is a real-time capable exploration planner.
The planner consists of the algorithm for detecting frontier points, followed by clustering of frontier points and selecting the best
frontier point to be explored. Performance is achieved by not relying on data obtained directly 
from the 3D sensor, but on data obtained by a mapping algorithm. In order to cluster the frontier points,
the properties of the Octree environment representation are used, which allows easy analysis with
different resolutions. 

Video recordings of frontier-based exploration for both simulation and experimental scenarios can be found at [YouTube](https://www.youtube.com/playlist?list=PLC0C6uwoEQ8a88D6cKfa81Hfo_s_qVZxf).

This README gives a brief overview. For more information, please refer to the [wiki](https://github.com/larics/uav_frontier_exploration_3d/wiki), where all further instructions on installation, visualization of exploration progress, as well as demo scenarios and descriptions of parameters can be found.

## Credits

The exploration algorithm was developed by [Ana Batinovic](mailto:ana.batinovic@fer.hr) 
with the help and support of the members of the [LARICS](https://larics.fer.hr/).

## Contact

You can contact [Ana Batinovic](mailto:ana.batinovic@fer.hr) for any question or remark. 

