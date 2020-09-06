# Kimera-PGMO

Kimera-PGMO (Pose Graph and Mesh Optimizer) optimizes the mesh and the trajectory based on detected loop closures. The deformation of the mesh induced by the loop closure is based on deformation graphs, which is generated by simplifying the mesh and embedding the trajectory pose graph. 

For example, given a mesh and a pose graph trajectory
<img src="images/mesh-and-posegraph.jpg" width="500">

We generate the embedded deformation graph and optimize simultaneously the mesh and the pose graph when a loop closure is obtained. 
<img src="images/deformation-graph.jpg" width="500">

## Dependencies 
In addition to PCL, OpenCV, GTSAM, Kimera-PGMO is designed as part of Kimera, so the following Kimera packages and their dependencies are needed:

[Kimera-VIO](https://github.mit.edu/SPARK/Kimera-VIO/tree/feature/mesh_deformation) branch: feature/mesh_deformation

[Kimera-VIO-ROS](https://github.mit.edu/SPARK/Kimera-VIO-ROS/tree/feature/deformable_mesh) branch: feature/deformable_mesh

[Kimera-Semantics](https://github.mit.edu/SPARK/Kimera-Semantics)

For the immediate dependencies, check out the rosinstall files. 

## Architecture 
For the common usage with a single robot, the following is the diagram of the architecture.
![Basic system setup in the single robot case](images/diagram_pgmo.png)

We also provide a merger node for possible centralized multi-robot usage. 
![Basic system setup in the single robot case](images/diagram_merger.png)

## Parameters 
Configure the parameters in the params folder for your dataset and environment. 
- `d_graph_resolution` sets how sparse to sample the mesh to obtain the vertices of the deformation graph
- `compression_time_horizon` sets a time horizon to discard older parts of the mesh to avoid associating a new node with an old part of the mesh 
- `embed_trajectory_delta_t` the slack to set when synchronizing incoming incremental mesh with incoming pose graph 
- `rpgo` the outlier rejection thresholds 

## Running Kimera-PGMO

### Single robot Kimera

#### Tesse UHumans2 dataset 
In one terminal, launch Kimera-VIO-ROS with stereo dense:
```bash
roslaunch kimera_vio_ros kimera_vio_ros_uhumans2.launch
```
Then launch kimera_pgmo:
```bash
rosunch kimera_pgmo kimera_pgmo.launch launch_voxblox:=false dataset:=uhumans
```
Launch Kimera-Semantics:
```basg
roslaunch kimera_semantics_ros kimera_semantics.launch
```
For visualization, an rviz configuration is provided: 
```bash
rviz -d $(rospack find kimera_pgmo)/rviz/kimera_pgmo.rviz
```
Finally play the rosbag 
```bash
rosbag play some_bag.bag --clock
```

## Developer notes 
One thing to note if a developer is working with GTSAM and want to add other factors into the system is that here we assumed that all nodes in the factor graph corresponding to a mesh vertex is marked with a prefix 'v'. By prefix we mean the key character as described [here](https://borg.cc.gatech.edu/sites/edu.borg/html/a00244.html). 