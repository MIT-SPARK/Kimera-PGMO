# Kimera-PGMO

Kimera-PGMO (Pose Graph and Mesh Optimizer) is an optimizer that takes in a mesh along with robot odometry and loop closure measurements and then optimizes for the trajectory and the mesh simultaneously. This is done by first creating a deformation graph using the pose graph, the mesh, and the time synchronized connections between the pose graph nodes and the mesh vertices. The optimization problem is then optimized using [Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO). 

<img src="images/mesh_and_graph.png" width="900">

<img src="images/pgmo_optimization.png" width="900">

## Dependencies 
In addition to PCL, OpenCV, and GTSAM, Kimera-PGMO is designed as part of Kimera, so the following Kimera packages and their dependencies are needed:

[Kimera-VIO](https://github.mit.edu/SPARK/Kimera-VIO) branch: feature/kimera_pgmo

[Kimera-VIO-ROS](https://github.mit.edu/SPARK/Kimera-VIO-ROS) branch: feature/kimera_pgmo

[Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO) branch: feature/kimera_pgmo

[pose_graph_tools](https://github.mit.edu/SPARK/pose_graph_tools) branch: feature/kimera_pgmo

[Kimera-Semantics](https://github.mit.edu/SPARK/Kimera-Semantics) branch: master

For the immediate dependencies, check out the rosinstall files. 

```bash
cd ~/catkin_ws/src
wstool merge Kimera-PGMO/install/kimera_pgmo_ssh.rosinstall
wstool update
catkin build kimera_pgmo
```
Note: doing a full `catkin build` might cause problems with some of the dependencies of Kimera-Semantics, so it is recommended to build the modules one by one.

## Architecture 
The following is the diagram of modules in Kimera-PGMO.
![Basic system setup in the single robot case](images/PGMO_modules.png)

## Parameters 

#### Mesh Frontend
- `horizon` is the compressor horizon. We are currently using an geometric octree based mesh simplification technique, vertices that is outside the horizon will no longer be in the octree allowing a new vertex to be places near it. 
- `output_mesh_resolution` resolution of the full mesh. Set to less than the Kimera-Semantics / Voxblox voxel size if you don't want any simplification on the full mesh. 
- `robot_id` can be just set to the default `0` if running single robot. This is really only important for the multirobot case. 
- `d_graph_resolution` resolution of the simplified mesh to be added to the deformation graph. 
- `log_path` path to the folder to save the mesh-frontend log file. 
- `log_output` toggle to log timing and statistics. 

#### Kimera PGMO
- `output_prefix` path to the folder to save the log file and the optimized mesh and trajectory files. 
- `robot_id` can be just set to the default `0` if running single robot. This is really only important for the distributed multirobot case. 
- `run_mode` toggles the different modes.Set to 0 to receive pose graph and mesh and perform simultaneous pose graph and mesh optimization. Set to 1 to optimize the mesh and subscribe to an optimized trajectory. Set to 2 to use with DPGMO and just update mesh based on optimized values. 
- `use_msg_time` toggle to use message header stamp or ROS time when connecting pose graph nodes to mesh vertices. 
- `log_output` log timing statistics. 
- `rpgo/translation_threshold` PCM translation threshold. 
- `rpgo/rotation_threshold` PCM rotation threshold. 

## Running Kimera-PGMO

### Single robot Kimera

#### Tesse UHumans2 dataset 
In one terminal, launch Kimera-VIO-ROS with stereo dense:
```bash
roslaunch kimera_vio_ros kimera_vio_ros_uhumans2.launch
```
Then launch kimera_pgmo:
```bash
rosunch kimera_pgmo kimera_pgmo.launch dataset:=uHumans2
```
Launch Kimera-Semantics:
```basg
roslaunch kimera_semantics_ros kimera_semantics_uHumans2.launch
```
For visualization, an rviz configuration is provided: 
```bash
rviz -d $(rospack find kimera_pgmo)/rviz/kimera_pgmo.rviz
```
Finally play the rosbag. 
```bash
rosbag play some_bag.bag --clock
```

To save the mesh, do 
```bash
rosservice call /kimera_pgmo_node/save_mesh
```

and to save optimized trajectory, do 
```bash
rosservice call /kimera_pgmo_node/save_trajectory
```
the mesh will be saved to ouput_folder/mesh_pgmo.ply and trajectory will be saved to output_folder/traj_pgmo.csv (see launch file)

#### Euroc dataset 
Similar to above: 
```bash
roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch use_lcd:=true run_stereo_dense:=true
rosbag play V1_02_medium.bag --clock --pause
roslaunch kimera_pgmo kimera_pgmo.launch dataset:=Euroc
roslaunch kimera_semantics_ros kimera_semantics_euroc.launch
rviz -d $(rospack find kimera_pgmo)/rviz/kimera_pgmo.rviz
```

## Developer notes 

### Running the Unit-tests: 
```bash
roscd kimera_pgmo
catkin run_tests --no-deps --this
catkin_test_results ~/catkin_ws/build/kimera_pgmo/
```
You can also run individual tests for example: 
```bash
rostest kimera_pgmo test_mesh_frontend.test --text
```
Or 
```bash
rosrun kimera_pgmo kimera_pgmo-test_deformation_graph
```

### Plotting the timing 
We provided some simple python scripts to plot the timing
```bash
python scripts/evaluate_pgmo_timing.py /home/yunchang/catkin_ws/src/kimera_pgmo/log/kimera_pgmo_log.csv
python scripts/evaluate_frontend_timing.py /home/yunchang/catkin_ws/src/kimera_pgmo/log/mesh_frontend_log.csv
```

### Misc Note
One thing to note if a developer is working with GTSAM and want to add other factors into the system is that here we specify different prefixes for different types of nodes in the deformation graph, take a look at `utils/CommonFunctions.h` for reference. By prefix we mean the key character as described [here](https://borg.cc.gatech.edu/sites/edu.borg/html/a00244.html). 