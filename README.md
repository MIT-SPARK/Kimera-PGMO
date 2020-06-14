# Mesher-Mapper

## Release News

## Running Mesher-Mapper 

### Running With Kimera 
In one terminal, launch Kimera-VIO-ROS with stereo dense:
```bash
roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch use_lcd:=true run_stereo_dense:=true
```

The launch mesher_mapper:
```bash
roslaunch mesher_mapper mesher_mapper.launch
```

For visualization, an rviz configuration is provided: 
```bash
rviz -d $(rospack find mesher_mapper)/rviz/mesher_mapper.rviz
```