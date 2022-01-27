# ROS package for point cloud preprocessing

This package performs preprocessing of point clouds.
It is based on kinect2_bridge which needs to be installed before, e.g. from https://github.com/mrudorfer/iai_kinect2_opencv4.
It has been used with ROS noetic / Ubuntu 20.04.

There are some python dependencies (e.g. ros_numpy) which might not be properly included in the package/CMakeLists files.
Please raise an issue to fix this when you come across.

### Launch files

`roslaunch pc_preprocessing perception.launch` brings up the kinect2 and performs:
- transforms pc into world frame (requires world-->kinect2)
- crops point cloud to a box w/ relevant extents
- [optional] performs voxel-based downsampling
- identifies a plane
- removes the points on the plane from the point cloud
- removes statistical outliers

See launch file for parametrisation.

### Nodes

`rosrun pc_preprocessing pc_sub.py _filename:=/save_path/file.npy`
brings up a subscriber node that converts the point cloud to numpy and outputs their shape.
When pressing Enter, the latest point cloud is saved to the given filename.

Per default, this node listens to the output of the statistical outlier removal
(the launch file's final output).