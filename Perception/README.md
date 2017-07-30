# Perception Exercises

In these exercises, we will perform object segmentation on 3D point cloud data using `python-pcl` to leverage the power of the [Point Cloud Library](http://pointclouds.org/).  

* Exercise 1 - practice performing filtering and RANSAC plane segmentation
* Exercise 2 - write a ROS node to perform those functions as well as euclidean clustering for object segmentation
* Exercise 3 - continue building up the perception pipeline in ROS


## Documentation for `pcl_helper.py`

`pcl_helper.py` in `Exercise-2/sensor_stick/scripts/` contains useful functions for working with point cloud data with ROS and PCL. Here's a brief description of the contents:
 
`random_color_gen()` 
```
Generates a random set of r,g,b values
Return: a 3-tuple with r,g,b values (range 0-255)
```

`ros_to_pcl(sensor_msgs/PointCloud2)` 
```
Converts sensor_msgs/PointCloud2 to XYZRGB Point Cloud
Return: pcl.PointCloud_PointXYZRGB
```

`pcl_to_ros(pcl.PointCloud_PointXYZRGB)` 
```
Converts XYZRGB Point Cloud to sensor_msgs/PointCloud2
Return: sensor_msgs/PointCloud2
```

`XYZRGB_to_XYZ(XYZRGB_cloud)` 
```
Converts XYZRGB Point Cloud to XYZ Point CLoud
Return: pcl.PointCloud
```

`XYZ_to_XYZRGB(XYZ_cloud, color)` 
```
Takes a 3-tuple as color and adds it to XYZ Point Cloud
Return: pcl.PointCloud_PointXYZRGB
```

`rgb_to_float(color)`
```
Converts 3-tuple color to a single float32
Return: rgb packed as a single float32
```

`get_color_list(cluster_count)` 
```
Creates a list of 3-tuple (rgb) with length of the list = cluster_count
Return: get_color_list.color_list
```
