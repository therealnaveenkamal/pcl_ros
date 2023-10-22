# 3D Scene Reconstruction with LIDAR Data

This repository contains code for a 3D scene reconstruction using a LIDAR sensor in a 6-legged robot environment. The program captures the scene using a Point Cloud Library (PCL) and processes the data to find edges and reconstruct the scene. The resulting output is a 3D representation of the environment.

## Prerequisites

Before running the code, ensure you have the following installed:

- ROS (Robot Operating System)

- PCL (Point Cloud Library)

## Usage

1\. Clone the repository to your ROS workspace:

```
git clone https://github.com/therealnaveenkamal/pcl_ros.git
```

2\. Build the code:

```
catkin_make
```

3\. Run the ROS node:

```
rosrun pcl_ros main_pcl2image_node
```

## Output

The code processes the LIDAR data and publishes the resulting 3D scene reconstruction as an image. You can view a sample output image and a video to visualize the reconstructed scene:

https://github.com/therealnaveenkamal/pcl_ros/assets/80611084/42b0171d-9b2e-4b5c-92f6-7baffae88431

![Edge Detection in PCL](https://github.com/therealnaveenkamal/pcl_ros/assets/80611084/8a10b716-7f1f-4f42-8e1a-243f4976ca2a)


## About the Code

### MagicSubscriber Class

The `MagicSubscriber` class in the provided code subscribes to a ROS topic that contains LIDAR data and performs the following steps to create a 3D reconstruction:

1\. Converts LIDAR data to a PointCloud2 message.

2\. Extracts information about the camera's intrinsic parameters (focal length, center coordinates, etc.).

3\. Maps the 3D points from the LIDAR data to the 2D image plane.

4\. Generates a depth map of the scene.

5\. Converts the depth map to a 16-bit image and publishes it.

The output image is used to visualize the 3D scene reconstruction.

## License

This code was produced under the supervision of [The Construct Team](https://www.theconstructsim.com/). 

## Author

- Naveenraj Kamalakannan

- Contact: therealnaveenkamal@gmail.com
