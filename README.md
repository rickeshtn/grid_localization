
![Screenshot](/Screenshot.png)

Demo video is [here](https://www.youtube.com/watch?v=GNgIDupisc4&t=).


[MRPT](https://www.mrpt.org/) is required to build the project.

See [how to get MRPT-1.5 on Ubuntu](https://www.mrpt.org/MRPT_in_GNU/Linux_repositories).


#About grid_localization
The project contains three functions:

1.Point cloud generation.

Generate a global point cloud with the trajectory formed form EKF using odometer, imu, and rtk-gps.

2.Elevation grid map generation.

Generate the elevation grid map with the global point cloud. Each grid map covers a 80*80m^2 area and is generated every 20m to ensure there is enough shared area between two grid maps (to ensure the vehicle is located in the central area of each grid map, better for localization).

3.Localization against elevation grid map.


