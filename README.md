#grid_localization started in 2017-09-25
#Elevation grid map based localization using LiDAR in urban environment.
#Using MRPT for programming.

MRPT is required to build the project.

video
https://www.youtube.com/watch?v=GNgIDupisc4&t=

#About grid_localization
The project contains three functions:
1.Point cloud generation.
Generate a global point cloud with the trajectory formed form EKF using odometer, imu, and rtk-gps.
2.Elevation grid map generation.
Generate the elevation grid map with the global point cloud. Each grid map covers a 80*80m^2 area and is generated every 20m to ensure there is enough shared area between two grid maps (to ensure the vehicle is located in the central area of each grid map, better for localization).
3.Localization against elevation grid map.
