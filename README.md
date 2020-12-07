# Autonomous Systems - Mapping with Depth Camera
The objective of the project was to develop a program that, using an Occupancy Grid mapping algorithm, gives us a map of a static space, given the P3-DX Pioneer Robot’s localization and the data from an Xbox Kinect depth camera. All the project was developed using ROS environment and packages. To achieve the major goal, it was used the Bresenham’s Line Algorithm to implement our mapping algorithm as well as the Adaptive Monte Carlo Localization for the robot’s localization. To evaluate our performance, it was used error metrics such as the binary difference of a reference map that was obtained using a ROS package (gmapping) and the one that our algorithm gives us. Another thing that should be mentioned is the fact that while acquiring the map, our reference frame is static, so when the algorithm is running, the map runs out this reference frame if we do not ensure a sufficiently high map dimensions. One solution for that is the use of a dynamic reference frame that is built along with the map.


# Authors
- Carolina Costa carolina.c.p.costa@tecnico.ulisboa.pt
- Francisco Melo francisco.raposo.melo@tecnico.ulisboa.pt
- Raúl Vaqueiro raulletrasv@tecnico.ulisboa.pt
- Rodrigo Rego rodrigorego@tecnico.ulisboa.pt
