#VICON playground
Keep the robot in a corner of the VICON playground and echo the topic /vicon/Pose to get the coordinates of the corner in the world frame. Repeat this with the other corners and the origin. This data is used to define the state space constraints.
Recorded translation data (X,Y,Z):
Origin: 0.07,0.00,0.14
SouthWest: 2.21,2.98,0.18
SouthEast: -1.718,-2.8,0.14
NorthWest: 1.87,-2.54,0.23
NorthEast: -1.76,-2.65,0.23

-1.7 <= x <= 2
-2.5 <= y <= 2.8
-1 <= theta_radians <= 1
-55 <= theta_degrees <= 55
