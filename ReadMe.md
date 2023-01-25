### Reposity for RC Car Trajectory Tracking demonstration

This project is extended from `rccar-demo` and is an attempt to implement Model Predictive 
Control (MPC) for tracking a given trajectory on the RC car.

Below are the instructions to run trajectory tracking using MPC

VICON calibration
------------------
Ensure to calibrate all the VICON cameras and then keep the RC car in the
VICON playground with all markers intact. On the VICON side computer, use the
object name as ```rccar_car_01``` and select "Track".


SSH
---
Use the below command to SSH into the RC car.
Note: You may experience trouble connecting, if the IP address of the RC car
has changed, in that case connect some peripherals (monitor, keyboard, mouse
etc.) to the RC car and find the IP address. Use this IP in the below command.

```
ssh <host-name>@<host-address>
```

```ssh ubuntu@130.215.219.237```


Run Serial Node on RC car (low-level control)
---------------------------------------------
SSH into RC car and run serial node

```roslaunch acc_ros rccar_serial.launch```


Trajectory Tracking using MPC
---------------------------------------------
Now, run the below commands on local computer

1. ```setrccaruri```: `scripts/setrccaruri` Use this alias to change the ROS_MASTER_URI to RC car's IP

2. ```roslaunch rccar_track trajectory_tracking.launch```

3. When the RViz window pops up, mark a desired goal with a desired orientation.

You will notice a trajectory planned from the current state to the goal. The car
will now attempt to track this trajectory. For best results, keep
- time step to 1 in `PoseCallBack.py`, `trajGen.py` and `trajTrac.py`
- horizon N = 2 in `trajTrac.py`

For any questions, you may contact me at mzaman@wpi.edu or Mitesh at
msagrawal@wpi.edu
