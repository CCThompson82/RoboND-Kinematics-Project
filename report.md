# Project: Kinematics Pick & Place

## Summary
The objective of this project was to develop a simple inverse kinematic solver
for the KUKA210 robot, for the purpose of an object pick and place task.  To
accomplish this objective, Denavit-Hartenberg (DH) parameters were defined using
the KUKA210 urdf file, and then used to generate homogeneous transformation
matrices for each of the 6 joints of the robot.  Individual transformation
matrices were composed to generate a total transformation matrix capable of
providing forward kinematic solutions.  Finally, a trigonometric solution was
generated in order to solve for a joint state, given an end-effect target pose.  
This inverse kinematics solution was used within a ROS service required for
completion of the pick and place task.    

This report outlines the process by which each of these steps were completed,
and the results of the pick and place task.     

[//]: # (Image References)

[zero]: ./misc_images/ZeroConfig_noAxes.jpg
[axes]: ./misc_images/ZeroConfig_v2.jpg
[J0J1]: ./misc_images/J0J1.jpg
[J1J2]: ./misc_images/J1J2.jpg
[J2J3]: ./misc_images/J2J3.jpg
[J3J6]: ./misc_images/J3J6.jpg
[J6JG]: ./misc_images/J6Gripper.jpg


## Kinematic Analysis

### Definition of Denavit-Hartenberg Parameters

#### Represent the KUKA210 robot in a zero joint angle configuration

In order to define DH parameters, joint axes must first be defined for the zero
angle configuration of the robot.  **Figure 1** shows an illustration of the
robot projected on the global x, z plane (with positive y heading towards the
viewer).

![alt text][zero]

**Figure 1 - Illustration of the zero angle configuration for the KUKA210 robot.
The robot is represented as a projection onto the global x, z plane,  with
the positive y direction leaving the page toward the viewer (The  gripper
orientation is not accurate in this schematic for clarity).   Joints are  shown
in purple, and links between joints are shown in blue.  Each joint and link
are numbered in accordance with DH parameter guidelines.**

Origins for each joint, with their respective positive $\hat{Z}$ and $\hat{X}$
axes must then be defined.  Choice of where to orient each joint is made with
the objective of minimizing the number of non-zero DH parameters.  The direction
for positive $\hat{Z}_{i}$ and $\hat{X}_{i}$ axes must be made such that
$\hat{X}_{i-1}$ must be  perpendicular to both $\hat{Z}_{i-1}$ and $\hat{Z}_{i}$
These definitions can be observed in **Figure 2**.

![alt text][axes]

**Figure 2 - Definition of positive Z and X axes for KUKA210 robot. Placement
of axes origins were made in order to minimize the number of non-zero DH
parameters.  For instance, setting the origin for joints 4, 5, and 6 as
conincident allows reduces the number of non-zero DH parameters for these joints
down to solely their joint rotation (theta values) and twist rotation (alpha
values).**

#### Calculate DH parameters using the KUKA210 urdf

For each joint, the following parameters must be defined:
* twist angle ($\alpha_{i-1}$) - angle (radians) between $\hat{Z}_{i-1}$ and $\hat{Z}_{i}$, about $\hat{X}_{i-1}$ using the right-hand rule
* link length ($a_{i-1}$) - signed distance between $\hat{Z}_{i-1}$ and $\hat{Z}_{i}$ along $\hat{X}_{i-1}$
* link offset ($d_{i}$) - signed distance from $\hat{X}_{i-1}$ to $\hat{X}_{i}$, along $\hat{Z}_{i}$
* joint angle ($\theta_{i}$) - angle between $\hat{X}_{i-1}$ to $\hat{X}_{i}$, about $\hat{Z}_{i}$ using the right-hand rule

##### Joint 1
* $\alpha_{0}$ := 0
    * as $\hat{Z}_{0}$ and $\hat{Z}_{1}$ are coincident and therefore there is no rotation between these axes about $\hat{X}_{0}
* $a_{0}$ := 0
    * as $\hat{Z}_{0}$ and $\hat{Z}_{1}$ are coincident and therefore there is no separation between these axes along $\hat{X}_{0}$
* $d_{1}$ := 0.75
    * as the distance from $\hat{X}_{0}$ to $\hat{X}_{1}$ along $\hat{Z}_{1}$ incorporates the z-lengths for both joint1 (0.33) and joint2 (0.42) in the urdf file.
    * $d_{1}$ is easily observed in the X-Z or Y-Z projection in **Figure 3**
* $\theta_{1}$ := $\theta_{1}$
    * as joint 1 is a revolute joint and $\hat{X}_{0}$ to $\hat{X}_{1}$ are parallel when $\theta_{1}$ is set to zero.  

![alt text][J0J1]  

**Figure 3 - Demonstration of DH parameter calculation for Joint 1.  Joint 1 and
surrounding links are shown as projections into the global X-Z, X-Y, and Y-Z
planes.  Notice that d1 is easily depicted when projected onto the Z axis.  In
the middle panel, in which the Z axis points directly towards the viewer,
$\hat{X}_{0}$ and $\hat{X}_{1}$ are parallel but not conincident (represented by
dashed marcation).**

##### Joint 2
* $\alpha_{1}$ := -$\frac{\pi}{2}$
    * as a negative (right-hand) rotation is required about $\hat{X}_{1}$ in order to transform $\hat{Z}_{1}$ to $\hat{Z}_{2}$
    * $\alpha_{1}$ is easily observed in Y-Z projection in **Figure 4**
* $a_{1}$ := 0.33
    * as the distance between $\hat{Z}_{1}$ and $\hat{Z}_{2}$ along $\hat{X}_{1}$ incorporates the x-frame length for joint2 in the urdf.
    * $a_{0}$ is easily observed in the X-Z or Y-Z projections in **Figure 4**
* $d_{2}$ := 0
    * as $\hat{X}_{1}$ and $\hat{X}_{2}$ both intersect $\hat{Z}_{2}$ at the same location, thus there is no offset distance.  
* $\theta_{2}$ := $\theta_{2}$ - $\frac{\pi}{2}$
    * as a constant negative rotation of $\frac{\pi}{2}$ is required to transform $\hat{X}_{1}$ to $\hat{X}_{2}$ about $\hat{Z}_{2}$

![alt text][J1J2]  

**Figure 4 -  Demonstration of DH parameter calculation for Joint 2.  The a1
distance is easily observed when the global Z-axis is present in the projection.
The twist angle is observable in the Y-Z projection, whereas the theta offset is
most easily observable in the X-Z projection.**

##### Joint 3
* $\alpha_{2}$ := 0
    * as $\hat{Z}_{2}$ and $\hat{Z}_{3}$ are parallel and therefore there is no rotation between these axes about $\hat{X}_{2}
* $a_{2}$ := 1.25
    * as the distance between $\hat{Z}_{2}$ and $\hat{Z}_{3}$ along $\hat{X}_{2}$ incorporates the z-frame length for joint3 in the urdf.
    * $a_{2}$ is easily observed in the X-Z or Y-Z projections in **Figure 5**
* $d_{3}$ := 0
    * as $\hat{X}_{2}$ and $\hat{X}_{3}$ are coincident   
* $\theta_{3}$ := $\theta_{3}$
    * as joint 3 is a revolute joint and $\hat{X}_{2}$ and $\hat{X}_{3}$ are coincident when $\theta_{3}$ is set to zero.   

![alt text][J2J3]  
**Figure 5 - Demonstration of DH parameter calculation for Joint 3.  The a2
parameter is clearly shown in projections X-Z and Y-Z.**

##### Joint 4
* $\alpha_{3}$ := -$\frac{\pi}{2}$
    * as a negative (right-hand) rotation is required about $\hat{X}_{3}$ in order to transform $\hat{Z}_{3}$ to $\hat{Z}_{4}$
    * $\alpha_{3}$ is easily observed in X-Y projection in **Figure 6**
* $a_{3}$ := -0.054
    * as the distance between $\hat{Z}_{3}$ and $\hat{Z}_{4}$ (which is coincident with both $\hat{Z}_{5}$ and $\hat{Z}_{6}$) along $\hat{X}_{3}$ incorporates the z-frame length for joint4 in the urdf.
* $d_{4}$ := 1.5
    * as the distance from $\hat{X}_{3}$ to $\hat{X}_{4}$ along $\hat{Z}_{4}$ incorporates the x-frame lengths for joint 4 (0.96 m) and joint 5 (0.54 m).
    * $d_{4}$ is easily observed in the X-Z projection in **Figure 6**
* $\theta_{4}$ := $\theta_{4}$
    * as joint 4 is a revolute joint and $\hat{X}_{3}$ and $\hat{X}_{4}$ are parallel when $\theta_{4}$ is set to zero.  

##### Joint 5
* $\alpha_{4}$ := $\frac{\pi}{2}$
    * as a positive (right-hand) rotation is required about $\hat{X}_{4}$ in order to transform $\hat{Z}_{4}$ to $\hat{Z}_{5}$
    * $\alpha_{4}$ is easily observed in X-Y projection in **Figure 6**
* $a_{4}$ := 0
    * as $\hat{Z}_{4}$ and $\hat{Z}_{5}$ both intersect $\hat{X}_{4}$ at the same position, thus there is no separation between these axes
* $d_{5}$ := 0
    * as $\hat{X}_{4}$ and $\hat{X}_{5}$ are coincident, therefore there is no separation along $\hat{Z}_{5}$
* $\theta_{5}$ := $\theta_{5}$
    * as joint 5 is a revolute joint and $\hat{X}_{4}$ and $\hat{X}_{5}$ are coincident when $\theta_{5}$ is set to zero.  

##### Joint 6
* $\alpha_{5}$ := -$\frac{\pi}{2}$
    * as a negative (right-hand) rotation is required about $\hat{X}_{5}$ in order to transform $\hat{Z}_{5}$ to $\hat{Z}_{6}$
    * $\alpha_{5}$ is easily observed in X-Y projection in **Figure 6**
* $a_{5}$ := 0
    * as $\hat{Z}_{5}$ and $\hat{Z}_{6}$ are coincident, thus there is no separation between these axes along $\hat{X}_{5}$
* $d_{6}$ := 0
    * as $\hat{X}_{5}$ and $\hat{X}_{6}$ are coincident, therefore there is no separation along $\hat{Z}_{6}$
* $\theta_{6}$ := $\theta_{6}$
    * as joint 5 is a revolute joint and $\hat{X}_{5}$ and $\hat{X}_{6}$ are coincident when $\theta_{6}$ is set to zero.

![alt text][J3J6]  
**Figure 6 - Demonstration of DH parameter calculation for joints 4, 5, and 6.
The choice for frame orientations reduces the number of non-zero DH parameters.
Notice that $\hat{X}_{4, 5, 6}$ are coincident, and that $\hat{X}_{3}$ is
parallel to these axes (indicated by the dashed demarcation in the Y-Z
projection)**

##### End-Effector (EEF)
* $\alpha_{5}$ := 0
    * as the gripper is a fixed link to joint 6
* $a_{6}$ := 0
    * as $\hat{Z}_{6}$ and $\hat{Z}_{G}$ are coincident, thus there is no separation between these axes along $\hat{X}_{6}$
* $d_{7}$ := 0.303
    * as the distance from $\hat{X}_{6}$ to $\hat{X}_{G}$ along $\hat{Z}_{6}$ incorporates the x-frame lengths for joint 6 (0.193 m) and the end-effector (0.11 m).
    * $d_{7}$ is easily observed in the X-Z or Y-Z projection in **Figure 7**
* $\theta_{G}$ := 0
    * as the gripper is a fixed link upon joint 6.  

![alt text][J6JG]  
**Figure 7 - Demonstration of DH parameter calculation for the gripper link.  
The gripper link is fixed to joint 6, and thus the only parameters necessary for
translation from J6 to the end-effector are the lengths of the J6 link length,
as well as the gripper length.  These values sum and are set as $d_{7}$.**

#### DH parameter table
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | $\theta_{1}$
1->2 | -$\frac{\pi}{2}$ | 0.35 | 0 | $\theta_{2}$ - $\frac{\pi}{2}$
2->3 | 0 | 1.25 | 0 | $\theta_{3}$
3->4 |  -$\frac{\pi}{2}$ | -0.054 | 1.5 | $\theta_{4}$
4->5 | $\frac{\pi}{2}$ | 0 | 0 | $\theta_{5}$
5->6 | -$\frac{\pi}{2}$ | 0 | 0 | $\theta_{6}$
6->EE | 0 | 0 | 0.303 | 0

### Generate homegenous transformation matrices

Homogenous transforms were generated using these DH parameters as arguments to
a utility function, `make_TF`, from the `kuka_arm/scripts/utils.py` module, with
the relevant code as follows:

```
T = Matrix([[cos(theta), -sin(theta), 0, a],
            [sin(theta)*cos(alpha), cos(theta)*cos(alpha),
             -sin(alpha), -sin(alpha)*d],
            [sin(theta)*sin(alpha), cos(theta)*sin(alpha),
             cos(alpha), cos(alpha)*d],
            [0, 0, 0, 1]])
```
The transformation for each link was built and then the sequence of
transformations were composed into a homogeneous transform between the
base_link and the end effector, in which only the $\theta_{i}$ values are
required for evaluation.  

### Inverse Kinematic Solution



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
