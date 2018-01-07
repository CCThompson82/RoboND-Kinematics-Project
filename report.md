 * # Project: Kinematics Pick & Place

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
[theta1]: ./misc_images/Theta1.jpg
[theta2]: ./misc_images/theta2.jpg
[theta3]: ./misc_images/Theta3.jpg
[theta3setup]: ./misc_images/Theta3setup.jpg
[result]: ./misc_images/result_img.png

## Abbrieviations
IK - Inverse Kinematic(s)
EEF - end-effector
WC - Wrist Center

## Kinematic Analysis

### Definition of Denavit-Hartenberg Parameters

#### Represent the KUKA210 robot in a zero joint angle configuration

In order to define DH parameters, joint axes must first be defined for the zero
angle configuration of the robot.  **Figure 1** shows an illustration of the
robot projected on the global X-Z plane (with positive y heading towards the
viewer).

![alt text][zero]
>Figure 1 - Illustration of the zero angle configuration for the KUKA210 robot.
The robot is represented as a projection onto the global x, z plane,  with
the positive y direction leaving the page toward the viewer (for clarity, the  
gripper orientation is not represented accurately in this schematic).  Joints
are  shown in purple, and links between joints are shown in blue.  Each joint
and link are numbered in accordance with DH parameter guidelines.


Origins for each joint, with their respective positive $\hat{Z}$ and $\hat{X}$
axes must then be defined.  Choice of where to orient each joint is made with
the objective of minimizing the number of non-zero DH parameters.  The direction
for positive $\hat{Z}_{i}$ and $\hat{X}_{i}$ axes must be made such that $\hat{X}_{i-1}$ must be  perpendicular to both $\hat{Z}_{i-1}$ and $\hat{Z}_{i}$
These definitions can be observed in **Figure 2**.

![alt text][axes]
>**Figure 2 - Definition of positive Z and X axes for KUKA210 robot.**
Placement of axes origins were made in order to minimize the number of non-zero DH
parameters.  For instance, setting the origin for joints 4, 5, and 6 as
conincident allows reduces the number of non-zero DH parameters for these joints
down to solely their joint rotation (theta values) and twist rotation (alpha
values).


#### Calculate DH parameters using the KUKA210 urdf

For each joint, the following parameters must be defined:
* twist angle ($\alpha_{i-1}$) - angle (radians) between $\hat{Z}_{i-1}$ and $\hat{Z}_{i}$, about $\hat{X}_{i-1}$ using the right-hand rule
* link length ($a_{i-1}$) - signed distance between $\hat{Z}_{i-1}$ and $\hat{Z}_{i}$ along $\hat{X}_{i-1}$
* link offset ($d_{i}$) - signed distance from $\hat{X}_{i-1}$ to $\hat{X}_{i}$, along $\hat{Z}_{i}$
* joint angle ($\theta_{i}$) - angle between $\hat{X}_{i-1}$ to $\hat{X}_{i}$, about $\hat{Z}_{i}$ using the right-hand rule

##### Joint 1
* $\alpha_{0}$ := 0
    * as $\hat{Z}_{0}$ and $\hat{Z}_{1}$ are coincident and therefore there is no rotation between these axes about $\hat{X}_{0}$
* $a_{0}$ := 0
    * as $\hat{Z}_{0}$ and $\hat{Z}_{1}$ are coincident and therefore there is no separation between these axes along $\hat{X}_{0}$
* $d_{1}$ := 0.75
    * as the distance from $\hat{X}_{0}$ to $\hat{X}_{1}$ along $\hat{Z}_{1}$ incorporates the z-lengths for both joint1 (0.33) and joint2 (0.42) in the urdf file.
    * $d_{1}$ is easily observed in the X-Z or Y-Z projection in **Figure 3**
* $\theta_{1}$ := $\theta_{1}$
    * as joint 1 is a revolute joint and $\hat{X}_{0}$ to $\hat{X}_{1}$ are parallel when $\theta_{1}$ is set to zero.  

![alt text][J0J1]  

> **Figure 3 - Demonstration of DH parameter calculation for Joint 1.**  Joint 1
and surrounding links are shown as projections into the global X-Z, X-Y, and Y-Z
planes.  Notice that d1 is easily depicted with either 2D projection including
the Z axis.  In the middle panel, in which the Z axis points directly towards
the viewer, $\hat{X}_{0}$ and $\hat{X}_{1}$ are parallel but not conincident
(represented here and henceforth by the dashed marcation).

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

>**Figure 4 -  Demonstration of DH parameter calculation for Joint 2.**  The a1
distance is easily observed when the global Z-axis is present in the projection.
The twist angle is observable in the Y-Z projection, whereas the theta offset is
most easily observable in the X-Z projection.

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
>**Figure 5 - Demonstration of DH parameter calculation for Joint 3.**  The a2
parameter is clearly shown in projections X-Z and Y-Z.

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
>**Figure 6 - Demonstration of DH parameter calculation for joints 4, 5, and 6.**
The choice for frame orientations reduces the number of non-zero DH parameters.
Notice that $\hat{X}_{4, 5, 6}$ are coincident, and that $\hat{X}_{3}$ is
parallel to these axes (indicated by the dashed demarcation in the Y-Z
projection).

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
>**Figure 7 - Demonstration of DH parameter calculation for the gripper link.**  
The gripper link is fixed to joint 6, and thus the only parameters necessary for
translation from J6 to the end-effector are the lengths of the J6 link length,
as well as the gripper length.  These values sum and are set as $d_{7}$.

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
required for evaluation.  The composition of these link transforms is completed
by the `ParamServer.generate_homegenous_transforms` method in parameters.py.  

### Inverse Kinematic Solution

The term, inverse kinematics (IK), describes the problem of determining what robot
state(s) (joint angles, prismatic link lengths, etc) will produce a desired
end-effector position and orientation.  For 6-DOF robots like the KUKA210, there
are often several IK solutions for each desired end-effector pose.  Solutions
to IK problems can be realized using open-form (gradient or random sampling), or
close-form methods.  

In mathematical terms, the IK problem asks what set of variable parameters (i.e. $\theta_{0:6}$)
satisfy:
$${}_{0}^{EE}T_{symbolic} = {}_{0}^{EE}T_{target}$$
, where:
* ${}_{0}^{EE}T_{symbolic}$ represents the unevalulated homogenous transformation
from base link to EE maintaining symbolic representation of $\theta_{0:6}$, and
* ${}_{0}^{EE}T_{target}$ represents the evaluated homegenous transformation
matrix taking into account the desired EE positions and gripper orientation.  

For this project, a closed-form solution was attempted.   In order to generate a
closed form IK solver, it is advantageous to decouple the position from the
WC-to-EE orientation.  In other words, we stipulate that joints 1, 2 and 3 are
utilized  solely to generate the correct EE position given a specific final EE
orientation, which  is accomplished solely through rotations in Joints 4, 5, and 6.  
While this  decoupling stipulation is not essential, it makes close-formed
solution  acheivable using basic trigonometric analysis.  

#### Step 1 - Define the target Transformation matrix
The first step towards IK solution is to define ${}_{0}^{EE}T_{target}$ for the
target pose.  
$$ {}_{0}^{EE}T_{target} =
\left[\begin{array}
{rrrr}
r11 & r12 & r13 & EE_{x} \\
r21 & r22 & r23 & EE_{y} \\
r31 & r32 & r33 & EE_{z} \\
0 & 0 & 0 & 1
\end{array}\right]
$$
, where:
* $\left[\begin{array}
{rrr}
r11 & r12 & r13  \\
r21 & r22 & r23  \\
r31 & r32 & r33  
\end{array}\right] = {}_{0}^{EE}R_{target}$
  - i.e. the rotation matrix representing the target EE orientation in the base link (in this case, also the world) frame
* $EE_{xyz}$ represent the positions of the EE in the frame of the base link (in this case, also the world frame)

This transformation matrix is built in lines 

#### Step 2 - Evaluate the coordinates of the Wrist Center (WC)


The next step to the IK solution is to work backwards from the target EEF
position to find the corresponding WC position given the target manipulator
pose.

A rotation matrix describing the relationship between roll, pitch, and yaw and
the end effector orientation was constructed.  This rotation matrix is then
evaluated after substitution with the target euler angles (converted from the
target quaternion).  The z-frame of this rotation matrix is the u, v, w vector
orientation of the WC-to-EEF, and is used to calculate the corresponding WC
position that would generate the target EEF position, given the target
WC-to-EEF orientation.  The code for these steps can be found in lines 54-57 in
`IK_server.py`.

#### Solution of $\theta_{1}$
$\theta_{1}$ represents the twist in joint 1.  Thus it can be solved
trigonometrically by projecting the WC position into its X and Y components. An
illustration of $\theta_{1}$ is shown in **Figure 8**.  

![alt_text][Theta1]
**Figure 8 - Illustration of $\theta_{1}$ in the X-Y projection (from above) of
the robot state.  Joint 1, 2, 3, and the spherical wrist are illustrated from
left to right.  The angle between $\hat{X}_{0}$ and $\hat{X}_{1}$ is the
arctangent of the $WC_{y}$ and $WC_{x}$ components.**

#### Solution of $\theta_{2}$

Depiction of the $\theta_{2}$ is shown in **Figure 9**.  Because the definition
of $\hat{X}_{1}$ and $\hat{X}_{2}$ are orthoganol, a constant rotation of
negative $\frac{\pi}{2}$ radians is shown.  The projection of the robot state
onto the X-Z plane illustrates the rotation from this adjusted $\hat{X}_{1}$ to
$\hat{X}_{2}$ rotation about $\hat{Z}_{1}$. The triangle generated by Joint 2,
Joint 3, and the WC can be exploited in order to calcluate $\theta_{2}$.  Sides
C and A are known from the robot description, whereas the length of side B can
be determined from applying pythagorean theourem to the sides of a right
triangle  formed by the vector between Joint 2 and the WC.  The cosine rule
can be applied to this SSS triangle to obtain each angle for the shape.  Finally
the $\theta_{2}$ angle can be calculated as the difference between $\frac{\pi}{2}$
and the sum of angle a with the arctangent of the opposite and adjacent sides
of the right triangle described by the vector between Joint 2 and the WC in the
X-Z plane.  

![alt_text][theta2]
**Figure 9 - Demonstration of the calculation of $\theta_{2}$. The $\theta_{2}$
angle can be observed by projecting the robot state into the X-Z plane (left
panel).  A simplification of the robot state is shown in the right panel to aid
description of the method for calculating the angle.  $\theta_{2}$ is the
difference between $\frac{\pi}{2}$ and the sum of angle a with the arctangent
of the green right triangle.**

#### Solution of $\theta_{3}$

Depiction of $\theta_{3}$ is less straight forward, however by displaying the
Joint 2, Joint3, and WC in the frame of the $\hat{X}_{2}$ axis helps to reveal
the rotation described by this value.  **Figure 10** depicts where the the WC
would be located in this custom frame if $\theta_{3}$ were zero.  By drawing an
orthoganol to $\hat{X}_{2}$,  we can show that $\frac{\pi}{2} = m + b$ when
$\theta_{3} = 0$ (**Figure 10**).  The angle $m$  can be calculated using the
arctangent of parameters $a_{3}$ and $d_{4}$.  The  angle $b$ is known from the
analysis of the SSS triangle used to calculate  $\theta_{2}$.  

![alt_text][theta3setup]
**Figure 10 - Logic behind calculation of $\theta_{3}$, using the WC position
at a $\theta_{3}$ angle of 0 radians.**

**Figure 11** depicts the robot state both at a Joint 3 angle of 0 radians, as
well as a example non-zero position.  From this illustration, it is clear that
$\theta_{3} = \frac{\pi}{2} - m - b$.  

![alt_text][theta3]
**Figure 11 - Demonstration of the $\theta_{3}$ angle.**

#### Solution of $\theta_{4}$, $\theta_{5}$, and $\theta_{6}$
The strategy to solve for the angles of  $\theta_{4}$, $\theta_{5}$, and
$\theta_{6}$ is as follows.  

##### Generate a rotation matrix describing the transformation from link 0 to link 3
The homogeneous transformation between the base link to link 3 is represented by:
$${}_{0}^{3}T = {}_{0}^{1}T \times{}  {}_{1}^{2}T \times{} {}_{2}^{3}T$$


Because the DH parameters regarding link length, offset, and twist angle are
constant for the KUKA210, the only variables that remain undefined for this
transformation matrix to describe any valid given robot state are $\theta_{1}$, $\theta_{2}$,
and $\theta_{3}$.  The rotation matrix describing the orientation of joint 3,
given the known angles can be defined by subsetting the homegenous transformation
matrix as follows:

$${}_{0}^{3}R = {}_{0}^{3}T[0:3, 0:3]$$

Finally, $\theta_{1}$, $\theta_{2}$, and $\theta_{3}$ can be substituted for the
symbolic variables within ${}_{0}^{3}R$ to generate a useful matrix composed of
numeric floats.  

$$
\left[\begin{array}
{rrr}
-sin(q4) * sin(q6) + cos(q4) * cos(q5) * cos(q6)& -sin(q4) * cos(q6) - sin(q6) * cos(q4) * cos(q5)& -sin(q5) * cos(q4)] \\ sin(q5) * cos(q6)& -sin(q5) * sin(q6)& cos(q5) \\ -sin(q4) * cos(q5) * cos(q6) - sin(q6) * cos(q4)& sin(q4) * sin(q6) * cos(q5) - cos(q4) * cos(q6)& sin(q4) * sin(q5)
\end{array}\right]
$$


##### Describe the rotation of links 4 to the end-effector in the frame of the current links 0 to link 3
The total homegenous transformation between the base-link to the end-effector
is represented by:
$${}_{0}^{EE}T = {}_{0}^{3}T \times{}  {}_{3}^{4}T \times{} {}_{4}^{5}T \times{} {}_{5}^{6}T \times{} {}_{6}^{EE}T $$

Unevaluated, variables for all six joint angles are contained in the ${}_{0}^{EE}T$
transformation matrix.  However,

THE ROTATION MATRIX OF JOINT 3 to the EE
$$
\left[\begin{array}
{rrr}
-sin(q4) * sin(q6) + cos(q4) * cos(q5) * cos(q6)& -sin(q4) * cos(q6) - sin(q6) * cos(q4) * cos(q5)& -sin(q5) * cos(q4)] \\ sin(q5) * cos(q6)& -sin(q5) * sin(q6)& cos(q5) \\ -sin(q4) * cos(q5) * cos(q6) - sin(q6) * cos(q4)& sin(q4) * sin(q6) * cos(q5) - cos(q4) * cos(q6)& sin(q4) * sin(q5)
\end{array}\right]
$$



$atan2(sqrt((Rwc_ee[0, 2])^{2} + (Rwc_ee[2, 2])^{2}), Rwc_ee[1, 2])$
$theta4 = atan2(-Rwc_ee[2, 2], Rwc_ee[0, 2])$

Given the $\theta$ values for joints 1-3, the total homegenous
transformation  matrix can be utilized to solve for the angles of joints 4-6
that provide the  target wrist orientation using a series of arctangent
evaluations.  The code for this calculation is found in  `Solver.solve_theta456`
of the kinematics.py module.    

The results of this full solution provide one valid joint state that yield the
target pose.  Notably, more solutions to this problem exist.  The joint limits
for the KUKA210 appear to reduce the number of solutions, however, due to the
fact that Joint 2 cannot flip completely over to the reverse side of its 0
radian orientation.   

## Project Implementation

The logic described is deployed in IK_server.py within the `handle_calculate_IK`
function.  This function receives a list of poses for a trajectory plan, and
returns a list of joint angle states that will yield the trajectory of poses.  

The objective of this project was ultimately to allow a pick and place task
to complete without error.  This objective was accomplished.  In over 20 attempts,
only 2 objects were failed to be placed, both due to an insufficient grasp, where
the fingers did not close fast enough prior to extraction.  **Figure 12** shows
that the vast majority of blue columns were successfully placed.  

![alt_text][result]
**Figure 12- Results of running the pick and place task with the implemented IK
service.  Notice the number of successfully placed columns compared to the one
that was not grasped (top right), and the one that was not grasped and then
knocked across the floor!**

## Discussion

This project implements a single solution to the Inverse Kinematics problem, in
which a target pose (EEF-position and orientation) is converted into a valid
set of joint angle states.  The code allows for additional solutions to be
solved, however sufficient performance in the pick and place task was acheived
without even a second closed form solution.  It is likely that the addition of
obstacles and different trajectory plans may require several possible solutions
in order to observe the same performance.

# Text Dump
The rotation matrix representing the orientation of the end effector can be defined
as $R_{zyx}$:
$$
\left[
\begin{array}
{rrr}
cos(\theta_{5})cos(\theta_{6}) & sin(\theta_{5})sin(\theta_{4})cos(\theta_{6}) - sin(\theta_{6})cos(\theta_{4}) & sin(\theta_{5})cos(\theta_{4})cos(\theta_{6}) + sin(\theta_{4})sin(\theta_{6}) \\
sin(\theta_{6})cos(\theta_{5}) & sin(\theta_{5})sin(\theta_{4})sin(\theta_{6}) + cos(\theta_{4})cos(\theta_{6}) & sin(\theta_{5})sin(\theta_{6})cos(\theta_{4}) - sin(\theta_{4})cos(\theta_{6}) \\
-sin(\theta_{5}) & sin(\theta_{4})cos(\theta_{5}) & cos(\theta_{5})cos(\theta_{4})
\end{array}
\right]
$$

However, the urdf file describes the frame of the end-effector that is
inconsistent with the world frame in which the rest of the robot was defined.  
To account for this, a rotation correction is required, resulting in the
following matrix that describes the relationship of $\theta_{4}$, $\theta_{5}$,
and $\theta_{6}$ to the orientation of the gripper in the world frame.  

$$
\left[\begin{array}
{rrr}
sin(\theta_{5})cos(\theta_{4})cos(\theta_{6}) + sin(\theta_{4})sin(\theta_{6}) & -sin(\theta_{5})sin(\theta_{4})cos(\theta_{6}) + sin(\theta_{6})cos(\theta_{4}) & cos(\theta_{5})cos(\theta_{6}) \\
sin(\theta_{5})sin(\theta_{6})cos(\theta_{4}) - sin(\theta_{4})cos(\theta_{6}) & -sin(\theta_{5})sin(\theta_{4})sin(\theta_{6}) - cos(\theta_{4})cos(\theta_{6}) & sin(\theta_{6})cos(\theta_{5}) \\
cos(\theta_{5})cos(\theta_{4}) & -sin(\theta_{4})cos(\theta_{5}) & -sin(\theta_{5})
\end{array}\right]
$$
