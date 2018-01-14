# Project: Kinematics Pick & Place

## Summary

The objective of this project was to develop a simple inverse kinematic solver for the KUKA210 robot, for the purpose of
an object pick and place task.  To accomplish this objective, Denavit-Hartenberg (DH) parameters were defined using the
KUKA210 urdf file, and then used to generate homogeneous transformation matrices for each of the 6 joints of the robot.
Individual transformation matrices were composed to generate a total transformation matrix capable of providing forward
kinematic solutions.  Finally, a trigonometric solution was generated in order to solve for a joint state, given an
end-effect target pose.   This inverse kinematics solution was used within a ROS service required for completion of the
pick and place task.    

This report outlines the process by which each of these steps were completed, and the results of the pick and place
task.     

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
[result1]: ./misc_images/solution2.png
[result2]: ./misc_images/solution3.png

## Abbrieviations
IK - Inverse Kinematic(s)
EEF - end-effector
WC - Wrist Center

## Kinematic Analysis

### Definition of Denavit-Hartenberg Parameters

#### Represent the KUKA210 robot in a zero joint angle configuration

In order to define DH parameters, joint axes must first be defined for the zero angle configuration of the robot.
**Figure 1** shows an illustration of the robot projected on the global X-Z plane (with positive y heading towards the
viewer).

![alt text][zero]

>Figure 1 - Illustration of the zero angle configuration for the KUKA210 robot. The robot is represented as a projection
onto the global x, z plane,  with the positive y direction leaving the page toward the viewer (for clarity, the
gripper orientation is not represented accurately in this schematic).  Joints are  shown in purple, and links between
joints are shown in blue.  Each joint and link are numbered in accordance with DH parameter guidelines.


Origins for each joint, with their respective positive $\hat{Z}$ and $\hat{X}$ axes must then be defined.  Choice of
where to orient each joint is made with the objective of minimizing the number of non-zero DH parameters.  The direction
for positive $\hat{Z}_{i}$ and $\hat{X}_{i}$ axes must be made such that $\hat{X}_{i-1}$ must be  perpendicular to both
$\hat{Z}_{i-1}$ and $\hat{Z}_{i}$ These definitions can be observed in **Figure 2**.

![alt text][axes]

>**Figure 2 - Definition of positive Z and X axes for KUKA210 robot.** Placement of axes origins were made in order to
minimize the number of non-zero DH parameters.  For instance, setting the origin for joints 4, 5, and 6 as conincident
allows reduces the number of non-zero DH parameters for these joints down to solely their joint rotation (theta values)
and twist rotation (alpha values).


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

> **Figure 3 - Demonstration of DH parameter calculation for Joint 1.**  Joint 1 and surrounding links are shown as
projections into the global X-Z, X-Y, and Y-Z planes.  Notice that d1 is easily depicted with either 2D projection
including the Z axis.  In the middle panel, in which the Z axis points directly towards the viewer, $\hat{X}_{0}$ and
$\hat{X}_{1}$ are parallel but not conincident (represented here and henceforth by the dashed marcation).

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

>**Figure 4 -  Demonstration of DH parameter calculation for Joint 2.**  The a1 distance is easily observed when the
global Z-axis is present in the projection. The twist angle is observable in the Y-Z projection, whereas the theta
offset is most easily observable in the X-Z projection.

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
>**Figure 5 - Demonstration of DH parameter calculation for Joint 3.**  The a2 parameter is clearly shown in projections
X-Z and Y-Z.

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
>**Figure 6 - Demonstration of DH parameter calculation for joints 4, 5, and 6.** The choice for frame orientations
reduces the number of non-zero DH parameters. Notice that $\hat{X}_{4, 5, 6}$ are coincident, and that $\hat{X}_{3}$ is
parallel to these axes (indicated by the dashed demarcation in the Y-Z projection).

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
>**Figure 7 - Demonstration of DH parameter calculation for the gripper link.**   The gripper link is fixed to joint 6,
and thus the only parameters necessary for translation from J6 to the end-effector are the lengths of the J6 link
length, as well as the gripper length.  These values sum and are set as $d_{7}$.

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

Homogenous transforms were generated using these DH parameters as arguments to a utility function, `make_TF`, from the
`kuka_arm/scripts/utils.py` module, with the relevant code as follows:

```
T = Matrix([[cos(theta), -sin(theta), 0, a],
            [sin(theta)*cos(alpha), cos(theta)*cos(alpha),
             -sin(alpha), -sin(alpha)*d],
            [sin(theta)*sin(alpha), cos(theta)*sin(alpha),
             cos(alpha), cos(alpha)*d],
            [0, 0, 0, 1]])
```

The transformation for each link was built and then the sequence of transformations were composed into a homogeneous
transform between the base_link and the end effector, in which only the $\theta_{i}$ values are required for evaluation.
The composition of these link transforms is completed by the `ParamServer.generate_homegenous_transforms` method in
parameters.py as follows:

$$ {}_{0}^{EE}T = {}_{0}^{1}T \times {}_{1}^{2}T \times {}_{2}^{3}T \times {}_{3}^{4}T \times {}_{4}^{5}T \times {}_{5}^{6}T \times {}_{6}^{EE}T $$

Evaluation of $ {}_{0}^{EE}T $ by substituting the symbolic $\theta$ variables will provide a matrix that explicitly
describes the end effector position and implicity describes the end-effector orientation (see Inverse Kinematic Solution
section for more on calculation of euler angle orientation from a rotation matrix).

$$ {}_{0}^{EE}T = \left[\begin{array} {rrrr} r_{11} & r_{12} & r_{13} & EE_{x} \\ r_{21} & r_{22} & r_{23} & EE_{y} \\
r_{31} & r_{32} & r_{33} & EE_{z} \\ 0 & 0 & 0 & 1 \end{array}\right] $$, where the $EE_{xyz}$ represent the coordinates
of the end-effector in the frame of  the base link.  

### Inverse Kinematic Solution

The term, inverse kinematics (IK), describes the problem of determining what robot state(s) (joint angles, prismatic
link lengths, etc) will produce a desired end-effector position and orientation.  For 6-DOF robots like the KUKA210,
there are often several IK solutions for each desired end-effector pose.  Solutions to IK problems can be realized using
open-form (gradient or random sampling), or close-form methods.  

In mathematical terms, the IK problem asks what set of variable parameters (i.e. $\theta_{0:6}$) satisfy:
$${}_{0}^{EE}T_{symbolic} = {}_{0}^{EE}T_{target}$$
, where:
* ${}_{0}^{EE}T_{symbolic}$ represents the unevalulated homogenous transformation
from base link to EE maintaining symbolic representation of $\theta_{0:6}$, and
* ${}_{0}^{EE}T_{target}$ represents the evaluated homegenous transformation
matrix taking into account the desired EE positions and gripper orientation.  

For this project, a closed-form solution was attempted.   In order to generate a closed form IK solver, it is
advantageous to decouple the position from the WC-to-EE orientation.  In other words, we stipulate that joints 1, 2 and
3 are utilized  solely to generate the correct EE position given a specific final EE orientation, which  is accomplished
solely through rotations in Joints 4, 5, and 6.   While this  decoupling stipulation is not essential, it makes
close-formed solution  acheivable using basic trigonometric analysis.  

#### Step 1 - Define the target Transformation matrix

The first step towards IK solution is to define ${}_{0}^{EE}T_{target}$ for the
target pose.   $$ {}_{0}^{EE}T_{target} = \left[\begin{array} {rrrr} r_{11} &
r_{12} & r_{13} & EE_{x} \\ r_{21} & r_{22} & r_{23} & EE_{y} \\ r_{31} & r_{32} &
r_{33} & EE_{z} \\ 0 & 0 & 0 & 1 \end{array}\right] $$ , where:
$$\left[\begin{array} {rrr} r_{11} & r_{12} & r_{13}  \\ r_{21} & r_{22} &
r_{23}  \\ r_{31} & r_{32} & r_{33}   \end{array}\right] =
{}_{0}^{EE}R_{target}$$ - i.e. the rotation matrix representing the target EE
orientation in the base link (in this case, also the world) frame, generated
using the roll, pitch, and yaw rotations given as the desired EE pose.
$EE_{xyz}$ represent the positions of the EE in the frame of the base link (in
this case, also the world frame)

This symbolic transformation matrix only needs to be generated once, and is done
so during the initiation  of the `ParamServer` object from parameters.py
(initiation occurs on line 33 in IK_server.py).  This generation incorporates
the given roll, pitch, and yaw angles into a $R_{zyx}$ matrix that has been
corrected to place the urdf frame definition into the frame of the base link (
also the world frame).  

#### Step 2 - Evaluate the coordinates of the Wrist Center (WC)


The next step to the IK solution is to work backwards from the ${}_{0}^{EE}T_{target}$ to obtain the positions of the WC
in the world frame.  This is accomplished by taking the z-frame of the ${}_{0}^{EE}R_{target}$, which represents the
vector describing the translation from the WC to the EE.  Thus, the WC position can be determined by translating the EE
position backwards along the length the gripper:

$$ \left[\begin{array}
{r}
WC_{x} \\
WC_{y} \\
WC_{z}
\end{array}\right] =
\left[\begin{array}
{r}
EE_{x} \\
EE_{y} \\
EE_{z}
\end{array}\right] - (d_{7} \times
\left[\begin{array}
{r}
r_{13} \\
r_{23} \\
r_{33}
\end{array}\right] )
$$

The code for this step can be found in line 38 of kinematics.py in the `Solver.solve_IK` method.  

#### Step 3 - Utilize the WC position to solve for $\theta_{1}$

$\theta_{1}$ represents the twist in joint 1.  Thus it can be solved trigonometrically by projecting the WC position
into its X and Y components, ignoring the Z-projection. An illustration of $\theta_{1}$ is shown in **Figure 8**.  

![alt_text][Theta1]
>**Figure 8 - Illustration of $\theta_{1}$ in the X-Y projection (from above) of the robot state**.  Joint 1, 2, 3, and
the spherical wrist are illustrated from left to right.  The angle between $\hat{X}_{0}$ and $\hat{X}_{1}$ is the
arctangent of the $WC_{y}$ and $WC_{x}$ components.

From this depiction it is clear that: $$ tan(\theta_{1}) = \frac{WC_{y}}{WC_{x}} $$ , thus:

$$ \theta_{1} = atan2(WC_{y}, WC_{x}) $$

Notably, both this solution to $\theta_{1}$ as well as $\pi - \theta_{1}$ could be part of valid solutions for most
6-DOF robots (where joint 2 can accomodate this  extra pi rotation by flipping all the way over).  However, this is not
the case for the KUKA210, due to the joint limit on Joint 2. Thus $\pi - \theta_{1}$ is ignored for this solution.  

#### Step 4 - Project robot state into X-Z to solve for $\theta_{2}$

Depiction of the $\theta_{2}$ is shown in **Figure 9**.  Because the definition of $\hat{X}_{1}$ and $\hat{X}_{2}$ are
orthoganol, a constant rotation of negative $\frac{\pi}{2}$ radians is shown.  The projection of the robot state onto
the X-Z plane illustrates the rotation from this adjusted $\hat{X}_{1}$ to $\hat{X}_{2}$ rotation about $\hat{Z}_{1}$.
The triangle generated by Joint 2, Joint 3, and the WC can be exploited in order to calcluate $\theta_{2}$.  Sides C and
A are known from the robot description, whereas the length of side B can be determined from applying the pythagorean
theorem to the sides of the right triangle that is formed by the vector between Joint 2 and the WC in the X-Z plane.
The cosine rule can be applied to this SSS triangle to obtain each angle of its angles.  

![alt_text][theta2]
>**Figure 9 - Demonstration of the calculation of $\theta_{2}$.** The $\theta_{2}$ angle can be observed by projecting
the robot state into the X-Z plane (left panel).  A simplification of the robot state is shown in the right panel to aid
description of the method for calculating the angle.  $\theta_{2}$ is the difference between $\frac{\pi}{2}$ and the sum
of angle a with the arctangent of the green right triangle.

From this depiction, it is demonstrated that:
$$ \frac{\pi}{2} = \theta_{2} + acos(\frac{(B^{2} + C^{2} - A^{2}}{(2\times B\times C)}) + atan2(WC_{z}-d_{1}, WC_{x} - a_{1})$$, where $ acos(\frac{(B^{2} + C^{2} - A^{2}}{(2\times B\times C)}) $ provides angle $a$ in the diagram, and thus:
$$ \theta_{2} = \frac{\pi}{2} - acos(\frac{(B^{2} + C^{2} - A^{2}}{(2\times B\times C)}) - atan2(WC_{z}-d_{1}, WC_{x} - a_{1})$$

The code for this calculation is found within `Solver.find_theta23` in kinematics.py, in lines 63-75.  

#### Step 5 - Project the WC into the frame of $\hat{X}_{2}$ to solve for $\theta_{3}$

Depiction of $\theta_{3}$ is less straight forward, however by displaying the Joint 2, Joint3, and WC in the frame of
the $\hat{X}_{2}$ axis helps to reveal the rotation described by this value.  **Figure 10** depicts where the the WC
would be located in this custom frame if $\theta_{3}$ were set to zero.  By drawing an orthoganol to $\hat{X}_{2}$,  we
can show that: $\frac{\pi}{2} = m + b$, when $\theta_{3} = 0$.

The angle $b$ can be calculated from application of the cosine rule to the SSS triangle shown in **Figure 9**.  The
angle $m$ can be calculated using the arctangent of parameters $a_{3}$ and $d_{4}$.    

![alt_text][theta3setup]
>**Figure 10 - Visualization of joint 3 to the WC in the frame of $\hat{X}_{2}$.** Joints 2, 3 and the WC are shown in
purple, with robot links in blue. An orthoganol to $\hat{X}_{2}$ that lies in the global X-Z frame is shown in dashed
red.  The vector that describes the Joint 3 to WC position is shown in dashed green, however due to our DH placement of
the WC, this does not represent the link between Joint 3 to Joint 4.  The links from Joint 3 to 4 and from 4 to 5 are
shown in dashed blue.  Notably, $\frac{\pi}{2} = m +b$, when $\theta_{3} = 0$.  

It is also useful to point out that $\hat{X}_{3}$ is coincident with $\hat{X}_{2}$ in the zero angle robot
configuration.  A non-zero joint 3 angle is depicted in **Figure 11**.  From this description, it is clear that the
$\theta_{3}$ angle is equal to the difference between $\frac{\pi}{2}$ and the sum of angles $b$ and $m$.  

![alt_text][theta3]
>**Figure 11 - Demonstration of a non-zero $\theta_{3}$ angle.**  

From this analysis it is clear that:
$$ \frac{\pi}{2} = \theta_{3} + m + b$$, thus:
$$ \theta_{3} = \frac{\pi}{2} - atan2(a_{3}, d_{4}) - acos(\frac{(A^{2} + C^{2} - B^{2}}{(2\times A\times C)})  $$

The code for this calculation is found within `Solver.find_theta23` in kinematics.py, in lines 63-76.  

#### Step 6 - Transform the ${}_{0}^{EE}R_{target}$ into the frame of the current WC

To accomplish this, the following matrix multiplication is required: $$ {}_{WC}^{EE}R_{target} = {}_{0}^{WC}R^{-1}
\times {}_{0}^{EE}R_{target}$$

To obtain ${}_{0}^{WC}R^{-1}$, the homegenous transform from base link to link 3 is evaluated using the  $\theta_{1}$,
$\theta_{2}$, and $\theta_{3}$ angles already calculated.  This evaluated matrix is transposed and the multiplied into
the evaluated target EE rotation matrix. The result of this multiplication is a matrix of float values (i.e. not
symbolic) that represent the rotation from the zero angle wrist orientation to the target wrist orientation given the
current orientation of joints 1, 2, and 3.  

#### Step 7 - Generate the symbolic rotation matrix corresponding to ${}_{0}^{EE}R_{target}$

Finally, values must be found for $\theta_{4}$, $\theta_{5}$, and $\theta_{6}$ that satisfy the following evaluation:

$${}_{WC}^{EE}R_{sym} = {}_{WC}^{EE}R_{target}$$

${}_{WC}^{EE}R_{sym}$ is defined as the rotation matrix parsed from the composition of individual homegenous
transformations from link 3 to the end effector:

$$ {}_{WC}^{EE}T = {}_{WC}^{4}T \times {}_{4}^{5}T \times {}_{5}^{6}T \times {}_{6}^{EE}T$$

$${}_{WC}^{EE}T =
\left[\begin{array} {rrrr} r_{11} & r_{12} & r_{13} & {}^{WC}EE_{x} \\ r_{21} & r_{22} & r_{23} & {}^{WC}EE_{y} \\ r_{31} & r_{32} &
r_{33} & {}^{WC}EE_{z} \\ 0 & 0 & 0 & 1 \end{array}\right] $$

$$ {}_{WC}^{EE}R_{sym} = \left[\begin{array} {rrr} r_{11} & r_{12} & r_{13}  \\
r_{21} & r_{22} & r_{23}  \\ r_{31} & r_{32} & r_{33}   \end{array}\right] $$


When simplified, the WC to EE rotation matrix can be represented as:

$${}_{WC}^{EE}R_{sym} =
\left[\begin{array}
{rrr}
-sin(\theta_{4}) \times sin(\theta_{6}) + cos(\theta_{4}) \times cos(\theta_{5}) \times cos(\theta_{6})& -sin(\theta_{4}) \times cos(\theta_{6}) - sin(\theta_{6}) \times cos(\theta_{4}) \times cos(\theta_{5})& -sin(\theta_{5}) \times cos(\theta_{4}) \\ sin(\theta_{5}) \times cos(\theta_{6})& -sin(\theta_{5}) \times sin(\theta_{6})& cos(\theta_{5}) \\ -sin(\theta_{4}) \times cos(\theta_{5}) \times cos(\theta_{6}) - sin(\theta_{6}) \times cos(\theta_{4})& sin(\theta_{4}) \times sin(\theta_{6}) \times cos(\theta_{5}) - cos(\theta_{4}) \times cos(\theta_{6})& sin(\theta_{4}) \times sin(\theta_{5})
\end{array}\right]
$$

Any set of $\theta_{4}$, $\theta_{5}$, and $\theta_{6}$ that statisfies

$${}_{WC}^{EE}R_{sym} = {}_{WC}^{EE}R_{target}$$

, where $ {}_{WC}^{EE}R_{target}$ represents the matrix of floats generated from
step 6, will produce a valid orientation in the pick and place task.  To
accomplish this, the symbolic matrix can be exploited in order to
trigonometrically solve for each of the $\theta_{4}$, $\theta_{5}$, and
$\theta_{6}$ angles.


#### Step 8 - Solve for wrist joint angles

##### Solve for $\theta_{5}$

Looking at the ${}_{WC}^{EE}R_{sym}$ matrix, the easiest angle for which to solve appears to be  $\theta_{5}$, as
$$r_{23} = cos(\theta_{5})$$, and thus

$$\theta_{5} = acos(r_{23})$$

, where $r_{23}$ is the value from the 2nd row and
third column from  ${}_{WC}^{EE}R_{target}$.  Because $cos(\theta) = cos(-\theta)$,  then both  

$$\theta_{5}^{1} = acos(r_{23})$$
$$\theta_{5}^{2} = -acos(r_{23})$$

must  be considered valid solutions for $\theta_{5}$.

#####  Solve for $\theta_{4}$

At first glance, solving for  $\theta_{4}$ does not present an obvious exploitation within ${}_{WC}^{EE}R_{sym}$.
However, considering the identify function

$$ tan(\theta) = \frac{sin(\theta)}{cos(\theta)}$$

with theta solved via

$$ \theta = atan2(sin(\theta), cos(\theta))$$

In this manner, $\theta_{4}$ can be found by generating this expression with
the quotient of $r_{33}$ and $r_{13}$ as follows:

$$ \frac{r_{33}}{r_{13}} = \frac{sin(\theta_{4}) \times sin(\theta_{5})}{sin(\theta_{5}) \times -cos(\theta_{4})}$$

$$ \frac{r_{33}}{r_{13}} = -1 \times \frac{sin(\theta_{4})}{cos(\theta_{4})}$$

$$\frac{r_{33}}{r_{13}} = -1 \times tan(\theta_{4})$$

$$ tan(\theta_{4}) = \frac{r_{33}}{-r_{13}} = \frac{-r_{33}}{r_{13}} $$

Thus $\theta_{4}$ can be derived from either of

$$ \theta_{4}^{?} = atan2(r_{33}, -r_{13}) $$

$$ \theta_{4}^{?} = atan2(-r_{33}, r_{13}) $$

But which of these solutions corresponds to the appropriate of the two $\theta_5$
angles?  One can utilize the sign of $sin(\theta_5)$ to define the quadrant
described by the terms received by the atan2 function.  For instance, when the
$sin(\theta_{5}) > 0$, then

$$ atan2(r_{33}, -r_{13}) =  atan2(\frac{r_{33}}{sin(\theta_{5})}, \frac{-r_{13}}{sin(\theta_{5})}) $$

and when $sin(\theta_{5}) < 0 $, then

$$ atan2(-r_{33}, r_{13}) =  atan2(\frac{r_{33}}{sin(\theta_{5})}, \frac{-r_{13}}{sin(\theta_{5})}) $$

Because the two solutions for $\theta_{5}$ opposite magnitudes, then we can
always define the solutions for $\theta_{4}$ as

$$ \theta_{4}^{1} =  atan2(\frac{r_{33}}{sin(\theta_{5}^{1})}, \frac{-r_{13}}{sin(\theta_{5}^{1})})$$

$$ \theta_{4}^{2} =  atan2(\frac{r_{33}}{sin(\theta_{5}^{2})}, \frac{-r_{13}}{sin(\theta_{5}^{2})})$$

This process will automatically assign the correct quadrant for the $\theta_{4}$
for all cases of $\theta_{5}$, except where $sin(\theta_{5}) = 0$.  In this  
special case, a different approach must be made, which is described in the
'Solving for $\theta_{4}$ and $\theta_{6}$ when $sin(\theta_{5}) = 0$' section.

##### Solve for $\theta_{6}$

In similar fashion to $\theta_{4}$, $\theta_{6}$ can be solved by recognizing that:

$$ \frac{r_{22}}{r_{21}} = \frac{-sin(\theta_{5}) \times sin(\theta_{6})}{sin(\theta_{5}) \times cos(\theta_{6})}$$

$$ \frac{r_{22}}{r_{21}} = -1 \times \frac{sin(\theta_{6})}{cos(\theta_{6})}$$

$$ tan(\theta_{6}) =  \frac{-r_{22}}{r_{21}} =  \frac{r_{22}}{-r_{21}} $$

With the similar trick described in solving for $\theta_{4}$, solutions for $\theta_{6}$,
can be defined as

$$ \theta_{6}^{1} =  atan2(\frac{-r_{22}}{sin(\theta_{5}^{1})}, \frac{r_{21}}{sin(\theta_{5}^{1})}) $$

$$ \theta_{6}^{2} = atan2(\frac{-r_{22}}{sin(\theta_{5}^{2})}, \frac{r_{21}}{sin(\theta_{5}^{2})}) $$


##### Solving for $\theta_{4}$ and $\theta_{6}$ when $sin(\theta_{5}) = 0$

When $\theta_{5}$ is equivalent to $\pm{\pi}$ or $0$, then the $sin(\theta_{5}) = 0$,
which makes the trick for combining the corresponding joint
angles together return as undefined (divide by zero).  In these cases
a different approach must be taken.  This approach begins by recognizing that
when $sin(\theta) = 0$, then the $cos(\theta) = \pm{1}$.  Therefore, $r_{11}$
can be simplified as follows

$$ r_{11} =  -sin(\theta_{4}) \times sin(\theta_{5}) +
cos(\theta_{4}) \times cos(\theta_{5}) \times cos(\theta_{6})$$

$$ r_{11} =  -sin(\theta_{4}) \times sin(\theta_{5}) +
cos(\theta_{4}) \times \pm{1} \times cos(\theta_{6})$$

Using identify functions we can redefine these equations as

$$ r_{11} =  cos(\theta_{4} + \theta_{6})$$

$$ r_{11} =  -sin(\theta_{4} + \theta_{6})$$

Any set of $\theta_{4}$ and $\theta_{6}$ that satisify these equations will be
valid with the given $\theta_{5}$ value.  

## Code implementation of solving for an IK solution  

The code for these step is located within kinematics.py.  This module contains
an object which is initiated with an instance of ParamServer, containing the
requisite transformation matrices and DH-parameters.  The Solver object contains
a method `Solver.solve_IK` that receives the target postion and orientation for
the EE, and returns a list of solution sets including the solution
provided by the Udacity demo (not discussed), and the two solution sets
described by the steps above.

## Pick and Place Implementation

The logic described above is deployed in IK_server.py within the
`handle_calculate_IK` function.  This function receives a list of poses for a
trajectory plan, and returns a list of joint angle states that will yield the
trajectory of poses.  As noted above, 3 solution sets are returned from the
`Solver.solve_IK` method.  Currently, the default solution
set corresponds to $(\theta_{1}, \theta_{2}, \theta_{3}, \theta_{4}^{1}, \theta_{5}^{1}, \theta_{6}^{1})$.  

The objective of this project was ultimately to allow a pick and place task to
complete without error.  This objective was accomplished for three solution
sets.  

![alt_text][result1]
>**Figure 12- Results of running the pick and place task with the implemented IK
service using solution set 1**

![alt_text][result2]
>**Figure 12- Results of running the pick and place task with the implemented IK
service using solution set 2**

## Discussion

This project implements a single solution to the Inverse Kinematics problem, in
which a target pose (EE-position and orientation) is converted into a valid set
of joint angle states.  
