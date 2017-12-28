from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##
    # import numpy as np
    # import kuka_arm.scripts.utils as utils
    # ## define DH parameters
    # d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    # a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    # alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    # q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    #
    # DH = {alpha0: 0, a0: 0, d1: 0.75, q1: q1,
    #      alpha1: -pi/2, a1: 0.35, d2: 0, q2: q2-(pi/2),
    #      alpha2: 0, a2: 1.25, d3: 0, q3: q3,
    #      alpha3: -pi/2, a3: -0.054, d4: 1.501, q4: q4,
    #      alpha4: pi/2, a4: 0, d5: 0, q5: q5,
    #      alpha5: -pi/2, a5: 0, d6: 0, q6: q6,
    #      alpha6: 0, a6: 0, d7: 0.303, q7: 0}
    #
    # T01 = utils.make_T(alpha=alpha0, a=a0, d=d1, theta=q1).subs(DH)
    # T12 = utils.make_T(alpha=alpha1, a=a1, d=d2, theta=q2).subs(DH)
    # T23 = utils.make_T(alpha=alpha2, a=a2, d=d3, theta=q3).subs(DH)
    # T34 = utils.make_T(alpha=alpha3, a=a3, d=d4, theta=q4).subs(DH)
    # T45 = utils.make_T(alpha=alpha4, a=a4, d=d5, theta=q5).subs(DH)
    # T56 = utils.make_T(alpha=alpha5, a=a5, d=d6, theta=q6).subs(DH)
    # T6EE = utils.make_T(alpha=alpha6, a=a6, d=d7, theta=q7).subs(DH)
    #
    # T0_3 = (T01 * T12 * T23)
    # T0_EE = (T0_3 * T34 * T45 * T56 * T6EE)

    from kuka_arm.scripts.parameters import ParamServer
    dhp = ParamServer()
    T0_WC, T0_EE = dhp.generate_homegenous_transforms()
    T0_3 = T0_WC

    px, py, pz = (req.poses[0].position.x, req.poses[0].position.y,
                  req.poses[0].position.z)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [req.poses[0].orientation.x, req.poses[0].orientation.y,
         req.poses[0].orientation.z, req.poses[0].orientation.w])


    ROT_EE = dhp.generate_EE_RotMat()
    ROT_EE = ROT_EE.subs({dhp.r: roll, dhp.p: pitch, dhp.y: yaw})

    EExyz = Matrix([[px], [py], [pz]])
    WC = EExyz - (dhp.DH[dhp.d7]*ROT_EE[:, 2])
    #######################################################################
    theta1 = atan2(WC[1], WC[0]) #NOTE: can also be plus pi, but requires changes to logic below such that theta2 calc uses theta1

    A = dhp.DH[dhp.d4]
    C = dhp.DH[dhp.a2]

    By = WC[2] - dhp.DH[dhp.d1]
    Bx = sqrt(WC[0]**2 + WC[1]**2) - dhp.DH[dhp.a1]
    B = sqrt((Bx)**2 + (By)**2)

    a = acos((B**2 + C**2 - A**2) / (2*B*C))
    b = acos((A**2 + C**2 - B**2) / (2*A*C))
    c = acos((A**2 + B**2 - C**2) / (2*A*B))

    theta2 = (pi/2) - a - atan2(By, Bx)
    theta3 = pi/2 - (b + 0.036)

    R0_3 = T0_3[:3, :3].evalf(subs={dhp.q1: theta1, dhp.q2: theta2, dhp.q3: theta3})
    R3_6 = R0_3.inv("LU") * ROT_EE

    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
    theta5 = atan2(sqrt((R3_6[0, 2])**2 + (R3_6[2, 2])**2), R3_6[1, 2]) #NOTE: watch out for +/-
    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]
    FK_T = T0_EE.evalf(subs={dhp.q1: theta1, dhp.q2: theta2, dhp.q3: theta3,
                             dhp.q4: theta4, dhp.q5: theta5, dhp.q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0], WC[1], WC[2]] # <--- Load your calculated WC values in this array
    your_ee = [FK_T[0, 3], FK_T[1, 3], FK_T[2, 3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
