from sympy import *
import numpy as np
import tf

class Solver(object):
    """

    """
    def __init__(self, dhp):
        """

        Args
            dhp (parameters.ParamServer object)
            ROT_EE (sympy.matrices.Matrix)
            WC (sympy.matrices.Matrix): Matrix representing the wrist center
                position ([[x], [y], [z]])
            EE  (sympy.matrices.Matrix): Matrix representing the end effector
                position ([[x], [y], [z]])
        """
        self.dhp = dhp
        self.T_target = None

    def solve_IK(self, EExyz, EErpy):
        """
        Calculates IK solution(s) to the target pose setup during the object
        initiation
        """
        self.EExyz = Matrix([[EExyz[0]], [EExyz[1]], [EExyz[2]]])

        self.Rtarget_0EE = self.dhp.sym_R_target.subs({self.dhp.r: EErpy[0],
                                                       self.dhp.p: EErpy[1],
                                                       self.dhp.y: EErpy[2]})

        # NOTE: The z-frame of Rtarget describes the u,v,w vector from WC to EE.
        # The WC position is the translation backwords along the z-frame of
        # Rtarget for the length of the gripper

        self.WC = self.EExyz - (self.dhp.DH[self.dhp.d7]*self.Rtarget_0EE[:, 2])

        theta1 = self.solve_theta1()
        theta2, theta3 = self.solve_theta23()
        wrist_solution_sets = self.solve_theta456(theta1=theta1, theta2=theta2, theta3=theta3)
        entry1 = [theta1, theta2, theta3] + wrist_solution_sets[0]
        entry2 = [theta1, theta2, theta3] + wrist_solution_sets[1]
        entry3 = [theta1, theta2, theta3] + wrist_solution_sets[2]
        print(entry1)
        print(entry2)
        print(entry3)
        solution_set = [entry2]

        # TODO: calculate further solutions to the target IK problem and append to solution_set

        return solution_set


    def solve_theta1(self):
        """
        Calculates the theta1 angle using Solver attr
        """
        theta1 = float(atan2(self.WC[1], self.WC[0]))
        return theta1

    def solve_theta23(self):
        """
        Calculates the theta2 angle given theta1 or class attr
        """
        A = self.dhp.DH[self.dhp.d4]
        C = self.dhp.DH[self.dhp.a2]

        Bz = self.WC[2] - self.dhp.DH[self.dhp.d1]
        Bx = sqrt((self.WC[0])**2 + (self.WC[1])**2) - self.dhp.DH[self.dhp.a1]

        B = sqrt((Bx)**2 + (Bz)**2)

        angle_a = acos(((B)**2 + (C)**2 - (A)**2) / (2*B*C))
        angle_b = acos(((A)**2 + (C)**2 - (B)**2) / (2*A*C))
        angle_c = acos(((A)**2 + (B)**2 - (C)**2) / (2*A*B))

        theta2 = ((pi/2) - angle_a - atan2(Bz, Bx)).evalf()
        m = atan2(self.dhp.DH[self.dhp.a3], self.dhp.DH[self.dhp.d4])

        theta3 = (pi/2 - (angle_b + abs(m)))
        return theta2, theta3


    def solve_theta456(self, theta1, theta2, theta3):
        """
        Calculates joint angles for the wrist given joint1, joint2, joint3.
        """

        # obtain the rotation matrix composed from the base link to link 3 transformations
        R0_WC = self.dhp.T0_WC[:3, :3]
        # evaluate the rotation matrix given the joint angle args
        R0_WC = R0_WC.evalf(subs={self.dhp.q1: theta1, self.dhp.q2: theta2,
                                  self.dhp.q3: theta3})

        # what is this going on here??

        Rt = R0_WC.transpose() * self.Rtarget_0EE # NOTE: This is the evaluated matrix, (WC-->EE)Rtarget in the report.

        ########################################################################
        # NOTE: demo plus slack channel
        theta5 = atan2(sqrt((Rt[0, 2])**2 + (Rt[2, 2])**2), Rt[1, 2]) #NOTE: watch out for +/- of root term
        if sin(theta5) < 0:
            theta4 = atan2(Rt[2, 2], -Rt[0, 2])
            theta6 = atan2(-Rt[1, 1], Rt[1, 0])
        else:
            theta4 = atan2(Rt[2, 2], -Rt[0, 2])
            theta6 = atan2(-Rt[1, 1], Rt[1, 0])
        theta4, theta5, theta6 = float(theta4), float(theta5), float(theta6)
        ########################################################################

        ########################################################################
        # NOTE: This code section should work but does not produce valid answers
        # Reference: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.371.6578&rep=rep1&type=pdf

        if Rt[1, 2] != 1 or Rt[1, 2] != -1:
            theta5_1 = acos(Rt[1, 2])
            theta5_2 = -1 * theta5_1 #NOTE: cos(theta) = cos(-theta)

            if cos(theta5_1) > 0:
                theta4_1 = atan2(Rt[2,2], -Rt[0, 2])
                theta4_2 = atan2(-Rt[2,2], Rt[0, 2])
                theta6_1 = atan2(Rt[1, 1], -Rt[1, 0])
                theta6_2 = atan2(-Rt[1, 1], Rt[1, 0])
            else:
                theta4_2 = atan2(Rt[2,2], -Rt[0, 2])
                theta4_1 = atan2(-Rt[2,2], Rt[0, 2])
                theta6_2 = atan2(Rt[1, 1], -Rt[1, 0])
                theta6_1 = atan2(-Rt[1, 1], Rt[1, 0])

        else:
            #TODO!!!!!!!!!!!!!!!!!!!!!
            theta6_1, theta6_2 = 0, 0 # can be anything, but zero is convenient
            if Rt[2,0] == -1:
                theta5_1, theta5_2 = pi/2, pi/2
                theta4_1 = theta5_1 + atan2(Rt[0,1], Rt[0,2])
                theta4_2 = theta4_1
            else:
                theta5_1, theta5_2 = -pi/2, -pi/2
                theta4_1 = -theta5_1 + atan2(-Rt[0,1], -Rt[0,2])
                theta4_2 = theta4_1
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        ########################################################################


        return [[theta4, theta5, theta6], [theta4_1, theta5_1, theta6_1], [theta4_2, theta5_2, theta6_2]]
























    #
