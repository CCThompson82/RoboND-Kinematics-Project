from sympy import *
import numpy as np
import tf

class Solver(object):
    """

    """
    def __init__(self, dhp, ROT_EE, WC, EE):
        """

        Args
            dhp (parameters.ParamServer object)
            ROT_EE (sympy.matrices.Matrix)
            WC (sympy.matrices.Matrix): Matrix representing the wrist center
                position ([[x], [y], [z]])
            EE  (sympy.matrices.Matrix): Matrix representing the end effector
                position ([[x], [y], [z]])
        """
        self.WC = WC
        self.EE = EE
        self.dhp = dhp
        self.ROT_EE = ROT_EE

    def solve_IK(self):
        """
        Calculates IK solution(s) to the target pose setup during the object
        initiation
        """
        theta1 = self.solve_theta1()
        theta2, theta3 = self.solve_theta23()
        theta4, theta5, theta6 = self.solve_theta456(
            theta1=theta1, theta2=theta2, theta3=theta3)
        entry1 = [theta1, theta2, theta3, theta4, theta5, theta6]
        solution_set = [entry1]

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
        theta3 = float(pi/2 - (angle_b + 0.036)) #TODO: explicit sag angle calc
        return theta2, theta3


    def solve_theta456(self, theta1, theta2, theta3):
        """
        Calculates joint angles for the wrist given joint1, joint2, joint3.
        """
        R0_WC = self.dhp.T0_WC[:3, :3]

        R0_WC = R0_WC.evalf(subs={self.dhp.q1: theta1, self.dhp.q2: theta2,
                                  self.dhp.q3: theta3})

        Rwc_ee = R0_WC.transpose() * self.ROT_EE

        # NOTE: Why does this logic work?  Reference the walkthrough demo and
        # kind souls on the RoboND slack channel.  
        theta5 = atan2(sqrt((Rwc_ee[0, 2])**2 + (Rwc_ee[2, 2])**2), Rwc_ee[1, 2]) #NOTE: watch out for +/- of root term
        if sin(theta5) < 0:
            theta4 = atan2(-Rwc_ee[2, 2], Rwc_ee[0, 2])
            theta6 = atan2(Rwc_ee[1, 1], -Rwc_ee[1, 0])
        else:
            theta4 = atan2(Rwc_ee[2, 2], -Rwc_ee[0, 2])
            theta6 = atan2(-Rwc_ee[1, 1], Rwc_ee[1, 0])
        theta4, theta5, theta6 = float(theta4), float(theta5), float(theta6)


        ########################################################################
        # NOTE: This code section should work but does not produce valid answers
        # Reference: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.371.6578&rep=rep1&type=pdf
        #
        # if Rwc_ee[2, 0] != 1 or Rwc_ee[2, 0] != -1:
        #     b1 = -asin(Rwc_ee[2, 0])
        #     b2 = float(pi - b1)
        #     a1 = atan2(Rwc_ee[2,1]/cos(b1), Rwc_ee[2, 2]/cos(b1))
        #     a2 = atan2(Rwc_ee[2,1]/cos(b2), Rwc_ee[2, 2]/cos(b2))
        #     g1 = atan2(Rwc_ee[1,0]/cos(b1), Rwc_ee[0, 0]/cos(b1))
        #     g2 = atan2(Rwc_ee[1,0]/cos(b2), Rwc_ee[0, 0]/cos(b2))
        # else:
        #     g1, g2 = 0, 0
        #     if Rwc_ee[2,0] == -1:
        #         b1, b2 = pi/2, pi/2
        #         a1 = b1 + atan2(Rwc_ee[0,1], Rwc_ee[0,2])
        #         a1 = a2
        #     else:
        #         b1, b2 = -pi/2, -pi/2
        #         a1 = -b1 + atan2(-Rwc_ee[0,1], -Rwc_ee[0,2])
        ########################################################################


        return theta4, theta5, theta6
