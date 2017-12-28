from sympy import *
import utils

class ParamServer(object):
    """
    Object contains DH parameter table and sympy Symbols.
    """
    def __init__(self):
        (self.alpha0, self.alpha1, self.alpha2, self.alpha3, self.alpha4,
         self.alpha5, self.alpha6) = symbols('alpha0:7')
        (self.a0, self.a1, self.a2, self.a3, self.a4, self.a5,
         self.a6) = symbols('a0:7')
        (self.d1, self.d2, self.d3, self.d4, self.d5, self.d6,
         self.d7) = symbols('d1:8')
        (self.q1, self.q2, self.q3, self.q4, self.q5, self.q6,
         self.q7) = symbols('q1:8')

        self.DH = {self.alpha0: 0, self.a0: 0, self.d1: 0.75, self.q1: self.q1,
                   self.alpha1: -pi/2, self.a1: 0.35, self.d2: 0,
                   self.q2: self.q2-(pi/2),
                   self.alpha2: 0, self.a2: 1.25, self.d3: 0, self.q3: self.q3,
                   self.alpha3: -pi/2, self.a3: -0.054, self.d4: 1.501,
                   self.q4: self.q4,
                   self.alpha4: pi/2, self.a4: 0, self.d5: 0, self.q5: self.q5,
                   self.alpha5: -pi/2, self.a5: 0, self.d6: 0, self.q6: self.q6,
                   self.alpha6: 0, self.a6: 0, self.d7: 0.303, self.q7: 0}

    def generate_homegenous_transforms(self):
        """
        Returns the homegenous transforms for the wrist center and end
        effector.

        Returns
            T0_3 (sympy.matrices.Matrix)
            T0_EE (sympy.matrices.Matrix)
        """
        T01 = utils.make_TF(alpha=self.alpha0, a=self.a0, d=self.d1,
                            theta=self.q1).subs(self.DH)
        T12 = utils.make_TF(alpha=self.alpha1, a=self.a1, d=self.d2,
                            theta=self.q2).subs(self.DH)
        T23 = utils.make_TF(alpha=self.alpha2, a=self.a2, d=self.d3,
                            theta=self.q3).subs(self.DH)
        T34 = utils.make_TF(alpha=self.alpha3, a=self.a3, d=self.d4,
                            theta=self.q4).subs(self.DH)
        T45 = utils.make_TF(alpha=self.alpha4, a=self.a4, d=self.d5,
                            theta=self.q5).subs(self.DH)
        T56 = utils.make_TF(alpha=self.alpha5, a=self.a5, d=self.d6,
                            theta=self.q6).subs(self.DH)
        T6EE = utils.make_TF(alpha=self.alpha6, a=self.a6, d=self.d7,
                            theta=self.q7).subs(self.DH)

        T0_WC = (T01 * T12 * T23)
        T0_EE = (T0_WC * T34 * T45 * T56 * T6EE)

        return T0_WC, T0_EE