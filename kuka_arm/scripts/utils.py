from sympy import *
from sympy.matrices import Matrix

alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = symbols('alpha1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')
a0, a1, a2, a3, a4, a5 = symbols('a1:7')
d1, d2, d3, d4, d5, d6, dG = symbols('d1:8')

r, p, y = symbols('r p y')


def make_TF(alpha, a, d, theta):
    """
    Generates a homogenous transformation matrix given sympy variables
    representing DH parameters.

    Args
        alpha (sympy.core.symbol.Symbol): twist angle
        a (sympy.core.symbol.Symbol): link length
        d (sympy.core.symbol.Symbol): offset length
        theta (sympy.core.symbol.Symbol): joint angle

    Returns
        homegenous transform matrix (sympy.matrices.Matrix)
    """
    T = Matrix([[cos(theta), -sin(theta), 0, a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha),
                 -sin(alpha), -sin(alpha)*d],
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha),
                 cos(alpha), cos(alpha)*d],
                [0, 0, 0, 1]])
    return T

def rotate_x(sbl):
    """
    Generates a rotation matrix containing a symbol for the theta angle

    Arg
        symbol (sympy.symbol)
    Returns
        RotMat (sympy.matrices.Matrix)
    """
    rot_X = Matrix([[1, 0, 0],
                    [0, cos(sbl), -sin(sbl)],
                    [0, sin(sbl), cos(sbl)]])
    return rot_X

def rotate_y(sbl):
    """
    Generates a rotation matrix containing a symbol for the theta angle

    Arg
        symbol (sympy.symbol)
    Returns
        RotMat (sympy.matrices.Matrix)
    """
    rot_Y = Matrix([[cos(sbl), 0, sin(sbl)],
                    [0, 1, 0],
                    [-sin(sbl), 0, cos(sbl)]])
    return rot_Y

def rotate_z(sbl):
    """
    Generates a rotation matrix containing a symbol for the theta angle

    Arg
        symbol (sympy.symbol)
    Returns
        RotMat (sympy.matrices.Matrix)
    """
    rot_Z =  Matrix([[cos(sbl), -sin(sbl), 0],
                    [sin(sbl), cos(sbl), 0],
                    [0, 0, 1]])

    return rot_Z






























#
