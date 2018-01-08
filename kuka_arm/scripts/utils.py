from sympy import *
from sympy.matrices import Matrix

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

def generate_sym_Rtarget(r, p, y):
    """
    Generates a symbolic rotation matrix that incorporates the roll, pitch,
    yaw variables to describe the target EE orientation.

    Args
        r
        p
        y

    Returns
        symbolic matrix
    """
    rot_x = rotate_x(r)
    rot_y = rotate_y(p)
    rot_z = rotate_z(y)

    rot_zyx = rot_z * rot_y * rot_x
    r_correction = (rot_z * rot_y).subs({y:pi, p:-pi/2})

    sym_R_target = rot_zyx * r_correction

    return sym_R_target






























#
