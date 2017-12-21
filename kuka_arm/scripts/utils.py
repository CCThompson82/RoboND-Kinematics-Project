from sympy import symbols, cos, sin, simplify
import numpy as np
from sympy.matrices import Matrix




def make_T(alpha, a, d, theta):
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

def compute_fk(joints):
    """
    Computes the homogenous transform matrix for the end effector of the KUKU210
    robot, given a joint state.

    Args
        joints (list): [joint1, ..., joint6] of joint angles in radians

    Returns
        homegenous transform matrix (sympy.matrices.Matrix)
    """
    # initiate sympy symbols
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = symbols('alpha1:7')
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')
    a0, a1, a2, a3, a4, a5 = symbols('a1:7')
    d1, d2, d3, d4, d5, d6, dG = symbols('d1:8')

    # define DH parameters
    DH_table = {'joint1': {'alpha_i': alpha0, 'a_i': a0, 'd_i': d1, 'q_i': q1},
                'joint2': {'alpha_i': alpha1, 'a_i': a1, 'd_i': d2, 'q_i': q2},
                'joint3': {'alpha_i': alpha2, 'a_i': a2, 'd_i': d3, 'q_i': q3},
                'joint4': {'alpha_i': alpha3, 'a_i': a3, 'd_i': d4, 'q_i': q4},
                'joint5': {'alpha_i': alpha4, 'a_i': a4, 'd_i': d5, 'q_i': q5},
                'joint6': {'alpha_i': alpha5, 'a_i': a5, 'd_i': d6, 'q_i': q6},
                'grip':   {'alpha_i': 0, 'a_i': 0, 'd_i': dG, 'q_i': 0}}
    # NOTE: constants substitution dictionary created from kr210.urdf.xacro
    # values with respect to my own DH schema of the robot model.
    # TODO: Link to robot schema
    s = {alpha0: 0, a0: 0, d1: 0.33+0.42,
         alpha1: -np.pi/2, a1: 0.35, d2: 0, q2: q2-(np.pi/2),
         alpha2: 0, a2: 1.25, d3: 0,
         alpha3: -np.pi/2, a3: -0.054, d4: 0.96,
         alpha4: np.pi/2, a4: 0, d5: 0,
         alpha5: -np.pi/2, a5: 0, d6: 0.54+0.193,
         dG: 0.11}

    DH_dict = DH_table.copy()
    DH_dict['grip']['d_i'] = s[dG]
    for joint in [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']:
        for key in ['alpha_i', 'a_i', 'd_i', 'q_i']:
            try:
                DH_dict[joint][key] = s[DH_table[joint][key]]
            except KeyError:
                # thetas except for q2 will trip here as they are not defined in the s dictionary
                pass

    # setup individual transform matrices
    T0_1 = make_T(
        *[DH_dict['joint1'][key] for key in ['alpha_i', 'a_i', 'd_i', 'q_i']])
    T1_2 = make_T(
        *[DH_dict['joint2'][key] for key in ['alpha_i', 'a_i', 'd_i', 'q_i']])
    T2_3 = make_T(
        *[DH_dict['joint3'][key] for key in ['alpha_i', 'a_i', 'd_i', 'q_i']])
    T3_4 = make_T(
        *[DH_dict['joint4'][key] for key in ['alpha_i', 'a_i', 'd_i', 'q_i']])
    T4_5 = make_T(
        *[DH_dict['joint5'][key] for key in ['alpha_i', 'a_i', 'd_i', 'q_i']])
    T5_6 = make_T(
        *[DH_dict['joint6'][key] for key in ['alpha_i', 'a_i', 'd_i', 'q_i']])
    T6_G = make_T(
        *[DH_dict['grip'][key] for key in ['alpha_i', 'a_i', 'd_i', 'q_i']])


    # compose individual transforms
    T0_G = ((((((T0_1 * T1_2) * T2_3) * T3_4) * T4_5) * T5_6) * T6_G)

    # correct for gripper frame definition misalignment
    R_z = Matrix([[cos(np.pi), -sin(np.pi), 0 , 0],
              [sin(np.pi), cos(np.pi), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
    R_y = Matrix([[cos(-np.pi/2), 0, sin(-np.pi/2), 0],
              [0, 1, 0, 0],
              [-sin(-np.pi/2), 0, cos(-np.pi/2), 0],
              [0, 0, 0, 1]])
    R_corr = R_z * R_y
    Ttotal = T0_G * R_corr

    # evaluate transformation
    joint_dict = {k:v for k, v in zip([q1, q2, q3, q4, q5, q6], joints)}
    R = Ttotal.subs(joint_dict)

    return R
