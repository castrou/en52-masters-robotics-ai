from typing import Literal as L, List
import numpy as np

import spatialmath as sm
import spatialmath.base as smb
import spatialmath.base.transforms2d as xfm2
import spatialmath.base.transforms3d as xfm3

from scipy.linalg import logm, expm

# ------------- Util -------------- #
# _eps = np.finfo(np.float64).eps

def is_zero(val, tol):
    return np.isclose(val, 0.0, rtol=tol, atol=tol)

def is_unit_vec(vec, tol):
    mag = np.linalg.norm(vec)
    if np.isclose(mag, 1.0, atol=tol, rtol=tol):
        return True
    return False

def is_null_rot(vec4_1):
    if vec4_1[0] == 1:
        if (vec4_1[1:] == 0).all():
            return True
    return False

def is_unit_quat(quat, tol):
    sq_sum = sum([val**2 for val in quat])
    mag = np.sqrt(sq_sum)
    return np.isclose(mag, 1.0, rtol=tol, atol=tol)

def pass_thru(rot):
    return rot

# --------- Question 1.1 ---------- #

def is_SO2(rot: np.ndarray, tol: float = 1e-6) -> bool:
    """
    Checks if the input matrix is a SO(2) matrix.

    This method checks if the input array is a valid SO(2) matrix,
    to the numerical threshold of `tol`. 'tol' is the numerical error
    tolerance for each key characteristic of a matrix in the Special
    Orthogonal Group and is calculated using the vector norm.


    Parameters
    ----------
    rot
        the input array to check

    Returns
    -------
    is_SO3
        True if the input array is a valid SO(2) matrix, False otherwise
    """
    if rot.shape != (2,2):
        return False
    # SO(2) inverse is the same as transpose
    if not np.allclose(np.linalg.inv(rot), rot.T, atol=tol):
        return False

    # Orthogonality
    if not np.allclose((rot @ rot.T), np.eye(2), atol=tol):
        return False

    # Orthogonal unit vectors -> 1 for right hand
    det_null = np.linalg.det(rot) - 1
    if (det_null > tol or det_null < -tol):
        return False

    return True


# --------- Question 1.2 ---------- #

def is_SO3(rot: np.ndarray, tol: float = 1e-6) -> bool:
    """
    Checks if the input matrix is a SO(3) matrix.

    This method checks if the input array is a valid SO(3) matrix,
    to the numerical threshold of `tol`. 'tol' is the numerical error
    tolerance for each key characteristic of a matrix in the Special
    Orthogonal Group and is calculated using the vector norm.


    Parameters
    ----------
    rot
        the input array to check

    Returns
    -------
    is_SO3
        True if the input array is a valid SO(3) matrix, False otherwise
    """
    # # tol' * _eps = tol, tol' = tol / _eps
    # return xfm3.isrot(rot, check=True, tol=(tol/(xfm3._eps)))
    if rot.shape != (3,3):
        return False
    
    # SO(3) inverse is the same as transpose
    if not np.allclose(np.linalg.inv(rot).all(), np.matrix.transpose(rot).all(), atol=tol, rtol=tol):
        return False
    
    # Orthogonality
    if not np.allclose((rot @ np.matrix.transpose(rot)), np.eye(3), atol=tol, rtol=tol):
        return False
    
    # Orthogonal unit vectors -> 1 for right hand
    det_null = np.linalg.det(rot) - 1
    if (det_null > tol or det_null < -tol):
        return False
    
    return True


# --------- Question 1.3 ---------- #

def RPY_to_SO3(rpy: np.ndarray) -> np.ndarray:
    """
    Converts ZYX roll-pitch-yaw angles to an SO(3) matrix.

    This method converts a set of RPY angles with ZYX ordering to
    an SO(3) matrix.

    Rotate by yaw about the z-axis, then by pitch about the
    new y-axis, then by roll about the new x-axis.

    Parameters
    ----------
    rpy
        a (3, 1) array of with the order [roll, pitch, yaw]

    Returns
    -------
    rot
        the corresponding SO(3) matrix

    """
    rotate = [sm.SO3.Rx, sm.SO3.Ry, sm.SO3.Rz]
    order = [2,1,0]

    rot = sm.SO3.Rz(0)
    for i in order:
        rot *= rotate[i](rpy[i])
    
    return rot
           

# --------- Question 2.1 ---------- #

def is_eulvec(eulvec: np.ndarray) -> bool:
    """
    Checks if the input array is a valid Euler vector.

    This method checks if the input array is a valid Euler vector, where
    the Euler vector must be a (3, 1) numpy array.

    Parameters
    ----------
    eulvec
        the input array to check

    Returns
    -------
    is_eulvec
        True if the input array is a valid Euler vector, False otherwise
    """

    if eulvec.shape != (3,1):
        return False
    if not np.isreal(eulvec).all():
        return False
    return True


# --------- Question 2.2 ---------- #

def is_angax(angax: np.ndarray, tol=1e-6) -> bool:
    """
    Checks if the input array is a valid angle-axis vector.

    This method checks if the input array is a valid angle-axis vector,
    where the angle-axis vector must be a (4, 1) numpy array:

    [
        [theta],
        [etahat_1],
        [etahat_2],
        [etahat_3]
    ]

    where theta is the rotation angle and etahat is the rotation axis. The
    error threshold for the vector norm of the rotation axis is given by
    `tol`.

    Parameters
    ----------
    angax
        the input array to check
    tol
        the error threshold for the norm of the rotation axis

    Returns
    -------
    is_angax
        True if the input array is a valid angle-axis vector, False otherwise
    """
    # Shape
    if angax.shape != (4,1):
        return False
    
    angle = angax[0]
    axis = angax[1:,0]

    # Norm check
    if not is_unit_vec(axis, tol):
        if is_null_rot(angax):
            return True
        return False
    
    return True


# --------- Question 2.3 ---------- #

def angax_to_eulvec(angax: np.ndarray) -> np.ndarray:
    """
    Converts an angle-axis vector to an Euler vector.

    This method converts an angle-axis vector to an Euler vector.

    Parameters
    ----------
    angax
        the input angle-axis vector as a (4, 1) numpy array as described by
        the `is_angax` method

    Returns
    -------
    eulvec
        the corresponding Euler vector as a (3, 1) numpy array
    """

    return angax[0] * angax[1:]

# angax = np.array([[1.0], [1.0], [0], [0]])
# angax_to_eulvec(angax)

# --------- Question 2.4 ---------- #

def eulvec_to_SO3(eulvec: np.ndarray) -> np.ndarray:
    """
    Converts a Euler vector to a SO(3) matrix.

    This method converts a Euler vector to a SO(3) matrix.

    Parameters
    ----------
    eulvec
        the input Euler vector as a (3, 1) numpy array as described by the
        `is_eulvec` method

    Returns
    -------
    rot
        the corresponding SO(3) matrix as a (3, 3) numpy array
    """
    rot = smb.exp2r(eulvec)    
    return np.asarray(rot)
    

# --------- Question 3.1 ---------- #

def is_quat(quat: np.ndarray, tol=1e-6) -> bool:
    """
    Checks if the input array is a valid unit quaternion.

    This method checks if the input array is a valid unit quaternion,
    where the unit quaternion must be a (4, 1) numpy array:

    [
        [s],
        [v_x],
        [v_y],
        [v_z]
    ]

    where s is the real component and v_x, v_y, and v_z are the imaginary
    components. The error threshold for the vector norm of the quaternion is
    given by `tol`.

    Parameters
    ----------
    quat
        the input array to check as a (4, 1) numpy array
    tol
        the error threshold for the norm of the quaternion

    Returns
    -------
    is_quat
        True if the input array is a valid quaternion, False otherwise
    """
    if quat.shape != (4,1):
        return False
    
    if not is_unit_quat(quat, tol):
        return False
    return True


# --------- Question 3.2 ---------- #

def quat_to_SO3(quat: np.ndarray) -> np.ndarray:
    """
    Converts a quaternion to a SO(3) matrix.

    This method converts a quaternion to a SO(3) matrix where the quaternion is
    given as described by `is_quat`.

    Parameters
    ----------
    quat
        the input quaternion as a (4, 1) numpy array

    Returns
    -------
    rot
        the corresponding SO(3) matrix as a (3, 3) numpy array
    """
    q = sm.UnitQuaternion(quat[:,0])
    return np.asarray(q.SO3())


# --------- Question 3.3 ---------- #

def SO3_to_quat(rot: np.ndarray) -> np.ndarray:
    """
    Converts a SO(3) matrix to a quaternion.

    This method converts a SO(3) matrix to a quaternion.

    Parameters
    ----------
    rot
        the input SO(3) matrix as a (3, 3) numpy array

    Returns
    -------
    quat
        the corresponding quaternion as a (4, 1) numpy array as described by
        the `is_quat` method
    """
    return smb.r2q(rot, check=False).reshape((4,1))

# --------- Question 4.1 ---------- #

def orientation_type(arg: np.ndarray, tol=1e-6) -> L["quat", "eulvec", "SO3"]:
    """
    Tests if the input is a valid 3D orientation representation

    This method tests if the input array `arg` is either a valid
    quaternion, Euler vector, or SO(3) matrix.

    Parameters
    ----------
    arg
        The input array to test

    Returns
    -------
    type
        A string indicating the type of orientation representation, either
        'quat', 'eulvec', or 'SO3'

    Raises
    ------
    ValueError
        if the input array is not a valid quaternion, Euler vector,
        or SO(3) matrix

    """

    if is_eulvec(arg):
        return 'eulvec'
    elif is_quat(arg, tol):
        return 'quat'
    elif is_SO3(arg, tol):
        return 'SO3'
    else:
        raise ValueError


# --------- Question 4.2 ---------- #

def SO3_to_qv(so3):
    q = SO3_to_quat(so3)
    return smb.q2v(q)

def eulvec_to_quat(eulvec):
    R = eulvec_to_SO3(eulvec)
    return SO3_to_quat(R)

def filter_orientations(rots: List[np.ndarray], tol=1e-6) -> List[np.ndarray]:
    """
    Filters a list of orientations and removes errors

    This method converts a list of orientations where each orientation
    may be a quaternion, Euler vector, SO(3) matrix, or an error. The
    method returns a list of valid orientations as a consistent type
    where the type may be a quaternion, Euler vector, or SO(3) matrix as
    defined by the `is_SO3`, `is_quat`, and `is_eulvec` methods but the
    choice is up to you.

    Parameters
    ----------
    rots
        A list of orientations where each orientation may be a quaternion,
        Euler vector, SO(3) matrix, or an error

    Returns
    -------
    rots
        A list of numpy arrays where each array is a valid orientation as a
        consistent type (errors should be removed)
    """
    SO3_unify = {'SO3': pass_thru, 'quat': quat_to_SO3, 'eulvec': eulvec_to_SO3}
    quat_unify = {'SO3': SO3_to_quat, 'quat': pass_thru, 'eulvec': eulvec_to_quat}

    unify = SO3_unify

    new_rots = []
    for rot in rots:
        try:
            o_type = orientation_type(rot)
        except ValueError:
            continue
        new_rots.append(np.asarray(unify[o_type](rot)))
    return new_rots

# --------- Question 4.3 ---------- #

def get_planet_orienations(rots: List[np.ndarray]) -> List[np.ndarray]:
    """
    Gets planet orientations realtive to Earth

    Given a list of converted planet orientations, as output from the
    `filter_orientations` method, this method returns the planet
    orientations relative to Earth (as the same orientation type as output by
    `filter_orientations`).

    Parameters
    ----------
    rots
        A list of planet orientations as output by `filter_orientations`

    Returns
    -------
    planet_rots
        A list of planet orientations relative to Earth where the orientation type
        is the same as the input `rots`
    """
    # # SO3 Implementation
    planet_rots = []
    planet_rots.append(rots[0])
    [planet_rots.append(planet_rots[-1] @ rot) for rot in rots[1:]]
    return planet_rots

    # Quaternion time
    # planet_rots = []
    # planet_rots.append(rots[0])
    # for rot in rots[1:]:
    #         new_rot = (sm.UnitQuaternion(planet_rots[-1]) * sm.UnitQuaternion(rot)).A
    #         planet_rots.append(new_rot)
    # return planet_rots