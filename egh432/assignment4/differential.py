from typing import Literal as L, List, Tuple
import numpy as np
import spatialmath as sm
import spatialmath.base as smb
import roboticstoolbox as rtb
import sympy as sym
from scipy.linalg import expm
import os


# --------- Question 1.1 ---------- #


def rotation_dot(theta: float, axis: np.ndarray, thetad: float) -> np.ndarray:
    """
    Find the derivative of a rotation matrix

    Given an angle axis representation of a 3D rotation, and the rate of
    change of that angle, calculate the derivative of the rotation in the
    form of a 3x3 matrix (the derivative of an SO(3) matrix).

    You can assume that the axis of rotation does not change with time.

    Parameters
    ----------
    theta
        The angle of rotation
    axis
        The axis of rotation as a 3-vector
    thetad
        The rate of change of theta

    Returns
    -------
    Rd
        The derivative of the rotation matrix as a 3x3 matrix

    """
    eulvec = axis*theta
    rot = smb.exp2r(eulvec)
    w = axis*thetad
    Rd = smb.skew(w) @ rot
    return Rd


# --------- Question 1.2 ---------- #


def angular_vel(R: np.ndarray, Rd: np.ndarray) -> np.ndarray:
    """
    Calculate the angular velocity from a rotation matrix and its derivative

    Given a rotation matrix and its derivative, calculate the angular
    velocity vector ω

    Parameters
    ----------
    R
        A 3x3 SO(3) rotation matrix
    Rd
        The derivative of R

    Returns
    -------
    omega
        The angular velocity vector as a (3,) numpy array

    """

    return smb.vex(Rd @ R.transpose())


# --------- Question 1.3 ---------- #


def transform_dot(
    theta: float, axis: np.ndarray, thetad: float, td: np.ndarray
) -> np.ndarray:
    """
    Calculate the derivative of a homogeneous transformation matrix

    Given the derivative of the rotation angle,
    and the derivative of the translation vector, calculate the derivative of
    the homogeneous transform matrix.

    Parameters
    ----------
    theta
        The rotation angle
    axis
        The axis of rotation as a (3,) numpy array
    thetad
        The derivative of the rotation angle
    td
        The derivative of the translation vector as a (3,) numpy array

    Returns
    -------
    Tdot
        The derivative of the homogeneous transformation matrix as a
        (4,4) numpy array

    """
    Rd = rotation_dot(theta, axis, thetad)
    return smb.Ab2M(Rd, td)


# --------- Question 1.4 ---------- #


def spatial_velocity(T: np.ndarray, Td: np.ndarray) -> np.ndarray:
    """
    Calculate the spatial velocity from a homogeneous transform matrix

    Given a homogeneous transform matrix and its derivative, calculate the
    spatial velocity vector

    Parameters
    ----------
    T
        A 4x4 SE(3) homogeneous transformation matrix
    Td
        The derivative of T

    Returns
    -------
    v
        The spatial velocity vector as a (6,) numpy array, where the first
        three values are linear velocity and the final three values are
        the angular velocity [v; ω]

    """
    
    Rd = Td[:3,:3]
    td = Td[:3,3]

    w = angular_vel(T[:3,:3], Rd)
    v = np.stack([td, w]).reshape(6,)
    return v

# --------- Question 2.1 ---------- #


def transform_velocity(wv: np.ndarray, wTa: np.ndarray) -> np.ndarray:
    """
    Transform a spatial velocity to a different reference frame

    Given a spatial velocity in the world frame, and a homogeneous transform
    matrix that describes the relationship between the world frame and a
    different frame a, transform the spatial velocity to be relative new frame.

    Parameters
    ----------
    wv
        The spatial velocity vector in the world frame as a (6,) numpy array
    wTa
        The homogeneous transformation matrix that describes the relationship
        between the world frame and the new frame a

    Returns
    -------
    av
        The spatial velocity vector relative to {a} as a (6,) numpy array

    """
    return np.linalg.pinv(sm.SE3(wTa).jacob()) @ wv


# --------- Question 3.1 ---------- #


def rotation_partial_diff(theta: float, axis: np.ndarray) -> np.ndarray:
    """
    Calculate the partial derivative of a rotation matrix

    Given an angle axis representation of a 3D rotation, first convert to a
    rotation matrix R, then calculate the partial derivative of the rotation
    with respect to the rotation angle theta:

    partial R / partial theta

    Parameters
    ----------
    theta
        The angle of rotation
    axis
        The axis of rotation as a (3,) numpy array

    Returns
    -------
    Rpd
        The partial derivative of the rotation matrix as a (3,3) numpy array

    """
    eulvec = axis*theta
    rot = smb.exp2r(eulvec)
    Rpd = smb.skew(axis) @ rot
    return Rpd


# --------- Question 4.1 ---------- #


def pose_graph_transform_velocity(
    av: np.ndarray, pose_graph: sm.SE3, a: int, b: int
) -> np.ndarray:
    """
    Transform a spatial velocity between two frames in a pose graph

    Given a spatial velocity in the `a` frame, and a list of SE3
    matrices, transform the spatial velocity from frame `a` to frame `b`.
    `a` will always be less than `b`.

    Parameters
    ----------
    av
        The spatial velocity vector in the `a` frame as a (6,) numpy array
    pose_graph
        An SE3 object containing a list matrices that describe the pose graph.
        The first matrix (pose_graph[0]) is relative to the world frame, and
        each subsequent SE(3) is relative to the previous SE(3) in the list.
    a
        Corresponds to an index in the pose_graph list
    b
        Corresponds to an index in the pose_graph list

    Returns
    -------
    bv
        The spatial velocity vector relative to {b} as a (6,) numpy array

    """
    world_pose = sm.SE3(np.eye(4))
    for index, pose in enumerate(pose_graph):
        world_pose @= sm.SE3(pose)
        if index == a:
            wTa = world_pose
        elif index == b:
            wTb = world_pose
            break

    # wTa = pose_graph[:a].prod()
    # wTb = pose_graph[:b].prod()

    wv = sm.SE3(wTa).jacob() @ av
    bv = np.linalg.pinv(sm.SE3(wTb).jacob()) @ wv
    return bv