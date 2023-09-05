from typing import Tuple
import numpy as np
import spatialmath as sm
import spatialmath.base as smb
import roboticstoolbox as rtb
import sympy as sym


# --------- Question 1.1 ---------- #


def is_twist(twist: np.ndarray) -> bool:
    """
    Check if a numpy array is a twist

    A valid twist has the shape (6,) with contents [v1, v2, v3, eta1, eta2, eta3]

    Parameters
    ----------
    twist
        The vector to check

    Returns
    -------
    bool
        True if the vector is a twist, False otherwise

    """

    if twist.shape != (6,):
        return False
    return True


# --------- Question 1.2 ---------- #


def SE3_to_twist(T: np.ndarray) -> np.ndarray:
    """
    Convert an SE3 matrix to a twist

    Parameters
    ----------
    T
        A (4, 4) numpy array representing an SE3 matrix

    Returns
    -------
    twist
        The twist representation of the SE3 as a numpy
        array: [v1, v2, v3, eta1, eta2, eta3]

    """

    return sm.Twist3(T).A


# --------- Question 1.3 ---------- #


def twist_to_SE3(twist: np.ndarray) -> np.ndarray:
    """
    Convert a twist to an SE3 matrix

    Parameters
    ----------
    twist
        The twist representation of the SE3 object as a numpy
        array: [v1, v2, v3, eta1, eta2, eta3]

    Returns
    -------
    T
        The SE3 matrix as a (4, 4) numpy array

    """
    twist: sm.Twist3 = sm.Twist3(twist)
    return twist.SE3().A


# --------- Question 2.1 ---------- #


def xyzrpy_traj(pose0: np.ndarray, pose1: np.ndarray, n: int) -> np.ndarray:
    """
    Compute the trajectory between two poses

    Computes the trajectory between two poses using a quintic polynomial
    profile taking n steps.

    Hint
    ----
    Use the `roboticstoolbox` method `mtraj` to compute the trajectory

    Parameters
    ----------
    pose0
        The start pose in the world frame as a numpy array: [x, y, z, roll, pitch, yaw]
        where the RPY angle is given in XYZ order
    pose1
        The end pose in the world frame as a numpy array: [x, y, z, roll, pitch, yaw]
        where the RPY angle is given in XYZ order
    n
        The number of steps in the trajectory

    Returns
    -------
    traj
        A numpy array of shape (n, 6) where each row is a pose in the world frame
        as a numpy array: [x, y, z, roll, pitch, yaw] where the RPY angle is given
        in XYZ order
    """
    return rtb.mtraj(rtb.quintic, pose0, pose1, n).q


# --------- Question 2.2 ---------- #


def xyzquat_traj(
    trans0: np.ndarray,
    quat0: sm.UnitQuaternion,
    trans1: np.ndarray,
    quat1: sm.UnitQuaternion,
    n: int,
) -> np.ndarray:
    """
    Compute the trajectory between two poses

    Computes the trajectory between two poses using a quintic polynomial
    profile taking n steps. Ensure that the orientation component of the
    trajectory is interpolated using the shortest path.

    Hint
    ----
    - Use the `roboticstoolbox` method `mtraj` to compute the translation component
      of the trajectory
    - Use the `UnitQuaternion` method `interp` to compute the orientation component
      of the trajectory

    Parameters
    ----------
    trans0
        The translation from the world frame to the origin of the start frame as a
        numpy array: [x, y, z]
    quat0
        The orientation of the start frame relative to the world frame as a
        `UnitQuaternion`
    trans1
        The translation from the world frame to the origin of the end frame as a
        numpy array: [x, y, z]
    quat1
        The orientation of the end frame relative to the world frame as a
        `UnitQuaternion`
    n
        The number of steps in the trajectory

    Returns
    -------
    traj
        A numpy array of shape (n, 7) where each row is a pose in the world frame
        as a numpy array: [x, y, z, w, v0, v1, v2] where [w, v0, v1, v2] is a unit
        quaternion
    """
    q0 = sm.UnitQuaternion(quat0)
    q1 = sm.UnitQuaternion(quat1)

    q_traj = q0.interp(q1, n, shortest=True)
    t_traj = rtb.mtraj(rtb.quintic, trans0, trans1, n)

    xyzquat_traj = np.concatenate((t_traj.q, q_traj.A), axis=1).reshape((n,7))
    return xyzquat_traj
    


# --------- Question 2.3 ---------- #


def SE3_xyzrpy_traj(wTa: sm.SE3, wTb: sm.SE3, n: int) -> sm.SE3:
    """
    Compute the trajectory between two poses

    Computes the trajectory between two poses using a quintic polynomial
    profile taking n steps.

    The trajectory is to be calculated using [x, y, z, roll, pitch, yaw] (where the
    RPY angles are in XYZ order) as the representation of the pose. The calculated
    trajectory is then converted back to an SE(3) representation and stored in a
    SE3 object.

    Parameters
    ----------
    wTa
        The start pose in the world frame
    wTb
        The end pose in the world frame
    n
        The number of steps in the trajectory

    Returns
    -------
    traj
        A SE3 object containing the trajectory

    """
    xyzrpy_0 = np.concatenate((wTa.t, wTa.rpy(order='xyz')))
    xyzrpy_f = np.concatenate((wTb.t, wTb.rpy(order='xyz')))

    xyzrpy_traj = rtb.mtraj(rtb.quintic, xyzrpy_0, xyzrpy_f, n)
    
    xyz_traj_q = xyzrpy_traj.q[:, :3]
    rpy_traj_q = xyzrpy_traj.q[:, 3:]

    SE3_traj = sm.SE3.Trans(xyz_traj_q) @ sm.SE3.RPY(rpy_traj_q, order='xyz')
    return SE3_traj 


# --------- Question 2.4 ---------- #


def SE3_twist_traj(wTa: sm.SE3, wTb: sm.SE3, n: int) -> sm.SE3:
    """
    Compute the trajectory between two poses

    Computes the trajectory between two poses using a quintic polynomial
    profile taking n steps.

    The trajectory is to be calculated using the twist representation of the pose.
    The calculated trajectory is then converted back to an SE(3) representation and
    stored in a SE3 object.

    Parameters
    ----------
    wTa
        The start pose in the world frame
    wTb
        The end pose in the world frame
    n
        The number of steps in the trajectory

    Returns
    -------
    traj
        A SE3 object containing the trajectory

    """

    tw_0 = wTa.twist()
    tw_f = wTb.twist()

    tw_traj = rtb.mtraj(rtb.quintic, tw_0.A, tw_f.A, n)
    traj = [sm.Twist3(traj).SE3() for traj in tw_traj.q]
    return traj



# --------- Question 2.5 ---------- #


def SE3_traj_relative(traj_world: sm.SE3) -> sm.SE3:
    """
    Converts a trajectory in the world frame to relative frames

    Given a trajectory in the world frame (each pose in the trajectory is
    relative to the world frame) this function converts the trajectory to
    a relative frame (each pose in the trajectory is relative to the previous
    pose in the trajectory).

    The first pose in the trajectory is ignored as it has no previous pose
    to be relative to. The returned trajectory will be of length n - 1 where
    n is the length of `traj_world`.

    Parameters
    ----------
    traj_world
        A SE3 object containing the trajectory in the world frame

    Returns
    -------
    traj_relative
        A SE3 object containing the trajectory in the relative frame

    """
    traj_relative = sm.SE3.Empty()
    prev_traj = traj_world[0]
    for traj in traj_world[1:]:
        new_traj = traj / prev_traj
        traj_relative.append(new_traj)
        prev_traj = traj
    # print("This should accept wTa.inv() @ wTb (or * with normalising) - documentation is unclear what '/' actually does")
    return traj_relative


# --------- Question 3.1 ---------- #


def quintic_properties_symbolic(
    coeffs: Tuple[float, float, float, float, float, float]
) -> Tuple[float, float]:
    """
    Extract the maximum acceleration and jerk from a quintic polynomial

    Given the coefficients of a quintic polynomial, this function extracts
    the maximum acceleration and jerk from the trajectory.

    Hint
    ----
    The Python package sympy may be helpful for this question.

    Parameters
    ----------
    coeffs
        The coefficients of the quintic polynomial in the form
        [A, B, C, D, E, F] where the polynomial is defined as
        q(t) = At^5 + Bt^4 + Ct^3 + Dt^2 + Et + F

    Returns
    -------
    maximums
        A Tuple containing the maximum absolute value of acceleration and
        jerk respectively

    """
    a,b,c,d,e,f = coeffs
    # Define the variables and polynomial as before
    x = sym.symbols('x', real=True)
    poly = sym.Poly(a*x**5 + b*x**4 + c*x**3 + d*x**2 + e*x + f, x)

    # Differentiate the polynomial to get the acceleration and jerk
    acc = poly.diff((x,2))
    jerk = poly.diff((x,3))
    snap = poly.diff ((x,4))

    jerk_roots = sym.solve(jerk, x)
    if jerk_roots:
        max_acc = max([abs(acc.subs(x, root).evalf()) for root in jerk_roots])
    else:
        max_acc = 0

    snap_roots = sym.solve(snap, x)
    if snap_roots:
        max_jerk = max([abs(jerk.subs(x, root).evalf()) for root in snap_roots])
    else:
        max_jerk = 0
    return (max_acc, max_jerk)


# --------- Question 4.1 ---------- #
def rocket_diagnostics(traj: sm.SE3, T: sm.SE3, tol=1e-4) -> float:
    """
    Find the time at which a rocket reaches a pose

    Given a trajectory `traj` for a rocket, this function finds the time at which
    the rocket reaches a given pose `T`. You can assume that the time between each
    pose in the trajectory is constant at 0.5 seconds. The trajectory is provided in
    relative form where each pose is relative to the previous pose. The first pose in
    the trajectory is relative to the world frame. The pose `T` is provided in the
    world frame.

    The maximum absolute tolerance of each element of `T` and the matched pose within
    `traj` is 1e-8.

    Parameters
    ----------
    traj
        A SE3 object containing the trajectory of the rocket
    T
        The pose to find the time for

    Returns
    -------
    time
        The time at which the rocket reaches the pose `T`

    """    
    world_pose = sm.SE3(np.eye(4))
    for index, pose in enumerate(traj.A):
        world_pose @= sm.SE3(pose)
        matches = np.isclose(world_pose, T.A, atol=tol, rtol=tol)
        if np.all(matches):
            break
    return index*0.5


# --------- Question 4.2 ---------- #
def rocket_pose(traj: sm.SE3, wTa: sm.SE3) -> Tuple[sm.SE3, float]:
    """
    Find aTg for a rocket given wTa and a trajectory and the time taken

    Given three coordinate frames

    - {w} is the world frame (the pose of the rocket at traj[0])
    - {a} is the frame of the rocket at some time during the trajectory

    Find aTg for a rocket given wTa and a trajectory and the time taken

    Given three coordinate frames

    - {w} is the world frame (the pose of the rocket at traj[0])
    - {a} is the frame of the rocket at some time during the trajectory
    - {g} is the goal frame (the pose of the rocket at traj[-1])

    The pose `wTa` in the world frame represents the frame {a} relative to {w}.
    This function finds the relative pose `aTg` and the time taken for the rocket
    to complete this segment of the trajectory. You can assume that the time
    between each pose in the trajectory is constant at 0.5 seconds. Each pose in
    the trajectory is given in the world frame. The pose `aTg` is the pose of the
    rocket at the goal {g} relative to {a}. `traj[0]` occurs at time 0.

    The maximum absolute tolerance of each element of `wTa` and the matched pose within
    `traj` is 1e-4.

    Parameters
    ----------
    traj
        A SE3 object containing the trajectory of the rocket in the world frame
    wTa
        The pose of {a} relative to {w}}

    Returns
    -------
    aTg
        The pose of the rocket at the goal {g} relative to {a}
    time
        The time taken for the rocket to complete the aTg segment of the
        trajectory

    """
    # np.set_printoptions(precision=6, floatmode='fixed', suppress=True, sign=' ')
    matches = np.isclose(traj.A, wTa, atol=1e-8).all(axis=(1,2))
    index = np.argmax(matches)
    aTg = wTa.inv() @ traj.pop()
    time_a = (index * 0.5)
    time_f = ((len(traj.A)) * 0.5)

    return aTg, (time_f-time_a)