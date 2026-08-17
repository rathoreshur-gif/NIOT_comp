import numpy as np
from scipy.spatial.transform import Rotation as R


def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion [x, y, z, w] to ZYX Euler angles in degrees.

    For display and localization publishing only — do NOT use for PID error
    computation (use compute_pid_forces which handles attitude error via
    rotation vector to avoid gimbal-lock coupling).

    Returns:
        (roll_deg, pitch_deg, yaw_deg)
    """
    rot = R.from_quat([x, y, z, w])
    yaw, pitch, roll = rot.as_euler('zyx', degrees=True)
    return roll, pitch, yaw


def wrap(angle):
    """Wrap an angle in degrees to the range [-180, +180)."""
    return (angle + 180) % 360 - 180


def transform_pose_from_gazebo(gazebo_pos_xyz, gazebo_quat_xyzw):
    """
    Convert a raw Gazebo pose into AUV-convention position and orientation.

    Axis conventions applied:
      Position:
        pos_x_auv =  gazebo_pos.x   (same)
        pos_y_auv = -gazebo_pos.y   (AUV +Y = Gazebo -Y)
        pos_z_auv = -gazebo_pos.z   (AUV +Z = depth-down = Gazebo -Z up)

      Orientation (ZYX Euler decomposed from quaternion):
        roll_auv  =  roll_gazebo    (same sign)
        pitch_auv = -pitch_gazebo   (AUV +pitch = Gazebo -pitch)
        yaw_auv   = -yaw_gazebo     (AUV +yaw   = Gazebo -yaw)

    The returned raw_quat is the UNCHANGED Gazebo quaternion.  It must be
    stored and used for all body<->world frame rotations so that rot.apply()
    works correctly against Gazebo-convention vectors.

    Parameters:
        gazebo_pos_xyz    : (x, y, z) position in Gazebo world frame (metres)
        gazebo_quat_xyzw  : (x, y, z, w) quaternion in Gazebo convention

    Returns:
        auv_pos    : (x, y, z) in AUV convention (metres)
        auv_euler  : (roll, pitch, yaw) in AUV convention (degrees)
        raw_quat   : (x, y, z, w) Gazebo quaternion, unchanged
    """
    gx, gy, gz = gazebo_pos_xyz
    auv_pos = (gx, -gy, -gz)

    roll, pitch, yaw = quaternion_to_euler(*gazebo_quat_xyzw)
    auv_euler = (roll, -pitch, -yaw)

    return auv_pos, auv_euler, tuple(gazebo_quat_xyzw)


def transform_odometry(body_linear_vel_xyz, body_angular_vel_xyz, current_quat_xyzw):
    """
    Convert Gazebo body-frame odometry into AUV-convention velocities.

    Linear velocity:
      1. Rotate body-frame → world frame using current Gazebo quaternion.
      2. Apply AUV axis flips: negate Y and Z.

    Angular velocity:
      Stays in body frame (keeps the rotational PID derivative term aligned
      with the vehicle's own axes — rotating to world frame would couple yaw
      rate into the roll/pitch derivative terms).
      Only convention flips are applied: negate Y and Z.

    Parameters:
        body_linear_vel_xyz   : (vx, vy, vz) in Gazebo body frame (m/s)
        body_angular_vel_xyz  : (wx, wy, wz) in Gazebo body frame (rad/s or deg/s)
        current_quat_xyzw     : (x, y, z, w) Gazebo-convention quaternion

    Returns:
        world_vel   : (vx, vy, vz) in AUV world frame (same units as input)
        auv_angular : (wx, wy, wz) in AUV body frame (same units as input)
    """
    rot = R.from_quat(current_quat_xyzw)

    v_body = np.array(body_linear_vel_xyz)
    v_world = rot.apply(v_body)
    world_vel = (v_world[0], -v_world[1], -v_world[2])

    wx, wy, wz = body_angular_vel_xyz
    auv_angular = (wx, -wy, -wz)

    return world_vel, auv_angular


def compute_pid_forces(
    position_xyz,
    setpoint_xyz,
    setpoint_euler_rpy,
    current_quat_xyzw,
    velocity_xyz,
    angular_velocity_xyz,
    pid_constants,
    integral,
    time_step,
):
    """
    Compute 6-DOF PID forces and update the integral accumulator.

    Translational DOF (surge=0, sway=1, heave=2):
        error     = setpoint - position
        integral += error * time_step  (clamped by ki_cap when below ki_error_threshold)
        force     = Kp*error + Kd*(0 - velocity) + Ki*integral
        force     = clip(force, maxBack, maxFront)

    Rotational DOF (roll=3, pitch=4, yaw=5):
        Uses quaternion attitude error to avoid the ZYX gimbal-lock coupling
        that causes roll/pitch cross-contamination during yaw manoeuvres.

        q_setpoint is reconstructed from setpoint_euler_rpy in AUV convention,
        then sign-flipped to Gazebo convention before building the quaternion
        (negate pitch and yaw) so it is consistent with current_quat (also
        Gazebo convention).

        q_err = q_current.inv() * q_setpoint   (error in current body frame)
        err_rotvec = q_err.as_rotvec(degrees=True)
        roll_error  =  err_rotvec[0]
        pitch_error = -err_rotvec[1]
        yaw_error   = -err_rotvec[2]

    Parameters:
        position_xyz         : (x, y, z) current AUV position in cm
        setpoint_xyz         : (x, y, z) desired AUV position in cm
        setpoint_euler_rpy   : (roll, pitch, yaw) desired orientation in degrees, AUV convention
        current_quat_xyzw    : (x, y, z, w) Gazebo-convention quaternion
        velocity_xyz         : (vx, vy, vz) AUV world-frame velocity in cm/s
        angular_velocity_xyz : (wx, wy, wz) AUV body-frame angular velocity in rad/s
                               (converted internally to deg/s to match error units)
        pid_constants        : dict keyed by DOF name, each with Kp, Ki, Kd,
                               maxBack, maxFront, ki_cap, ki_error_threshold
        integral             : np.ndarray shape (6,), updated in-place
        time_step            : elapsed seconds since last call

    Returns:
        (forces_6dof, integral)
        forces_6dof : np.ndarray shape (6,) — [surge, sway, heave, roll, pitch, yaw]
        integral    : same object as the input, returned for explicitness
    """
    forces = np.zeros(6)

    # --- Translational PID (indices 0, 1, 2) ---
    trans_dofs = ['surge', 'sway', 'heave']
    pos = np.array(position_xyz)
    sp  = np.array(setpoint_xyz)
    vel = np.array(velocity_xyz)

    for i, dof in enumerate(trans_dofs):
        c = pid_constants[dof]
        error = sp[i] - pos[i]
        if abs(error) < c['ki_error_threshold']:
            integral[i] += error * time_step
            integral[i] = np.clip(integral[i], -c['ki_cap'], c['ki_cap'])
        force_temp = c['Kp'] * error + c['Kd'] * (0.0 - vel[i]) + c['Ki'] * integral[i]
        forces[i] = np.clip(force_temp, c['maxBack'], c['maxFront'])

    # --- Rotational PID via quaternion attitude error (indices 3, 4, 5) ---
    #
    # WHY NOT EULER ANGLES:
    # ZYX Euler decomposition has a gimbal coupling problem at yaw=90 deg:
    # a physical roll error maps to ZYX pitch, and the correction torque
    # (applied in body frame) actively destabilises the vehicle.
    #
    # WHY QUATERNION ERROR WORKS:
    # q_err = q_current^{-1} * q_setpoint gives the minimal rotation from
    # current to desired, expressed in the current body frame. Converting to
    # a rotation vector yields [roll_err, pitch_err, yaw_err] directly in
    # body axes with no cross-coupling regardless of heading.

    q_current  = R.from_quat(current_quat_xyzw)

    sp_roll, sp_pitch, sp_yaw = setpoint_euler_rpy
    # Reconstruct setpoint quaternion in Gazebo convention:
    # AUV +yaw  = Gazebo -yaw, AUV +pitch = Gazebo -pitch, roll sign same.
    q_setpoint = R.from_euler('zyx', [-sp_yaw, -sp_pitch, sp_roll], degrees=True)

    q_err      = q_current.inv() * q_setpoint
    err_rotvec = q_err.as_rotvec(degrees=True)

    roll_error  =  err_rotvec[0]
    pitch_error = -err_rotvec[1]
    yaw_error   = -err_rotvec[2]

    # Convert angular velocity from rad/s to deg/s so that Kp and Kd have
    # the same units (N·m/deg and N·m·s/deg). Without this conversion, Kd is
    # 57× heavier than expected relative to Kp, making rotational PIDs untunable.
    RAD_TO_DEG = 180.0 / np.pi
    angvel_degs = np.array(angular_velocity_xyz) * RAD_TO_DEG
    rot_errors = [roll_error, pitch_error, yaw_error]
    rot_dofs   = ['roll', 'pitch', 'yaw']
    rot_angvel = [angvel_degs[0], angvel_degs[1], angvel_degs[2]]

    for j, (dof, err, av) in enumerate(zip(rot_dofs, rot_errors, rot_angvel)):
        idx = j + 3
        c = pid_constants[dof]
        if abs(err) < c['ki_error_threshold']:
            integral[idx] += err * time_step
            integral[idx] = np.clip(integral[idx], -c['ki_cap'], c['ki_cap'])
        force_temp = c['Kp'] * err + c['Kd'] * (0.0 - av) + c['Ki'] * integral[idx]
        forces[idx] = np.clip(force_temp, c['maxBack'], c['maxFront'])

    return forces, integral


def transform_imu_from_gazebo(accel_xyz, gyro_xyz, quat_xyzw):
    """Convert a raw Gazebo IMU sample into the AUV (NED) convention.

    The project's NED convention is Gazebo with the Y and Z axes negated
    (forward unchanged, left->right, up->down) — exactly the flip
    `transform_pose_from_gazebo` applies to pose. Applied here to the
    body-frame accelerometer and gyro vectors and to the orientation
    quaternion:

        accel/gyro (x, y, z)        -> (x, -y, -z)
        quaternion (x, y, z, w)     -> (x, -y, -z, w)   [(roll, -pitch, -yaw)]

    The quaternion map is conjugation by a 180° X rotation, which is the
    quaternion equivalent of the (roll, -pitch, -yaw) Euler flip. The whole
    transform is its OWN INVERSE, so auv_localization undoes it with the
    identical operation (see auv_localization/frame_conversions.py).

    Parameters:
        accel_xyz   : (ax, ay, az)  body-frame linear acceleration
        gyro_xyz    : (wx, wy, wz)  body-frame angular velocity
        quat_xyzw   : (x, y, z, w)  orientation quaternion (Gazebo)

    Returns:
        (accel_ned, gyro_ned, quat_ned_xyzw)
    """
    ax, ay, az = accel_xyz
    wx, wy, wz = gyro_xyz
    qx, qy, qz, qw = quat_xyzw
    accel_ned = (ax, -ay, -az)
    gyro_ned  = (wx, -wy, -wz)
    quat_ned  = (qx, -qy, -qz, qw)
    return accel_ned, gyro_ned, quat_ned

def compute_wrench_from_global_forces(global_forces_6dof, current_quat_xyzw):
    """
    Convert 6-DOF AUV global forces to a Gazebo world-frame wrench.

    Force convention (AUV world frame → Gazebo world frame):
        fx_gazebo =  f[0]   surge: same axis
        fy_gazebo = -f[1]   sway:  AUV +Y = Gazebo -Y
        fz_gazebo = -f[2]   heave: AUV +Z = depth-down = Gazebo -Z

    Torque convention (AUV body frame → Gazebo world frame):
        t_body = [ f[3],   roll:  AUV X = Gazebo X,      no flip
                  -f[4],   pitch: AUV +pitch = Gazebo -pitch, negate
                  -f[5] ]  yaw:   AUV +yaw   = Gazebo -yaw,   negate
        t_world = R.from_quat(current_quat).apply(t_body)

    Parameters:
        global_forces_6dof : np.ndarray shape (6,) [surge, sway, heave, roll, pitch, yaw]
        current_quat_xyzw  : (x, y, z, w) Gazebo-convention quaternion

    Returns:
        force_xyz  : (fx, fy, fz) in Gazebo world frame
        torque_xyz : (tx, ty, tz) in Gazebo world frame
    """
    f = global_forces_6dof

    force_xyz = (f[0], -f[1], -f[2])

    t_body  = np.array([f[3], -f[4], -f[5]])
    t_world = R.from_quat(current_quat_xyzw).apply(t_body)
    torque_xyz = (t_world[0], t_world[1], t_world[2])

    return force_xyz, torque_xyz
