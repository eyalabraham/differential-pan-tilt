###########################################################
#
# ptik.py
#
#   Inverse Kinematics module for differential pan-tilt system.
#
#   April 26, 2020
#
###########################################################

import math

# Initialize the Inverse Kinematics model

# System mechanical characteristics
sensor_displacement = 60.0  # in [mm]

# Default motor characteristics
deg_per_step = 1.8
gear_ratio = 3
micro_steps = 8

# System characteristics
pan_deg_per_step = deg_per_step / gear_ratio / micro_steps
tilt_deg_per_step = deg_per_step / gear_ratio / micro_steps
pan_rad_per_step = pan_deg_per_step * math.pi / 180
tilt_rad_per_step = tilt_deg_per_step * math.pi / 180

# Limits from home position in degrees
absolute_pan_limit = 120.0
absolute_tilt_limit = 90.0 # Should be 87.0 because of driver integer calculations

def move_to_angle(alpha, theta, pan_pos = 0, tilt_pos = 0, base_rate = 500, object_distance_hint = -1):
    """
    Calculate and return relative steps from position (pan_pos, tilt_pos) to
    move to an alpha pan angle and a theta tilt angle.
    Angles in degrees relative to home position, where positive is CW and negative is CCW.
    Calculation accounts for sensor center displacement above tilt axis as well
    as object distance if object_distance_hint is a positive distance in [mm].

    Also calculate and return motor turn rates to synchronously pan and tilt
    from current position (pan_pos, tilt_pos) to new position (pan, tilt) angle.
    Will use base_rate as the rate for the fastest moving axis.

    Returns pan and tilt steps and rates in a tuple of (pan_steps, tilt_steps, pan_rate, tilt_rate)
    where rate sign indicate direction of positive for CW and negative for CCW.
    Returns a tuple of (-1, -1, 0, 0) if out of range.
    """

    if abs(alpha) > absolute_pan_limit or abs(theta) > absolute_tilt_limit:
        return (-1, -1)

    # First calculate pan movement
    # TODO Account for displacement perpendicular to pan axis.
    #      Similar calculation to tilt displacement but will have
    #      to take into account left or right of axis.
    pan_steps = int(alpha / pan_deg_per_step) - pan_pos

    # Calculate compensation for sensor displacement
    # if object distance hint is specified.
    theta_comp_deg = 0.0

    if object_distance_hint > 0:
        # Cannot look "back"
        if object_distance_hint < sensor_displacement:
            return (-1, -1, 0, 0)
        # Compute angle compensation and compare to system's step resolution.
        # No need to bother correcting an angle that the motors cannot reach.
        angle_sensitivity = deg_per_step / gear_ratio / micro_steps
        theta_comp = math.asin(sensor_displacement / object_distance_hint)
        theta_comp_deg = theta_comp * 180.0 / math.pi
        #print(f'sensitivity={angle_sensitivity}, comp={theta_comp}[rad]/{theta_comp_deg}[deg]')
        if theta_comp_deg < angle_sensitivity:
            theta_comp_deg = 0.0

    # Calculate tilt movement
    tilt_steps = pan_steps + (int(round((theta - theta_comp_deg) / tilt_deg_per_step)) - tilt_pos)

    # Calculate relative step rate per motor and output as list
    max_delta = max(abs(pan_steps), abs(tilt_steps))

    if max_delta > 0:
        return (abs(pan_steps), abs(tilt_steps), int(round(base_rate * pan_steps / max_delta)), int(round(base_rate * tilt_steps / max_delta)))
    else:
        return (-1, -1, 0, 0)


def angle_to_steps(alpha, theta):
    """
    Convert position given as absolute angle relative to home to pan-tilt steps relative to home.
    Return conversion as a tuple.
    """
    pan_steps = int(round(alpha / pan_deg_per_step))
    tilt_steps = int(round(theta / tilt_deg_per_step))
    return (pan_steps, tilt_steps)


def steps_to_angle():
    """
    Convert system position in steps to pan-tilt angle relative to home.
    Return conversion as a tuple.
    """
    pass


def cart_to_pol():
    """
    Covert from Cartesian coordinate system (x,y,z) to pan-tilt polar coordinates (alpha,theta).
    Return conversion as a tuple.
    """
    pass


def pol_to_cart():
    """
    Covert from pan-tilt polar coordinates (alpha,theta,distance) to Cartesian coordinate system (x,y,z).
    Return conversion as a tuple.
    """
    pass

