"""The car profile (params/wato_car.yaml) mapped onto the optimizer's parameters.

f110.ini keeps the solver settings; everything about the car comes from the profile, so the
car is described once, in plain units, with a source for each number.
"""

import math
import os

import numpy as np
import yaml

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "params", "wato_car.yaml")
G = 9.81


def load(path=None):
    with open(path or DEFAULT_PATH) as f:
        return yaml.safe_load(f)


def width_opt(car):
    """The optimizer's 'vehicle width including safety distance'."""
    return car["car"]["width_m"] + 2.0 * car["track"]["edge_margin_m"]


def lateral_limit(car):
    lim = car["limits"].get("max_lateral_accel_mps2")
    if lim is not None:
        return float(lim)
    return car["grip"]["mue"] * car["grip"]["safety_factor"] * G


def ggv(car):
    """ggv table [v, ax_max, ay_max] for the velocity-profile and check functions.
    The table's ax_max bounds braking (and acceleration before ax_max_machines)."""
    top = car["limits"]["top_speed_mps"]
    ax = car["limits"]["max_decel_mps2"]
    ay = lateral_limit(car)
    return np.array([[0.0, ax, ay], [top, ax, ay], [top + 5.0, ax, ay]])


def ax_max_machines(car):
    """[v, ax] the drivetrain allows when speeding up: the speed limiter's rate."""
    top = car["limits"]["top_speed_mps"]
    ax = car["limits"]["max_accel_mps2"]
    return np.array([[0.0, ax], [top, ax], [top + 5.0, ax]])


def apply(pars, car, opt_type):
    """Overwrite the car-dependent entries of the optimizer's `pars` (in place)."""
    c, lim, pw, grip, res = car["car"], car["limits"], car["powertrain"], car["grip"], car["resistance"]
    wheelbase = c["wheelbase_front_m"] + c["wheelbase_rear_m"]
    pars["veh_params"].update({
        "v_max": float(lim["top_speed_mps"]),
        "length": float(c["length_m"]),
        "width": float(c["width_m"]),
        "mass": float(c["mass_kg"]),
        "dragcoeff": float(res["drag_coeff"]),
        # tightest turn the steering allows (kinematic): tan(delta_max) / wheelbase
        "curvlim": math.tan(c["max_steer_rad"]) / wheelbase,
        "g": G,
    })
    pars["optim_opts"]["width_opt"] = width_opt(car)
    if opt_type == "mintime":
        pars["optim_opts"].update({
            "mue": float(grip["mue"] * grip["safety_factor"]),
            "safe_traj": True,
            "ax_pos_safe": float(lim["max_accel_mps2"]),
            "ax_neg_safe": float(lim["max_decel_mps2"]),
            "ay_safe": lateral_limit(car),
        })
        pars["vehicle_params_mintime"].update({
            "wheelbase_front": float(c["wheelbase_front_m"]),
            "wheelbase_rear": float(c["wheelbase_rear_m"]),
            "wheelbase": wheelbase,
            "track_width_front": float(c["width_m"]),
            "track_width_rear": float(c["width_m"]),
            "cog_z": float(c["cog_height_m"]),
            "I_z": float(c["yaw_inertia_kgm2"]),
            "liftcoeff_front": float(res["lift_coeff_front"]),
            "liftcoeff_rear": float(res["lift_coeff_rear"]),
            "power_max": float(pw["power_w"]),
            "f_drive_max": float(pw["drive_force_n"]),
            "f_brake_max": float(pw["brake_force_n"]),
            "delta_max": float(c["max_steer_rad"]),
        })
        pars["tire_params_mintime"].update({
            "c_roll": float(res["rolling_coeff"]),
            "f_z0": float(c["mass_kg"]) * G / 4.0,     # nominal load: a quarter of the car's weight
        })
    return pars


def summary(car):
    """One line per number that shapes the raceline, for logs and reports."""
    return {
        "mass_kg": car["car"]["mass_kg"],
        "top_speed_mps": car["limits"]["top_speed_mps"],
        "max_accel_mps2": car["limits"]["max_accel_mps2"],
        "max_decel_mps2": car["limits"]["max_decel_mps2"],
        "max_lateral_accel_mps2": round(lateral_limit(car), 3),
        "mue": car["grip"]["mue"],
        "mue_safety_factor": car["grip"]["safety_factor"],
        "edge_margin_m": car["track"]["edge_margin_m"],
        "width_opt_m": round(width_opt(car), 3),
    }


def check_against_stack(car, stack_root):
    """Warnings where the profile disagrees with the stack's car config (if the stack is there)."""
    warnings = []
    vesc_yaml = os.path.join(stack_root, "src", "interfacing", "interfacing_bringup", "config", "car", "vesc.yaml")
    if not os.path.exists(vesc_yaml):
        return warnings
    with open(vesc_yaml) as f:
        vesc = yaml.safe_load(f)
    shared = vesc.get("/**", {}).get("ros__parameters", {})
    limiter = vesc.get("ackermann_to_vesc_node", {}).get("ros__parameters", {})
    gain = shared.get("speed_to_erpm_gain")
    lim = car["limits"]
    if gain and shared.get("speed_max"):
        stack_top = shared["speed_max"] / gain
        if lim["top_speed_mps"] > stack_top + 1e-6:
            warnings.append("top_speed_mps %.2f is above the car's cap %.2f m/s (vesc.yaml speed_max)"
                            % (lim["top_speed_mps"], stack_top))
    for key, name in (("max_accel", "max_accel_mps2"), ("max_decel", "max_decel_mps2")):
        if key in limiter and abs(limiter[key] - lim[name]) > 1e-6:
            warnings.append("%s %.2f differs from the car's speed limiter %s %.2f m/s^2 (vesc.yaml)"
                            % (name, lim[name], key, limiter[key]))
    return warnings
