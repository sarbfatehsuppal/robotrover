"""Load rover_geometry.yaml into a small dataclass.

PyYAML is used when it is installed (it always is on a ROS 2 machine). On a
bare Python install we fall back to a tiny parser that understands the flat
``key: number  # comment`` format that file uses, so the climb analysis can
run anywhere.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, fields
from pathlib import Path


def _find_geometry_file() -> Path:
    env = os.environ.get('ROVER_GEOMETRY')
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('rover_description'))
        candidate = share / 'config' / 'rover_geometry.yaml'
        if candidate.exists():
            return candidate
    except Exception:  # not running inside a sourced ROS 2 workspace
        pass
    # Source checkout: ros2_ws/src/rover_sim/rover_sim/params.py
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / 'rover_description' / 'config' / 'rover_geometry.yaml'
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        'rover_geometry.yaml not found. Set ROVER_GEOMETRY=/path/to/rover_geometry.yaml')


def _parse_flat_yaml(text: str) -> dict:
    out = {}
    for raw in text.splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line or ':' not in line:
            continue
        key, value = line.split(':', 1)
        value = value.strip()
        if not value:
            continue
        try:
            out[key.strip()] = float(value)
        except ValueError:
            out[key.strip()] = value
    return out


def load_yaml(path: Path) -> dict:
    text = Path(path).read_text()
    try:
        import yaml  # type: ignore

        return yaml.safe_load(text)
    except ImportError:
        return _parse_flat_yaml(text)


@dataclass
class RoverParams:
    wheel_radius: float
    wheel_width: float
    track_width: float
    front_axle_x: float
    front_axle_z: float
    bogie_pivot_x: float
    bogie_pivot_z: float
    bogie_limit: float
    rocker_limit: float
    middle_axle_x: float
    middle_axle_z: float
    rear_axle_x: float
    rear_axle_z: float
    body_length: float
    body_width: float
    body_height: float
    body_center_x: float
    body_center_z: float
    mass_body: float
    body_cg_x: float
    body_cg_z: float
    mass_rocker: float
    mass_bogie: float
    mass_wheel_motor: float
    motor_bus_voltage: float
    motor_resistance: float
    driver_resistance: float
    motor_ke: float
    motor_kt: float
    motor_no_load_current: float
    motor_time_constant: float
    mu_ground: float
    rolling_resistance: float
    steer_limit: float
    steer_rate: float
    sonar_x: float
    sonar_z: float
    sonar_tilt: float
    sonar_half_angle: float
    sonar_min_range: float
    sonar_max_range: float

    # ---- derived values -------------------------------------------------
    @property
    def total_mass(self) -> float:
        return self.mass_body + 2 * (self.mass_rocker + self.mass_bogie) + 6 * self.mass_wheel_motor

    @property
    def pivot_height(self) -> float:
        """Rocker pivot height above flat ground."""
        return self.wheel_radius - self.front_axle_z

    @property
    def r_total(self) -> float:
        return self.motor_resistance + self.driver_resistance

    def stall_torque(self, duty: float = 1.0) -> float:
        current = duty * self.motor_bus_voltage / self.r_total
        return max(0.0, self.motor_kt * (current - self.motor_no_load_current))

    def no_load_speed(self, duty: float = 1.0) -> float:
        """Wheel speed in rad/s with no load at the given duty."""
        v = duty * self.motor_bus_voltage - self.motor_no_load_current * self.r_total
        return max(0.0, v / self.motor_ke)

    def wheel_speed(self, duty: float, load_torque: float) -> float:
        """Steady-state wheel speed (rad/s) for a signed duty and load torque.

        load_torque is the torque the ground asks the motor for (positive
        resists forward motion). Linear DC motor model:
            tau_em(w) = kt * (V - ke*w) / R
            tau_em(w) = load + kt*I0*sign(w)       (gearbox friction)
        Duty 0 means the MX1616 is in brake mode (both inputs high), so the
        motor is shorted and V = 0. That still resists motion, which is why a
        rover parked halfway up a step creeps back slowly instead of rolling.
        """
        v = duty * self.motor_bus_voltage
        r = self.r_total
        friction = self.motor_kt * self.motor_no_load_current
        w_fwd = (v - r * (load_torque + friction) / self.motor_kt) / self.motor_ke
        if w_fwd > 0.0:
            return w_fwd
        w_rev = (v - r * (load_torque - friction) / self.motor_kt) / self.motor_ke
        if w_rev < 0.0:
            return w_rev
        return 0.0  # inside the friction band: stalled / held

    def duty_for(self, wheel_speed: float, load_torque: float) -> float:
        """Duty needed to hold a wheel speed against a load torque."""
        direction = 1.0 if wheel_speed >= 0 else -1.0
        current = load_torque / self.motor_kt + self.motor_no_load_current * direction
        return (current * self.r_total + self.motor_ke * wheel_speed) / self.motor_bus_voltage


def load_params(path: str | os.PathLike | None = None) -> RoverParams:
    data = load_yaml(Path(path) if path else _find_geometry_file())
    names = {f.name for f in fields(RoverParams)}
    missing = names - set(data)
    if missing:
        raise KeyError(f'rover_geometry.yaml is missing: {sorted(missing)}')
    return RoverParams(**{k: float(data[k]) for k in names})
