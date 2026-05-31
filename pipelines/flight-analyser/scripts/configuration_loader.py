"""
Configuration loader -- reads config.json from a run folder.

Functions
---------
load_configuration(run_dir) -> Configuration
    Parse config.json. Exits immediately if absent or a required field is missing.
"""

import json
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class PidConfig:
    kp:          float
    ki:          float
    kd:          float
    iterm_limit: float


@dataclass
class MotorConfig:
    throttle_min:  float
    throttle_max:  float
    base_throttle: float


@dataclass
class ImuConfig:
    angle_report:    str
    angle_report_hz: int
    rate_report:     str
    rate_report_hz:  int


@dataclass
class TelemetryConfig:
    sample_every: int


@dataclass
class Configuration:
    setpoint_roll_deg:   float
    angle_pid:           PidConfig
    rate_pid:            PidConfig
    motor:               MotorConfig
    imu:                 ImuConfig
    telemetry:           TelemetryConfig
    feedforward_lead_ms: float | None


def _req(obj, *path):
    """Navigate a nested dict by path; exit with a clear message if any key is missing."""
    cur = obj
    for key in path:
        try:
            cur = cur[key]
        except (KeyError, TypeError):
            sys.exit(f"config.json missing: {'.'.join(str(k) for k in path)}")
    return cur


def load_configuration(run_dir) -> Configuration:
    """Load and parse config.json from run_dir. Exits immediately if absent or malformed."""
    path = Path(run_dir) / "config.json"
    if not path.exists():
        sys.exit(f"config.json not found in {run_dir}")
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)

    vehicle   = _req(raw, "vehicle")
    apid      = _req(vehicle, "angle_pid")
    rpid      = _req(vehicle, "rate_pid")
    motor     = _req(vehicle, "motor")
    imu       = _req(vehicle, "imu")
    ff        = vehicle.get("feedforward", {})
    telemetry = _req(raw, "telemetry")

    return Configuration(
        setpoint_roll_deg=float(_req(raw, "bench", "session", "setpoint", "roll_deg")),
        angle_pid=PidConfig(
            kp=float(_req(apid, "kp")),
            ki=float(_req(apid, "ki")),
            kd=float(_req(apid, "kd")),
            iterm_limit=float(_req(apid, "iterm_limit")),
        ),
        rate_pid=PidConfig(
            kp=float(_req(rpid, "kp")),
            ki=float(_req(rpid, "ki")),
            kd=float(_req(rpid, "kd")),
            iterm_limit=float(_req(rpid, "iterm_limit")),
        ),
        motor=MotorConfig(
            throttle_min=float(_req(motor, "throttle_min")),
            throttle_max=float(_req(motor, "throttle_max")),
            base_throttle=float(motor.get("base_throttle", 0)),
        ),
        imu=ImuConfig(
            angle_report=str(_req(imu, "angle_report")),
            angle_report_hz=int(_req(imu, "angle_report_hz")),
            rate_report=str(_req(imu, "rate_report")),
            rate_report_hz=int(_req(imu, "rate_report_hz")),
        ),
        telemetry=TelemetryConfig(
            sample_every=int(_req(telemetry, "sample_every")),
        ),
        feedforward_lead_ms=float(ff["lead_ms"]) if "lead_ms" in ff else None,
    )
