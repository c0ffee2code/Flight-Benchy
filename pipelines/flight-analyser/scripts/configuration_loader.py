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
class LoopConfig:
    frequency_hz: int
    imu_report:   str
    pid:          PidConfig


@dataclass
class LoopsConfig:
    angle: LoopConfig
    rate:  LoopConfig


@dataclass
class TelemetryConfig:
    sample_every: int


@dataclass
class Configuration:
    setpoint_roll_deg:   float
    loops:               LoopsConfig
    motor:               MotorConfig
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


def _load_pid(pid_raw) -> PidConfig:
    return PidConfig(
        kp=float(_req(pid_raw, "kp")),
        ki=float(_req(pid_raw, "ki")),
        kd=float(_req(pid_raw, "kd")),
        iterm_limit=float(_req(pid_raw, "iterm_limit")),
    )

def _load_loop(loop_raw) -> LoopConfig:
    return LoopConfig(
        frequency_hz=int(_req(loop_raw, "frequency_hz")),
        imu_report=str(_req(loop_raw, "imu_report")),
        pid=_load_pid(_req(loop_raw, "pid")),
    )

def load_configuration(run_dir) -> Configuration:
    """Load and parse config.json from run_dir. Exits immediately if absent or malformed."""
    path = Path(run_dir) / "config.json"
    if not path.exists():
        sys.exit(f"config.json not found in {run_dir}")
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)

    vehicle   = _req(raw, "vehicle")
    loops_raw = _req(vehicle, "loops")
    motor     = _req(vehicle, "motor")
    ff        = vehicle.get("feedforward", {})
    telemetry = _req(raw, "telemetry")

    return Configuration(
        setpoint_roll_deg=float(_req(raw, "bench", "session", "setpoint", "roll_deg")),
        loops=LoopsConfig(
            angle=_load_loop(_req(loops_raw, "angle")),
            rate=_load_loop(_req(loops_raw, "rate")),
        ),
        motor=MotorConfig(
            throttle_min=float(_req(motor, "throttle_min")),
            throttle_max=float(_req(motor, "throttle_max")),
            base_throttle=float(motor.get("base_throttle", 0)),
        ),
        telemetry=TelemetryConfig(
            sample_every=int(_req(telemetry, "sample_every")),
        ),
        feedforward_lead_ms=float(ff["lead_ms"]) if "lead_ms" in ff else None,
    )
