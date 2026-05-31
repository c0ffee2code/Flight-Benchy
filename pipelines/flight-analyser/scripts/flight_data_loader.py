"""
Flight telemetry loader for the analyse-flight pipeline.

Provides:
  load_flight(csv_path)                              -> FlightData
  detect_reach_event(flight, setpoint, tolerance_deg) -> ReachEvent | None
  detect_hold_window(flight, reach_event, setpoint, tolerance_deg) -> HoldWindow | None

FlightData holds all CSV columns converted to numpy arrays; quaternions are
converted to roll angles in degrees inline.  No config or spec fields are mixed
in -- those come from configuration_loader / specification_loader.

ReachEvent is present when the encoder entered the tolerance band at least once
(the T->SP reach event -- first entry into the settling window band).
HoldWindow is present when the encoder remained in the band continuously for
at least HOLD_WINDOW_S seconds through to the end of the run (in-position confirmed).
Absence of either object means the respective event did not occur.
"""

import csv
import io
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


HOLD_WINDOW_S = 5.0


@dataclass
class ReachEvent:
    """
    First entry into the +-tolerance band (T->SP reach event).

    Fields
    ------
    start_idx    : Index into FlightData arrays of the first in-band sample.
    start_time_s : Elapsed time from run start to first in-band sample, seconds.
    """
    start_idx:    int
    start_time_s: float


@dataclass
class HoldWindow:
    """
    Confirmed stable hold: encoder remained in band from start_idx to end of run
    for at least HOLD_WINDOW_S seconds (in-position confirmed).

    Fields
    ------
    start_idx    : Index into FlightData arrays where the confirmed hold begins.
    start_time_s : Elapsed time from run start to hold start, seconds.
    duration_s   : Length of the confirmed hold (hold start to end of run), seconds.
    """
    start_idx:    int
    start_time_s: float
    duration_s:   float


@dataclass
class FlightData:
    """
    All telemetry columns from one log.csv, loaded by load_flight().

    Quaternion columns (ENC_QR/QI, IMU_QR/QI) are converted to roll angles in
    degrees inline; the raw quaternion values are not stored.

    Fields
    ------
    t_ms     : Sample timestamps, milliseconds. Shape (n,).
    enc_roll : Encoder roll angle, degrees (+ve = M1 end lower). Shape (n,).
    imu_roll : IMU roll angle, degrees (same sign convention). Shape (n,).
    gyro_x   : Calibrated gyroscope rate, degrees/s. Shape (n,).
    ang_err  : Outer PID angle error (ANG_ERR), degrees. Shape (n,).
    ang_p    : Outer PID P-term (ANG_P), degrees/s. Shape (n,).
    ang_i    : Outer PID I-term (ANG_I), degrees/s. Shape (n,).
    ang_d    : Outer PID D-term (ANG_D), degrees/s. Shape (n,).
    rate_sp  : Inner-loop rate setpoint (RATE_SP), degrees/s. Shape (n,).
    rate_err : Inner PID rate error (RATE_ERR), degrees/s. Shape (n,).
    rate_p   : Inner PID P-term (RATE_P), throttle units. Shape (n,).
    rate_i   : Inner PID I-term (RATE_I), throttle units. Shape (n,).
    rate_d   : Inner PID D-term (RATE_D), throttle units. Shape (n,).
    pid_out  : Combined PID output (PID_OUT), throttle units. Shape (n,).
    m1       : Motor 1 throttle command (M1), throttle units. Shape (n,).
    m2       : Motor 2 throttle command (M2), throttle units. Shape (n,).
    """
    t_ms:     np.ndarray
    enc_roll: np.ndarray
    imu_roll: np.ndarray
    gyro_x:   np.ndarray
    ang_err:  np.ndarray
    ang_p:    np.ndarray
    ang_i:    np.ndarray
    ang_d:    np.ndarray
    rate_sp:  np.ndarray
    rate_err: np.ndarray
    rate_p:   np.ndarray
    rate_i:   np.ndarray
    rate_d:   np.ndarray
    pid_out:  np.ndarray
    m1:       np.ndarray
    m2:       np.ndarray


def load_raw_rows(csv_path: Path) -> list:
    """Load log.csv as a list of raw string dicts, stripping pre-allocation null padding."""
    with open(csv_path, "rb") as f:
        data = f.read()
    null_pos = data.find(b'\x00')
    if null_pos != -1:
        data = data[:null_pos]
    return list(csv.DictReader(io.StringIO(data.decode("utf-8"))))


def _quat_to_roll(qr: np.ndarray, qi: np.ndarray) -> np.ndarray:
    return np.degrees(2.0 * np.arctan2(qi, qr))


def load_flight(csv_path: Path) -> FlightData:
    """
    Load log.csv and return a FlightData with all columns as numpy arrays.
    Quaternion columns are converted to roll degrees inline.
    Exits immediately if the file is missing or empty.
    """
    if not csv_path.exists():
        sys.exit(f"log.csv not found: {csv_path}")

    rows = load_raw_rows(csv_path)

    if not rows:
        sys.exit(f"Empty CSV: {csv_path}")

    def col(name: str) -> np.ndarray:
        return np.array([float(r[name]) for r in rows])

    return FlightData(
        t_ms=col("T_MS"),
        enc_roll=_quat_to_roll(col("ENC_QR"), col("ENC_QI")),
        imu_roll=_quat_to_roll(col("IMU_QR"), col("IMU_QI")),
        gyro_x=col("GYRO_X"),
        ang_err=col("ANG_ERR"),
        ang_p=col("ANG_P"),
        ang_i=col("ANG_I"),
        ang_d=col("ANG_D"),
        rate_sp=col("RATE_SP"),
        rate_err=col("RATE_ERR"),
        rate_p=col("RATE_P"),
        rate_i=col("RATE_I"),
        rate_d=col("RATE_D"),
        pid_out=col("PID_OUT"),
        m1=col("M1"),
        m2=col("M2"),
    )


def detect_reach_event(
    flight: FlightData,
    setpoint: float,
    tolerance_deg: float,
) -> ReachEvent | None:
    """
    Return the first entry into the +-tolerance_deg band, or None if never reached.
    """
    enc  = flight.enc_roll
    t_ms = flight.t_ms
    t0   = float(t_ms[0])

    for i in range(len(enc)):
        if abs(float(enc[i]) - setpoint) <= tolerance_deg:
            return ReachEvent(
                start_idx=i,
                start_time_s=float((t_ms[i] - t0) / 1000.0),
            )
    return None


def detect_hold_window(
    flight: FlightData,
    reach_event: ReachEvent | None,
    setpoint: float,
    tolerance_deg: float,
) -> HoldWindow | None:
    """
    Return the confirmed stable hold, or None if not settled.

    Searches for the last sample that exits the band after first reach; the hold
    starts at the sample immediately after it.  Requires the hold to extend
    continuously to the end of the run for at least HOLD_WINDOW_S seconds.

    Returns None immediately if reach_event is None (never reached the band).
    """
    if reach_event is None:
        return None

    enc  = flight.enc_roll
    t_ms = flight.t_ms
    t0   = float(t_ms[0])
    n    = len(enc)

    last_out = None
    for i in range(n - 1, reach_event.start_idx - 1, -1):
        if abs(float(enc[i]) - setpoint) > tolerance_deg:
            last_out = i
            break

    if last_out is None:
        hold_idx = reach_event.start_idx
    elif last_out + 1 < n:
        hold_idx = last_out + 1
    else:
        return None

    hold_dur = float((t_ms[-1] - t_ms[hold_idx]) / 1000.0)
    if hold_dur < HOLD_WINDOW_S:
        return None

    return HoldWindow(
        start_idx=hold_idx,
        start_time_s=float((t_ms[hold_idx] - t0) / 1000.0),
        duration_s=hold_dur,
    )
