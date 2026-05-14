"""
Step 4 of the analyse-flight pipeline. Run only on flights that passed score_flight.py.

Prints grouped diagnostic metrics to console. Reads config.json alongside log.csv;
config.json is mandatory — the script exits with an error if it is absent or if any
required field is missing.

All hold-window stats are computed from first reach (hold_start_idx) onward, so they
exclude the pre-spin and rise phases. Whole-run sensor health (IMU vs ENC) is still
computed over the full run — it measures sensor quality, not control quality.

Metric groups:
  Canonical KPIs    — T→SP, rise time, overshoot, settling time, HoldMAE_s from
                      score_flight.compute_kpis()
  Sample Rate       — Achieved Hz; dt mean/median/p99/max
  Sensor Health     — IMU-vs-encoder whole-run: MAE (overall/fast/slow), bias,
  (IMU vs ENC)        RMS error, trail %, enc/imu range
  Hold-Window       — ENC vs setpoint from first reach: signed bias, std, P95,
  Tracking            max |error|, whole-run ENC MAE (for reference), Pearson r
                      (hold window only), FFT dominant frequency
  Control Effort    — Mean/RMS throttle (avg M1+M2), saturation_upper/saturation_lower %
                      (>= throttle_max / <= throttle_min, reported separately),
                      per-motor RMS dM1/dt and dM2/dt — all over the hold window
  Inner Loop        — RMS of (RATE_SP − GYRO_X) during hold — inner-loop tracking error
  Windup            — Ticks where |I-term| ≥ 50% of iterm_limit (whole-run)

Run from project root:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs/flights/<flight_id>
"""

import csv
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from score_flight import compute_kpis  # noqa: E402


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------

@dataclass
class RunData:
    """
    Flight telemetry columns required by the profile pipeline, loaded from log.csv.

    Fields
    ------
    t_ms      : Raw sample timestamps from T_MS column, milliseconds. Shape (n,).
    t_s       : Elapsed time in seconds from the first sample (t_ms shifted and scaled).
                Shape (n,).
    enc_roll  : Encoder roll angle in degrees, derived from ENC_QR/ENC_QI quaternion.
                Positive = M1 end lower. Shape (n,).
    imu_roll  : IMU roll angle in degrees, derived from IMU_QR/IMU_QI quaternion.
                Same sign convention as enc_roll. Shape (n,).
    gyro_x    : Calibrated gyroscope rate around the roll axis (GYRO_X), degrees/s.
                Shape (n,).
    rate_sp   : Inner-loop rate setpoint output by the outer PID (RATE_SP), degrees/s.
                Shape (n,).
    ang_i     : Outer (angle) PID I-term (ANG_I), degrees/s. Shape (n,).
    rate_i    : Inner (rate) PID I-term (RATE_I), throttle units. Shape (n,).
    m1        : Motor 1 throttle command (M1), throttle units. Shape (n,).
    m2        : Motor 2 throttle command (M2), throttle units. Shape (n,).
    label     : Run folder name (YYYY-MM-DD_hh-mm-ss), used in printed output.
    """
    t_ms:     np.ndarray
    t_s:      np.ndarray
    enc_roll: np.ndarray
    imu_roll: np.ndarray
    gyro_x:   np.ndarray
    rate_sp:  np.ndarray
    ang_i:    np.ndarray
    rate_i:   np.ndarray
    m1:       np.ndarray
    m2:       np.ndarray
    label:    str


@dataclass
class RunConfig:
    """
    System configuration for one run, parsed from config.json.

    Fields
    ------
    setpoint_roll_deg   : Target encoder roll angle, degrees (bench.session.setpoint.roll_deg).
    throttle_min        : Lower motor throttle clamp (vehicle.motor.throttle_min).
    throttle_max        : Upper motor throttle clamp (vehicle.motor.throttle_max).
    motor_base          : Base (hover) throttle offset (vehicle.motor.base_throttle).
    angle_kp            : Outer PID proportional gain.
    angle_ki            : Outer PID integral gain.
    angle_kd            : Outer PID derivative gain.
    angle_iterm_limit   : Outer PID I-term clamp (vehicle.angle_pid.iterm_limit).
    rate_kp             : Inner PID proportional gain.
    rate_ki             : Inner PID integral gain.
    rate_kd             : Inner PID derivative gain.
    rate_iterm_limit    : Inner PID I-term clamp (vehicle.rate_pid.iterm_limit).
    feedforward_lead_ms : Feedforward lead time in ms; None if not configured.
    imu_summary         : Pre-formatted IMU report string for display (handles both
                          single-report and dual-report config layouts).
    """
    setpoint_roll_deg:   float
    throttle_min:        float
    throttle_max:        float
    motor_base:          float
    angle_kp:            float
    angle_ki:            float
    angle_kd:            float
    angle_iterm_limit:   float
    rate_kp:             float
    rate_ki:             float
    rate_kd:             float
    rate_iterm_limit:    float
    feedforward_lead_ms: float | None
    imu_summary:         str


@dataclass
class SampleRateStats:
    """
    Timing diagnostics for one flight run, computed by _sample_rate_stats().

    Fields
    ------
    n_samples    : Total number of CSV rows.
    duration_s   : Elapsed time from first to last sample, seconds.
    actual_hz    : Achieved sample rate = (n_samples − 1) / duration_s.
    dt_mean_ms   : Mean inter-sample interval, milliseconds.
    dt_median_ms : Median inter-sample interval, milliseconds.
    dt_p99_ms    : 99th-percentile inter-sample interval, milliseconds.
    dt_max_ms    : Maximum inter-sample interval, milliseconds.
    """
    n_samples:    int
    duration_s:   float
    actual_hz:    float
    dt_mean_ms:   float
    dt_median_ms: float
    dt_p99_ms:    float
    dt_max_ms:    float


@dataclass
class SensorHealthStats:
    """
    IMU-vs-encoder agreement over the full run, computed by _sensor_health_stats().

    Fields
    ------
    mae         : Mean absolute error |IMU − ENC|, degrees.
    rms_error   : RMS of (IMU − ENC), degrees.
    max_ae      : Maximum |IMU − ENC|, degrees.
    bias        : Mean signed error (IMU − ENC), degrees.
    mae_fast    : MAE restricted to the top-quartile encoder velocity samples.
    mae_slow    : MAE restricted to the bottom-quartile encoder velocity samples.
    correlation : Pearson r between enc_roll and imu_roll over the full run.
    trail_pct   : % of moving samples where IMU lags behind encoder direction.
    enc_range   : Peak-to-peak encoder range, degrees.
    imu_range   : Peak-to-peak IMU range, degrees.
    """
    mae:         float
    rms_error:   float
    max_ae:      float
    bias:        float
    mae_fast:    float
    mae_slow:    float
    correlation: float
    trail_pct:   float
    enc_range:   float
    imu_range:   float


@dataclass
class HoldTrackingStats:
    """
    Encoder tracking statistics over the hold window (from first reach), computed by _hold_tracking_stats().

    Fields
    ------
    bias                   : Mean signed error enc_roll − setpoint, degrees.
    std                    : Standard deviation of hold error, degrees.
    p95                    : 95th percentile of |hold error|, degrees.
    max_ae                 : Maximum |hold error|, degrees.
    pearson_r              : Pearson r between enc_roll and imu_roll over the hold window.
    fft_freq_hz            : Dominant FFT frequency of hold error above 3× noise floor; None if
                             no peak clears that threshold.
    fft_freq_resolution_hz : Width of one FFT bin = 1 / hold_duration_s; None if window too short.
    whole_run_enc_mae      : Whole-run ENC MAE vs setpoint (includes rise phase — reference only).
    """
    bias:                   float
    std:                    float
    p95:                    float
    max_ae:                 float
    pearson_r:              float
    fft_freq_hz:            float | None
    fft_freq_resolution_hz: float | None
    whole_run_enc_mae:      float


@dataclass
class ControlEffortStats:
    """
    Motor output statistics over the hold window, computed by _control_effort_stats().

    Fields
    ------
    mean_throttle        : Mean of (M1 + M2) / 2, throttle units.
    rms_throttle         : RMS of (M1 + M2) / 2, throttle units.
    saturation_upper_pct : % of hold samples where M1 or M2 >= throttle_max.
    saturation_lower_pct : % of hold samples where M1 or M2 <= throttle_min.
    rms_dm1_dt           : RMS of dM1/dt (throttle/s) — motor 1 command activity.
    rms_dm2_dt           : RMS of dM2/dt (throttle/s) — motor 2 command activity.
    ang_i_mean           : Mean ANG_I over hold window, degrees/s.
    m2_m1_mean           : Mean (M2 − M1) over hold window, throttle units.
    iterm_sign_ok        : True when sign(ang_i_mean) == sign(m2_m1_mean). Disagreement
                           indicates a sign error somewhere in the control chain.
                           None when |ANG_P mean| >= |ANG_I mean| — the P-term dominates
                           during hold, so the I-term-drives-differential assumption is
                           violated (typical after a slow rise with residual I-term buildup).
    """
    mean_throttle:        float
    rms_throttle:         float
    saturation_upper_pct: float
    saturation_lower_pct: float
    rms_dm1_dt:           float
    rms_dm2_dt:           float
    ang_i_mean:           float
    m2_m1_mean:           float
    iterm_sign_ok:        bool | None


@dataclass
class InnerLoopStats:
    """
    Inner rate-loop tracking quality over the hold window, computed by _inner_loop_stats().

    Fields
    ------
    rate_tracking_rms : RMS of (RATE_SP − GYRO_X) during hold, degrees/s.
    """
    rate_tracking_rms: float


@dataclass
class WindupStats:
    """
    I-term saturation event counts over the full run, computed by _windup_stats().

    Fields
    ------
    ang_windup_events     : Samples where |ANG_I| >= 50% of angle_iterm_limit.
    ang_windup_threshold  : The 50% threshold applied (= angle_iterm_limit × 0.5).
    rate_windup_events    : Samples where |RATE_I| >= 50% of rate_iterm_limit.
    rate_windup_threshold : The 50% threshold applied (= rate_iterm_limit × 0.5).
    """
    ang_windup_events:     int
    ang_windup_threshold:  float
    rate_windup_events:    int
    rate_windup_threshold: float


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def _quat_to_roll(qr, qi):
    return np.degrees(2.0 * np.arctan2(qi, qr))


def _require(cfg, *path):
    """Navigate a nested dict by path; exit with a clear message if any key is missing."""
    obj = cfg
    for key in path:
        try:
            obj = obj[key]
        except (KeyError, TypeError):
            sys.exit(f"config.json missing: {'.'.join(str(k) for k in path)}")
    return obj


def load_run(path_str):
    p        = Path(path_str)
    csv_path = (p / "log.csv") if p.is_dir() else p
    label    = p.name if p.is_dir() else p.stem

    if not csv_path.exists():
        sys.exit(f"log.csv not found: {csv_path}")

    rows = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    if not rows:
        sys.exit(f"Empty CSV: {csv_path}")

    def col(name):
        return np.array([float(r[name]) for r in rows])

    t_ms = col("T_MS")
    return RunData(
        t_ms=t_ms,
        t_s=(t_ms - t_ms[0]) / 1000.0,
        enc_roll=_quat_to_roll(col("ENC_QR"), col("ENC_QI")),
        imu_roll=_quat_to_roll(col("IMU_QR"), col("IMU_QI")),
        gyro_x=col("GYRO_X"),
        rate_sp=col("RATE_SP"),
        ang_i=col("ANG_I"),
        rate_i=col("RATE_I"),
        m1=col("M1"),
        m2=col("M2"),
        label=label,
    )


def load_config(run_dir):
    cfg_path = Path(run_dir) / "config.json"
    if not cfg_path.exists():
        sys.exit(f"config.json not found in {run_dir} — mandatory for profile")
    with open(cfg_path) as f:
        cfg = json.load(f)

    vehicle = _require(cfg, "vehicle")
    angle   = _require(vehicle, "angle_pid")
    rate    = _require(vehicle, "rate_pid")
    motor   = _require(vehicle, "motor")
    imu = _require(vehicle, "imu")
    ff  = vehicle.get("feedforward", {})

    imu_summary = (f"angle={_require(imu, 'angle_report')} @ {_require(imu, 'angle_report_hz')} Hz, "
                   f"rate={_require(imu, 'rate_report')} @ {_require(imu, 'rate_report_hz')} Hz")

    return RunConfig(
        setpoint_roll_deg=float(_require(cfg, "bench", "session", "setpoint", "roll_deg")),
        throttle_min=float(_require(motor, "throttle_min")),
        throttle_max=float(_require(motor, "throttle_max")),
        motor_base=float(motor.get("base_throttle", 0)),
        angle_kp=float(_require(angle, "kp")),
        angle_ki=float(_require(angle, "ki")),
        angle_kd=float(_require(angle, "kd")),
        angle_iterm_limit=float(_require(angle, "iterm_limit")),
        rate_kp=float(_require(rate, "kp")),
        rate_ki=float(_require(rate, "ki")),
        rate_kd=float(_require(rate, "kd")),
        rate_iterm_limit=float(_require(rate, "iterm_limit")),
        feedforward_lead_ms=float(ff["lead_ms"]) if "lead_ms" in ff else None,
        imu_summary=imu_summary,
    )


# ---------------------------------------------------------------------------
# Stat groups — each owns one section of the output
# ---------------------------------------------------------------------------

def _sample_rate_stats(run_data):
    n        = len(run_data.t_ms)
    dts      = np.diff(run_data.t_ms)
    duration = run_data.t_s[-1]
    return SampleRateStats(
        n_samples=n,
        duration_s=duration,
        actual_hz=(n - 1) / duration if duration > 0 else 0.0,
        dt_mean_ms=float(np.mean(dts)),
        dt_median_ms=float(np.median(dts)),
        dt_p99_ms=float(np.percentile(dts, 99)),
        dt_max_ms=float(np.max(dts)),
    )


def _sensor_health_stats(run_data):
    dts      = np.diff(run_data.t_ms)
    enc_roll = run_data.enc_roll
    imu_roll = run_data.imu_roll

    errors     = imu_roll - enc_roll
    abs_errors = np.abs(errors)

    enc_vel   = np.abs(np.diff(enc_roll)) / (dts / 1000.0)
    vel_order = np.argsort(enc_vel)
    quarter   = len(vel_order) // 4
    slow_idx  = vel_order[:quarter] + 1
    fast_idx  = vel_order[-quarter:] + 1

    motion_dir = np.diff(enc_roll)
    imu_offset = enc_roll[1:] - imu_roll[1:]
    moving     = np.abs(motion_dir) > 1.0
    trail_pct  = 0.0
    if np.sum(moving) > 0:
        trailing  = (motion_dir[moving] * imu_offset[moving]) > 0
        trail_pct = float(np.sum(trailing) / np.sum(moving) * 100)

    return SensorHealthStats(
        mae=float(np.mean(abs_errors)),
        rms_error=float(np.sqrt(np.mean(errors ** 2))),
        max_ae=float(np.max(abs_errors)),
        bias=float(np.mean(errors)),
        mae_fast=float(np.mean(abs_errors[fast_idx])) if len(fast_idx) > 0 else 0.0,
        mae_slow=float(np.mean(abs_errors[slow_idx])) if len(slow_idx) > 0 else 0.0,
        correlation=float(np.corrcoef(enc_roll, imu_roll)[0, 1])
                    if np.std(enc_roll) > 0 and np.std(imu_roll) > 0 else 0.0,
        trail_pct=trail_pct,
        enc_range=float(np.ptp(enc_roll)),
        imu_range=float(np.ptp(imu_roll)),
    )


def _hold_tracking_stats(run_data, setpoint, hi):
    hold_enc  = run_data.enc_roll[hi:]
    hold_imu  = run_data.imu_roll[hi:]
    hold_t_ms = run_data.t_ms[hi:]
    hold_err  = hold_enc - setpoint

    fft_freq = None
    fft_freq_resolution = None
    if len(hold_err) > 10:
        win      = np.hanning(len(hold_err))
        mag      = np.abs(np.fft.rfft(hold_err * win))
        dt_s     = float(np.mean(np.diff(hold_t_ms))) / 1000.0
        freqs    = np.fft.rfftfreq(len(hold_err), d=dt_s)
        nondc    = mag[1:]
        if len(freqs) > 1:
            fft_freq_resolution = float(freqs[1])
        peak_idx     = int(np.argmax(nondc)) + 1
        median_nondc = float(np.median(nondc))
        if median_nondc > 0 and float(mag[peak_idx]) >= 3.0 * median_nondc:
            fft_freq = float(freqs[peak_idx])

    pearson_r = 0.0
    if len(hold_enc) > 1 and np.std(hold_enc) > 0 and np.std(hold_imu) > 0:
        pearson_r = float(np.corrcoef(hold_enc, hold_imu)[0, 1])

    return HoldTrackingStats(
        bias=float(np.mean(hold_err)),
        std=float(np.std(hold_err)),
        p95=float(np.percentile(np.abs(hold_err), 95)),
        max_ae=float(np.max(np.abs(hold_err))),
        pearson_r=pearson_r,
        fft_freq_hz=fft_freq,
        fft_freq_resolution_hz=fft_freq_resolution,
        whole_run_enc_mae=float(np.mean(np.abs(run_data.enc_roll - setpoint))),
    )


def _control_effort_stats(run_data, run_config, hi):
    hold_m1   = run_data.m1[hi:]
    hold_m2   = run_data.m2[hi:]
    hold_t_ms = run_data.t_ms[hi:]
    avg_thr   = (hold_m1 + hold_m2) / 2.0

    saturation_upper = (hold_m1 >= run_config.throttle_max) | (hold_m2 >= run_config.throttle_max)
    saturation_lower = (hold_m1 <= run_config.throttle_min) | (hold_m2 <= run_config.throttle_min)

    hold_dts = np.diff(hold_t_ms) / 1000.0
    safe_dts = np.where(hold_dts > 0, hold_dts, 1e-6)
    dm1_dt   = np.diff(hold_m1) / safe_dts
    dm2_dt   = np.diff(hold_m2) / safe_dts

    # I-term self-consistency: in a settled hold, ANG_P ≈ 0 and ANG_D ≈ 0, so the
    # motor differential (M2−M1 = 2×output) is driven almost entirely by ANG_I.
    # sign(mean ANG_I) must equal sign(mean M2−M1). Disagreement indicates a sign
    # error somewhere in the control chain (gain, axis orientation, mixer direction).
    # Guard: only meaningful when I-term dominates (|ANG_P| < |ANG_I|). A large
    # P-term means the hold is not quiet — either the error is large or the I-term
    # is a residual from the rise phase that hasn't drained yet. In that case the
    # assumption is violated and the check is suppressed (iterm_sign_ok = None).
    ang_i_mean = float(np.mean(run_data.ang_i[hi:]))
    m2_m1_mean = float(np.mean(hold_m2 - hold_m1))
    ang_p_mean = float(np.mean(
        run_config.angle_kp * (run_data.imu_roll[hi:] - run_config.setpoint_roll_deg)
    ))
    if abs(ang_i_mean) > abs(ang_p_mean):
        iterm_sign_ok: bool | None = ang_i_mean * m2_m1_mean >= 0
    else:
        iterm_sign_ok = None

    return ControlEffortStats(
        mean_throttle=float(np.mean(avg_thr)),
        rms_throttle=float(np.sqrt(np.mean(avg_thr ** 2))),
        saturation_upper_pct=float(np.mean(saturation_upper) * 100.0),
        saturation_lower_pct=float(np.mean(saturation_lower) * 100.0),
        rms_dm1_dt=float(np.sqrt(np.mean(dm1_dt ** 2))),
        rms_dm2_dt=float(np.sqrt(np.mean(dm2_dt ** 2))),
        ang_i_mean=ang_i_mean,
        m2_m1_mean=m2_m1_mean,
        iterm_sign_ok=iterm_sign_ok,
    )


def _inner_loop_stats(run_data, hi):
    rate_err = run_data.rate_sp[hi:] - run_data.gyro_x[hi:]
    return InnerLoopStats(
        rate_tracking_rms=float(np.sqrt(np.mean(rate_err ** 2))),
    )


def _windup_stats(run_data, run_config):
    return WindupStats(
        ang_windup_events=int(np.sum(np.abs(run_data.ang_i)  >= run_config.angle_iterm_limit  * 0.5)),
        ang_windup_threshold=run_config.angle_iterm_limit  * 0.5,
        rate_windup_events=int(np.sum(np.abs(run_data.rate_i) >= run_config.rate_iterm_limit   * 0.5)),
        rate_windup_threshold=run_config.rate_iterm_limit   * 0.5,
    )


def compute_stats(run_data, run_config, hold_start_idx=None):
    hi = hold_start_idx if hold_start_idx is not None else 0
    return (
        _sample_rate_stats(run_data),
        _sensor_health_stats(run_data),
        _hold_tracking_stats(run_data, run_config.setpoint_roll_deg, hi),
        _control_effort_stats(run_data, run_config, hi),
        _inner_loop_stats(run_data, hi),
        _windup_stats(run_data, run_config),
    )


# ---------------------------------------------------------------------------
# Printing
# ---------------------------------------------------------------------------

def _row(name, value, fmt=".2f"):
    print(f"  {name:<38} {value:>10{fmt}}")


def print_config_summary(run_config):
    print(f"  Angle PID: kp={run_config.angle_kp}, ki={run_config.angle_ki}, "
          f"kd={run_config.angle_kd}, iterm_limit={run_config.angle_iterm_limit}")
    print(f"  Rate PID:  kp={run_config.rate_kp}, ki={run_config.rate_ki}, "
          f"kd={run_config.rate_kd}, iterm_limit={run_config.rate_iterm_limit}")
    print(f"  IMU: {run_config.imu_summary}")
    print(f"  Motor: base={run_config.motor_base}, "
          f"min={run_config.throttle_min}, max={run_config.throttle_max}")
    if run_config.feedforward_lead_ms is not None:
        print(f"  Feedforward: lead_ms={run_config.feedforward_lead_ms}")


def print_canonical_kpis(kpis):
    def _fmt(val, fmt, suffix=""):
        return f"{val:{fmt}}{suffix}" if val is not None else "-"

    if kpis.damping_ratio is not None:
        zeta_str = f"{kpis.damping_ratio:.3f}"
    elif kpis.overshoot_pct is not None and kpis.overshoot_pct == 0.0:
        zeta_str = ">=1 (no OS)"
    else:
        zeta_str = "-"

    print("  --- Canonical KPIs (from score_flight) ---")
    print(f"  {'Reached setpoint':<38} {'YES' if kpis.reached else 'NO':>10}")
    print(f"  {'T->SP (s)':<38} {_fmt(kpis.time_to_s,        '.1f', 's'):>10}")
    print(f"  {'Rise time 10-90% (s)':<38} {_fmt(kpis.rise_time_s,     '.1f', 's'):>10}")
    print(f"  {'Overshoot (% of step)':<38} {_fmt(kpis.overshoot_pct,   '.1f', '%'):>10}")
    print(f"  {'Damping ratio zeta':<38} {zeta_str:>10}")
    print(f"  {'Settling time T_s (s)':<38} {_fmt(kpis.settling_time_s, '.1f', 's'):>10}")
    print(f"  {'HoldMAE_s (deg), post-settle':<38} {_fmt(kpis.hold_mae_settled, '.2f', 'deg'):>10}")


def print_profile(run_data, run_config, rate, sensor, hold, effort, inner, windup, kpis):
    setpoint = run_config.setpoint_roll_deg
    w = 58
    print("=" * w)
    print(f"  {run_data.label}")
    print("-" * w)
    print_config_summary(run_config)
    print()

    if kpis is not None:
        print_canonical_kpis(kpis)
        print()

    print("  --- Sample Rate ---")
    _row("Samples",        rate.n_samples,    ".0f")
    _row("Duration (s)",   rate.duration_s,   ".1f")
    _row("Achieved Hz",    rate.actual_hz,    ".1f")
    _row("Mean dt (ms)",   rate.dt_mean_ms,   ".1f")
    _row("Median dt (ms)", rate.dt_median_ms, ".1f")
    _row("dt_p99 (ms)",    rate.dt_p99_ms,    ".1f")
    _row("dt_max (ms)",    rate.dt_max_ms,    ".1f")

    print("\n  --- Sensor Health (IMU vs ENC, whole-run) ---")
    _row("MAE (overall)",         sensor.mae)
    _row("MAE (fast motion)",     sensor.mae_fast)
    _row("MAE (slow motion)",     sensor.mae_slow)
    _row("Max AE",                sensor.max_ae)
    _row("RMS error",             sensor.rms_error)
    _row("Bias (IMU-ENC)",        sensor.bias)
    _row("IMU trails motion (%)", sensor.trail_pct, ".1f")
    _row("Encoder range (deg)",   sensor.enc_range, ".1f")
    _row("IMU range (deg)",       sensor.imu_range, ".1f")

    print(f"\n  --- Hold-Window Tracking (ENC vs {setpoint:+.0f}deg, post-reach) ---")
    _row("Whole-run ENC MAE (deg)",   hold.whole_run_enc_mae)
    print("  (includes rise - not comparable to HoldMAE_s)")
    _row("Hold bias (deg, signed)",   hold.bias)
    _row("Hold std (deg)",            hold.std)
    _row("Hold P95 |error| (deg)",    hold.p95)
    _row("Hold max |error| (deg)",    hold.max_ae)
    _row("Pearson r (hold window)", hold.pearson_r, ".4f")
    fft_val = f"{hold.fft_freq_hz:.3f}" if hold.fft_freq_hz is not None else "-"
    print(f"  {'FFT dominant freq (Hz)':<38} {fft_val:>10}")
    if hold.fft_freq_resolution_hz is not None:
        print(f"  {'  (freq resolution Hz)':<38} {hold.fft_freq_resolution_hz:>10.3f}")
    if (hold.fft_freq_hz is not None and
            hold.fft_freq_resolution_hz is not None and
            hold.fft_freq_hz < 3.0 * hold.fft_freq_resolution_hz):
        print("  (advisory - peak within 3x resolution; may be drift rather than oscillation)")

    print("\n  --- Control Effort (hold window) ---")
    _row("Mean throttle (avg M1+M2)",          effort.mean_throttle,        ".1f")
    _row("RMS throttle",                       effort.rms_throttle,         ".1f")
    _row("Saturation upper % (>= max)",        effort.saturation_upper_pct, ".1f")
    _row("Saturation lower % (<= min)",        effort.saturation_lower_pct, ".1f")
    _row("RMS dM1/dt (throttle/s)",            effort.rms_dm1_dt,           ".1f")
    _row("RMS dM2/dt (throttle/s)",            effort.rms_dm2_dt,           ".1f")
    _row("ANG_I mean (hold, deg/s)",           effort.ang_i_mean,           ".2f")
    _row("M2-M1 mean (hold, throttle)",        effort.m2_m1_mean,           ".1f")
    if effort.iterm_sign_ok is None:
        sign_str = "N/A (P-term dominant)"
    elif effort.iterm_sign_ok:
        sign_str = "OK"
    else:
        sign_str = "FLIP - sign error in control chain"
    print(f"  {'I-term sign vs dM':<38} {sign_str:>10}")

    print("\n  --- Inner Loop (hold window) ---")
    _row("Rate tracking RMS (deg/s)", inner.rate_tracking_rms)

    print("\n  --- Windup (whole-run) ---")
    _row("Angle windup events",  windup.ang_windup_events,  ".0f")
    print(f"  {'Angle windup threshold':<38} {windup.ang_windup_threshold:>10.1f}")
    _row("Rate windup events",   windup.rate_windup_events, ".0f")
    print(f"  {'Rate windup threshold':<38} {windup.rate_windup_threshold:>10.1f}")

    print("=" * w)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: python .claude/skills/analyse-flight/scripts/profile_flight.py "
                 "test_runs/flights/<flight_id>")

    p        = Path(sys.argv[1])
    run_dir  = p if p.is_dir() else p.parent
    run_data   = load_run(sys.argv[1])
    run_config = load_config(run_dir)

    kpis = compute_kpis(
        list(zip(run_data.t_ms.tolist(), run_data.enc_roll.tolist())),
        run_config.setpoint_roll_deg,
    )
    hold_start_idx = kpis.hold_start_idx if kpis else None

    rate, sensor, hold, effort, inner, windup = compute_stats(
        run_data, run_config, hold_start_idx=hold_start_idx,
    )

    print(f"Loaded {run_data.label}: {rate.n_samples} samples, {rate.duration_s:.1f}s\n")
    print_profile(run_data, run_config, rate, sensor, hold, effort, inner, windup, kpis)


if __name__ == "__main__":
    main()
