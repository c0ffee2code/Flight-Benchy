"""
Diagnosis layer -- why did this run perform that way?

Answers: "What do the sensors, control loop, and timing diagnostics tell us?"

Writes diagnose.json to the run folder. Reads config.json alongside log.csv; both are mandatory.
Run after gate.py and verdict.py.

All confirmed-hold stats (hold_tracking, control_effort, inner_loop) are computed from
HoldWindow.start_idx (T_s) onward -- the confirmed continuous in-band phase. The pre-hold
approach phase (T->SP to T_s) is captured separately in approach_tracking. Whole-run sensor
health (IMU vs ENC) is still computed over the full run.

Metric groups in diagnose.json:
  sample_rate       -- n_samples, duration_s, actual_hz, dt mean/median/p99/max
  sensor_health     -- IMU vs ENC: MAE (overall/fast/slow), bias, rms_error, trail_pct, ranges
  hold_tracking     -- ENC vs setpoint from T_s (confirmed hold): bias, std, p95, max_ae,
                       fft_freq_hz (null if no dominant peak), fft_freq_resolution_hz,
                       whole_run_enc_mae
  approach_tracking -- ENC vs setpoint from T->SP to T_s: bias, std, p95, max_ae, pearson_r,
                       duration_s. Null when reach_idx == hold_idx (immediate settle) or no
                       confirmed hold.
  control_effort    -- mean/rms throttle, saturation upper/lower %, rms dM1/dt, rms dM2/dt,
                       ang_i_mean, m2_m1_mean, iterm_sign_ok (null if P-term dominant)
  inner_loop        -- rate_tracking_rms
  windup            -- ang/rate windup event counts and thresholds (whole-run)

Run from project root:
  python pipelines/flight-analyser/scripts/diagnose.py test_runs/flights/<flight_id>
"""


import json
import sys
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from specification_loader import load_specification                                          # noqa: E402
from configuration_loader import load_configuration, Configuration                          # noqa: E402
from flight_data_loader import load_flight, detect_reach_event, detect_hold_window, \
    FlightData, ReachEvent, HoldWindow                                                       # noqa: E402


# ---------------------------------------------------------------------------
# Stat output containers
# ---------------------------------------------------------------------------

@dataclass
class SampleRateStats:
    """
    Timing diagnostics for one flight run, computed by _sample_rate_stats().

    Fields
    ------
    n_samples    : Total number of CSV rows.
    duration_s   : Elapsed time from first to last sample, seconds.
    nominal_hz   : Configured sample rate = imu.rate_report_hz / telemetry.sample_every.
    actual_hz    : Achieved sample rate = (n_samples - 1) / duration_s.
    dt_mean_ms   : Mean inter-sample interval, milliseconds.
    dt_median_ms : Median inter-sample interval, milliseconds.
    dt_p99_ms    : 99th-percentile inter-sample interval, milliseconds.
    dt_max_ms    : Maximum inter-sample interval, milliseconds.
    """
    n_samples:    int
    duration_s:   float
    nominal_hz:   float
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
    mae         : Mean absolute error |IMU - ENC|, degrees.
    rms_error   : RMS of (IMU - ENC), degrees.
    max_ae      : Maximum |IMU - ENC|, degrees.
    bias        : Mean signed error (IMU - ENC), degrees.
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
    Encoder tracking statistics over the confirmed hold window (from T_s),
    computed by _hold_tracking_stats().

    Fields
    ------
    bias                   : Mean signed error enc_roll - setpoint, degrees.
    std                    : Standard deviation of hold error, degrees.
    p95                    : 95th percentile of |hold error|, degrees.
    max_ae                 : Maximum |hold error|, degrees.
    fft_freq_hz            : Dominant FFT frequency of hold error above 3x noise floor; None if
                             no peak clears that threshold.
    fft_freq_resolution_hz : Width of one FFT bin = 1 / hold_duration_s; None if window too short.
    whole_run_enc_mae      : Whole-run ENC MAE vs setpoint (includes rise phase -- reference only).
    """
    bias:                   float
    std:                    float
    p95:                    float
    max_ae:                 float
    fft_freq_hz:            float | None
    fft_freq_resolution_hz: float | None
    whole_run_enc_mae:      float


@dataclass
class ApproachTrackingStats:
    """
    Encoder tracking statistics over the approach phase (T->SP to T_s),
    computed by _approach_tracking_stats(). Present only when the encoder
    bounced in and out of the band before confirming a stable hold.

    Fields
    ------
    bias       : Mean signed error enc_roll - setpoint, degrees.
    std        : Standard deviation of approach error, degrees.
    p95        : 95th percentile of |approach error|, degrees.
    max_ae     : Maximum |approach error|, degrees.
    pearson_r  : Pearson r between enc_roll and imu_roll (meaningful: real motion present).
    duration_s : Length of the approach phase, seconds.
    """
    bias:       float
    std:        float
    p95:        float
    max_ae:     float
    pearson_r:  float
    duration_s: float


@dataclass
class ControlEffortStats:
    """
    Motor output statistics over the confirmed hold window (from T_s),
    computed by _control_effort_stats().

    Fields
    ------
    mean_throttle        : Mean of (M1 + M2) / 2, throttle units.
    rms_throttle         : RMS of (M1 + M2) / 2, throttle units.
    saturation_upper_pct : % of hold samples where M1 or M2 >= throttle_max.
    saturation_lower_pct : % of hold samples where M1 or M2 <= throttle_min.
    rms_dm1_dt           : RMS of dM1/dt (throttle/s) -- motor 1 command activity.
    rms_dm2_dt           : RMS of dM2/dt (throttle/s) -- motor 2 command activity.
    ang_i_mean           : Mean ANG_I over hold window, degrees/s.
    m2_m1_mean           : Mean (M2 - M1) over hold window, throttle units.
    iterm_sign_ok        : True when sign(ang_i_mean) == sign(m2_m1_mean). Disagreement
                           indicates a sign error somewhere in the control chain.
                           None when |ANG_P mean| >= |ANG_I mean| -- the P-term dominates
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
    Inner rate-loop tracking quality over the confirmed hold window (from T_s),
    computed by _inner_loop_stats().

    Fields
    ------
    rate_tracking_rms : RMS of (RATE_SP - GYRO_X) during hold, degrees/s.
    """
    rate_tracking_rms: float


@dataclass
class WindupStats:
    """
    I-term saturation event counts over the full run, computed by _windup_stats().

    Fields
    ------
    ang_windup_events     : Samples where |ANG_I| >= 50% of angle_iterm_limit.
    ang_windup_threshold  : The 50% threshold applied (= angle_iterm_limit x 0.5).
    rate_windup_events    : Samples where |RATE_I| >= 50% of rate_iterm_limit.
    rate_windup_threshold : The 50% threshold applied (= rate_iterm_limit x 0.5).
    """
    ang_windup_events:     int
    ang_windup_threshold:  float
    rate_windup_events:    int
    rate_windup_threshold: float


@dataclass
class DiagnoseOutput:
    """
    Full diagnose.json output. Serialised via dataclasses.asdict() -- field names
    become JSON keys, no magic strings in the producer.
    """
    sample_rate:       SampleRateStats
    sensor_health:     SensorHealthStats
    hold_tracking:     HoldTrackingStats
    approach_tracking: ApproachTrackingStats | None
    control_effort:    ControlEffortStats
    inner_loop:        InnerLoopStats
    windup:            WindupStats


# ---------------------------------------------------------------------------
# Stat groups -- each owns one section of the output
# ---------------------------------------------------------------------------

def _sample_rate_stats(fd: FlightData, cfg: Configuration) -> SampleRateStats:
    n        = len(fd.t_ms)
    dts      = np.diff(fd.t_ms)
    duration = float((fd.t_ms[-1] - fd.t_ms[0]) / 1000.0)
    return SampleRateStats(
        n_samples=n,
        duration_s=duration,
        nominal_hz=cfg.imu.rate_report_hz / cfg.telemetry.sample_every,
        actual_hz=(n - 1) / duration if duration > 0 else 0.0,
        dt_mean_ms=float(np.mean(dts)),
        dt_median_ms=float(np.median(dts)),
        dt_p99_ms=float(np.percentile(dts, 99)),
        dt_max_ms=float(np.max(dts)),
    )


def _sensor_health_stats(fd: FlightData) -> SensorHealthStats:
    dts      = np.diff(fd.t_ms)
    enc_roll = fd.enc_roll
    imu_roll = fd.imu_roll

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


def _hold_tracking_stats(fd: FlightData, setpoint: float, hi: int) -> HoldTrackingStats:
    hold_enc  = fd.enc_roll[hi:]
    hold_t_ms = fd.t_ms[hi:]
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

    return HoldTrackingStats(
        bias=float(np.mean(hold_err)),
        std=float(np.std(hold_err)),
        p95=float(np.percentile(np.abs(hold_err), 95)),
        max_ae=float(np.max(np.abs(hold_err))),
        fft_freq_hz=fft_freq,
        fft_freq_resolution_hz=fft_freq_resolution,
        whole_run_enc_mae=float(np.mean(np.abs(fd.enc_roll - setpoint))),
    )


def _approach_tracking_stats(
    fd: FlightData,
    setpoint: float,
    h_idx: int,
    sw_idx: int,
) -> ApproachTrackingStats:
    enc  = fd.enc_roll[h_idx:sw_idx]
    imu  = fd.imu_roll[h_idx:sw_idx]
    t_ms = fd.t_ms[h_idx:sw_idx]
    err  = enc - setpoint

    pearson_r = 0.0
    if len(enc) > 1 and np.std(enc) > 0 and np.std(imu) > 0:
        pearson_r = float(np.corrcoef(enc, imu)[0, 1])

    return ApproachTrackingStats(
        bias=float(np.mean(err)),
        std=float(np.std(err)),
        p95=float(np.percentile(np.abs(err), 95)),
        max_ae=float(np.max(np.abs(err))),
        pearson_r=pearson_r,
        duration_s=float((t_ms[-1] - t_ms[0]) / 1000.0) if len(t_ms) > 1 else 0.0,
    )


def _control_effort_stats(fd: FlightData, cfg: Configuration, hi: int) -> ControlEffortStats:
    hold_m1   = fd.m1[hi:]
    hold_m2   = fd.m2[hi:]
    hold_t_ms = fd.t_ms[hi:]
    avg_thr   = (hold_m1 + hold_m2) / 2.0

    saturation_upper = (hold_m1 >= cfg.motor.throttle_max) | (hold_m2 >= cfg.motor.throttle_max)
    saturation_lower = (hold_m1 <= cfg.motor.throttle_min) | (hold_m2 <= cfg.motor.throttle_min)

    hold_dts = np.diff(hold_t_ms) / 1000.0
    safe_dts = np.where(hold_dts > 0, hold_dts, 1e-6)
    dm1_dt   = np.diff(hold_m1) / safe_dts
    dm2_dt   = np.diff(hold_m2) / safe_dts

    # I-term self-consistency: in a settled hold, ANG_P ~= 0 and ANG_D ~= 0, so the
    # motor differential (M2-M1 = 2*output) is driven almost entirely by ANG_I.
    # sign(mean ANG_I) must equal sign(mean M2-M1). Disagreement indicates a sign
    # error somewhere in the control chain (gain, axis orientation, mixer direction).
    # Guard: only meaningful when I-term dominates (|ANG_P| < |ANG_I|). A large
    # P-term means the hold is not quiet -- either the error is large or the I-term
    # is a residual from the rise phase that hasn't drained yet. In that case the
    # assumption is violated and the check is suppressed (iterm_sign_ok = None).
    ang_i_mean = float(np.mean(fd.ang_i[hi:]))
    m2_m1_mean = float(np.mean(hold_m2 - hold_m1))
    ang_p_mean = float(np.mean(
        cfg.angle_pid.kp * (fd.imu_roll[hi:] - cfg.setpoint_roll_deg)
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


def _inner_loop_stats(fd: FlightData, hi: int) -> InnerLoopStats:
    rate_err = fd.rate_sp[hi:] - fd.gyro_x[hi:]
    return InnerLoopStats(
        rate_tracking_rms=float(np.sqrt(np.mean(rate_err ** 2))),
    )


def _windup_stats(fd: FlightData, cfg: Configuration) -> WindupStats:
    return WindupStats(
        ang_windup_events=int(np.sum(np.abs(fd.ang_i)  >= cfg.angle_pid.iterm_limit * 0.5)),
        ang_windup_threshold=cfg.angle_pid.iterm_limit * 0.5,
        rate_windup_events=int(np.sum(np.abs(fd.rate_i) >= cfg.rate_pid.iterm_limit  * 0.5)),
        rate_windup_threshold=cfg.rate_pid.iterm_limit  * 0.5,
    )


def compute_stats(
    fd: FlightData,
    cfg: Configuration,
    reach_event: ReachEvent | None = None,
    hold_window: HoldWindow | None = None,
) -> DiagnoseOutput:
    # All confirmed-hold groups use hold_window.start_idx; fall back to reach_event
    # if hold was never confirmed (e.g. run ended before 5s continuous in-band).
    if hold_window is not None:
        hi = hold_window.start_idx
    elif reach_event is not None:
        hi = reach_event.start_idx
    else:
        hi = 0

    # approach_tracking: present only when there is a gap between first reach and
    # confirmed hold (encoder bounced in/out of band before settling).
    approach = None
    if reach_event is not None and hold_window is not None:
        if hold_window.start_idx > reach_event.start_idx:
            approach = _approach_tracking_stats(
                fd, cfg.setpoint_roll_deg,
                reach_event.start_idx, hold_window.start_idx,
            )

    return DiagnoseOutput(
        sample_rate=_sample_rate_stats(fd, cfg),
        sensor_health=_sensor_health_stats(fd),
        hold_tracking=_hold_tracking_stats(fd, cfg.setpoint_roll_deg, hi),
        approach_tracking=approach,
        control_effort=_control_effort_stats(fd, cfg, hi),
        inner_loop=_inner_loop_stats(fd, hi),
        windup=_windup_stats(fd, cfg),
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: diagnose.py test_runs/flights/<flight_id>")

    run_dir = Path(sys.argv[1])
    cfg     = load_configuration(run_dir)
    spec    = load_specification(run_dir)

    fd      = load_flight(run_dir / "log.csv")
    reach_w = detect_reach_event(fd, cfg.setpoint_roll_deg, spec.tolerance_deg)
    hold_w  = detect_hold_window(fd, reach_w, cfg.setpoint_roll_deg, spec.tolerance_deg)

    output = compute_stats(fd, cfg, reach_event=reach_w, hold_window=hold_w)

    analysis_dir = run_dir / "analysis"
    analysis_dir.mkdir(exist_ok=True)
    out_path = analysis_dir / "diagnose.json"
    out_path.write_text(json.dumps(asdict(output), indent=2), encoding="utf-8")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
