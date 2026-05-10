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
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from score_flight import compute_kpis, load_setpoint  # noqa: E402


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_run(path_str):
    p        = Path(path_str)
    csv_path = (p / "log.csv")     if p.is_dir() else p
    cfg_path = (p / "config.json") if p.is_dir() else (p.parent / "config.json")
    label    = p.name if p.is_dir() else p.stem

    if not csv_path.exists():
        sys.exit(f"log.csv not found: {csv_path}")
    if not cfg_path.exists():
        sys.exit(f"config.json not found in {p} — mandatory for profile")

    with open(cfg_path) as f:
        config = json.load(f)

    rows = []
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        sys.exit(f"Empty CSV: {csv_path}")

    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    return cols, config, label


def quat_to_roll(qr, qi):
    return np.degrees(2.0 * np.arctan2(qi, qr))


def cols_to_samples(cols):
    angles = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    return list(zip(cols["T_MS"].tolist(), angles.tolist()))


def _get_iterm_limit(config, pid_key):
    try:
        return float(config["vehicle"][pid_key]["iterm_limit"])
    except (KeyError, TypeError):
        sys.exit(f"config.json missing vehicle.{pid_key}.iterm_limit")


# ---------------------------------------------------------------------------
# Stat groups — each owns one section of the output
# ---------------------------------------------------------------------------

def _sample_rate_stats(cols):
    t_ms       = cols["T_MS"]
    n          = len(t_ms)
    dts        = np.diff(t_ms)
    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0
    return {
        "n_samples":    n,
        "duration_s":   duration_s,
        "actual_hz":    (n - 1) / duration_s if duration_s > 0 else 0.0,
        "dt_mean_ms":   float(np.mean(dts)),
        "dt_median_ms": float(np.median(dts)),
        "dt_p99_ms":    float(np.percentile(dts, 99)),
        "dt_max_ms":    float(np.max(dts)),
    }


def _sensor_health_stats(cols):
    t_ms     = cols["T_MS"]
    dts      = np.diff(t_ms)
    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

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

    return {
        "mae":         float(np.mean(abs_errors)),
        "rms_error":   float(np.sqrt(np.mean(errors ** 2))),
        "max_ae":      float(np.max(abs_errors)),
        "bias":        float(np.mean(errors)),
        "mae_fast":    float(np.mean(abs_errors[fast_idx])) if len(fast_idx) > 0 else 0.0,
        "mae_slow":    float(np.mean(abs_errors[slow_idx])) if len(slow_idx) > 0 else 0.0,
        "correlation": float(np.corrcoef(enc_roll, imu_roll)[0, 1])
                       if np.std(enc_roll) > 0 and np.std(imu_roll) > 0 else 0.0,
        "trail_pct":   trail_pct,
        "enc_range":   float(np.ptp(enc_roll)),
        "imu_range":   float(np.ptp(imu_roll)),
    }


def _hold_tracking_stats(cols, setpoint, hi):
    enc_roll  = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll  = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])
    hold_enc  = enc_roll[hi:]
    hold_imu  = imu_roll[hi:]
    hold_t_ms = cols["T_MS"][hi:]
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
        peak_idx     = int(np.argmax(nondc)) + 1   # index into mag / freqs
        median_nondc = float(np.median(nondc))
        if median_nondc > 0 and float(mag[peak_idx]) >= 3.0 * median_nondc:
            fft_freq = float(freqs[peak_idx])

    pearson_r = 0.0
    if len(hold_enc) > 1 and np.std(hold_enc) > 0 and np.std(hold_imu) > 0:
        pearson_r = float(np.corrcoef(hold_enc, hold_imu)[0, 1])

    return {
        "bias":              float(np.mean(hold_err)),
        "std":               float(np.std(hold_err)),
        "p95":               float(np.percentile(np.abs(hold_err), 95)),
        "max_ae":            float(np.max(np.abs(hold_err))),
        "pearson_r":         pearson_r,
        "fft_freq_hz":            fft_freq,
        "fft_freq_resolution_hz": fft_freq_resolution,
        "whole_run_enc_mae":      float(np.mean(np.abs(enc_roll - setpoint))),
    }


def _control_effort_stats(cols, config, hi):
    try:
        throttle_min = float(config["vehicle"]["motor"]["throttle_min"])
        throttle_max = float(config["vehicle"]["motor"]["throttle_max"])
    except (KeyError, TypeError):
        sys.exit("config.json missing vehicle.motor.throttle_min / throttle_max")

    hold_m1   = cols["M1"][hi:]
    hold_m2   = cols["M2"][hi:]
    hold_t_ms = cols["T_MS"][hi:]
    avg_thr   = (hold_m1 + hold_m2) / 2.0
    saturation_upper_mask = (hold_m1 >= throttle_max) | (hold_m2 >= throttle_max)
    saturation_lower_mask = (hold_m1 <= throttle_min) | (hold_m2 <= throttle_min)

    hold_dts  = np.diff(hold_t_ms) / 1000.0
    safe_dts  = np.where(hold_dts > 0, hold_dts, 1e-6)
    dm1_dt    = np.diff(hold_m1) / safe_dts
    dm2_dt    = np.diff(hold_m2) / safe_dts

    return {
        "mean_throttle":        float(np.mean(avg_thr)),
        "rms_throttle":         float(np.sqrt(np.mean(avg_thr ** 2))),
        "saturation_upper_pct": float(np.mean(saturation_upper_mask) * 100.0),
        "saturation_lower_pct": float(np.mean(saturation_lower_mask) * 100.0),
        "rms_dm1_dt":           float(np.sqrt(np.mean(dm1_dt ** 2))),
        "rms_dm2_dt":           float(np.sqrt(np.mean(dm2_dt ** 2))),
    }


def _inner_loop_stats(cols, hi):
    rate_err = cols["RATE_SP"][hi:] - cols["GYRO_X"][hi:]
    return {
        "rate_tracking_rms": float(np.sqrt(np.mean(rate_err ** 2))),
    }


def _windup_stats(cols, config):
    ang_limit  = _get_iterm_limit(config, "angle_pid")
    rate_limit = _get_iterm_limit(config, "rate_pid")
    return {
        "ang_windup_events":     int(np.sum(np.abs(cols["ANG_I"])  >= ang_limit  * 0.5)),
        "ang_windup_threshold":  ang_limit  * 0.5,
        "rate_windup_events":    int(np.sum(np.abs(cols["RATE_I"]) >= rate_limit * 0.5)),
        "rate_windup_threshold": rate_limit * 0.5,
    }


def compute_stats(cols, config, setpoint, hold_start_idx=None):
    hi = hold_start_idx if hold_start_idx is not None else 0
    return (
        _sample_rate_stats(cols),
        _sensor_health_stats(cols),
        _hold_tracking_stats(cols, setpoint, hi),
        _control_effort_stats(cols, config, hi),
        _inner_loop_stats(cols, hi),
        _windup_stats(cols, config),
    )


# ---------------------------------------------------------------------------
# Printing
# ---------------------------------------------------------------------------

def _row(name, value, fmt=".2f"):
    print(f"  {name:<38} {value:>10{fmt}}")


def print_config_summary(config):
    vehicle = config.get("vehicle", {})
    angle   = vehicle.get("angle_pid", {})
    rate    = vehicle.get("rate_pid", {})
    motor   = vehicle.get("motor", {})
    imu     = vehicle.get("imu", {})
    print(f"  Angle PID: kp={angle.get('kp')}, ki={angle.get('ki')}, "
          f"kd={angle.get('kd')}, iterm_limit={angle.get('iterm_limit')}")
    print(f"  Rate PID:  kp={rate.get('kp')}, ki={rate.get('ki')}, "
          f"kd={rate.get('kd')}, iterm_limit={rate.get('iterm_limit')}")
    if "report_hz" in imu:
        print(f"  IMU: {imu.get('report')} @ {imu.get('report_hz')} Hz")
    else:
        print(f"  IMU: angle={imu.get('angle_report')} @ {imu.get('angle_report_hz')} Hz, "
              f"rate={imu.get('rate_report')} @ {imu.get('rate_report_hz')} Hz")
    print(f"  Motor: base={motor.get('base_throttle')}, "
          f"min={motor.get('throttle_min')}, max={motor.get('throttle_max')}")
    ff = vehicle.get("feedforward", {})
    if ff:
        print(f"  Feedforward: lead_ms={ff.get('lead_ms')}")


def print_canonical_kpis(kpis):
    def _fmt(val, fmt, suffix=""):
        return f"{val:{fmt}}{suffix}" if val is not None else "-"

    print("  --- Canonical KPIs (from score_flight) ---")
    print(f"  {'Reached setpoint':<38} {'YES' if kpis['reached'] else 'NO':>10}")
    print(f"  {'T->SP (s)':<38} {_fmt(kpis['time_to_s'],        '.1f', 's'):>10}")
    print(f"  {'Rise time 10-90% (s)':<38} {_fmt(kpis['rise_time_s'],     '.1f', 's'):>10}")
    print(f"  {'Overshoot (% of step)':<38} {_fmt(kpis['overshoot_pct'],   '.1f', '%'):>10}")
    print(f"  {'Settling time T_s (s)':<38} {_fmt(kpis['settling_time_s'], '.1f', 's'):>10}")
    print(f"  {'HoldMAE_s (°), post-settle':<38} {_fmt(kpis['hold_mae_settled'], '.2f', '°'):>10}")


def print_profile(label, rate, sensor, hold, effort, inner, windup, config, kpis, setpoint):
    w = 58
    print("=" * w)
    print(f"  {label}")
    print("-" * w)
    print_config_summary(config)
    print()

    if kpis is not None:
        print_canonical_kpis(kpis)
        print()

    print("  --- Sample Rate ---")
    _row("Samples",        rate["n_samples"],    ".0f")
    _row("Duration (s)",   rate["duration_s"],   ".1f")
    _row("Achieved Hz",    rate["actual_hz"],    ".1f")
    _row("Mean dt (ms)",   rate["dt_mean_ms"],   ".1f")
    _row("Median dt (ms)", rate["dt_median_ms"], ".1f")
    _row("dt_p99 (ms)",    rate["dt_p99_ms"],    ".1f")
    _row("dt_max (ms)",    rate["dt_max_ms"],    ".1f")

    print("\n  --- Sensor Health (IMU vs ENC, whole-run) ---")
    _row("MAE (overall)",         sensor["mae"])
    _row("MAE (fast motion)",     sensor["mae_fast"])
    _row("MAE (slow motion)",     sensor["mae_slow"])
    _row("Max AE",                sensor["max_ae"])
    _row("RMS error",             sensor["rms_error"])
    _row("Bias (IMU-ENC)",        sensor["bias"])
    _row("IMU trails motion (%)", sensor["trail_pct"], ".1f")
    _row("Encoder range (°)",     sensor["enc_range"], ".1f")
    _row("IMU range (°)",         sensor["imu_range"], ".1f")

    print(f"\n  --- Hold-Window Tracking (ENC vs {setpoint:+.0f}°, post-reach) ---")
    _row("Whole-run ENC MAE (°)",   hold["whole_run_enc_mae"])
    print("  (includes rise — not comparable to HoldMAE_s)")
    _row("Hold bias (°, signed)",   hold["bias"])
    _row("Hold std (°)",            hold["std"])
    _row("Hold P95 |error| (°)",    hold["p95"])
    _row("Hold max |error| (°)",    hold["max_ae"])
    _row("Pearson r (hold window)", hold["pearson_r"], ".4f")
    fft_val = f"{hold['fft_freq_hz']:.3f}" if hold["fft_freq_hz"] is not None else "-"
    print(f"  {'FFT dominant freq (Hz)':<38} {fft_val:>10}")
    if hold["fft_freq_resolution_hz"] is not None:
        print(f"  {'  (freq resolution Hz)':<38} {hold['fft_freq_resolution_hz']:>10.3f}")

    print("\n  --- Control Effort (hold window) ---")
    _row("Mean throttle (avg M1+M2)",          effort["mean_throttle"],        ".1f")
    _row("RMS throttle",                       effort["rms_throttle"],         ".1f")
    _row("Saturation upper % (>= max)",        effort["saturation_upper_pct"], ".1f")
    _row("Saturation lower % (<= min)",        effort["saturation_lower_pct"], ".1f")
    _row("RMS dM1/dt (throttle/s)",            effort["rms_dm1_dt"],           ".1f")
    _row("RMS dM2/dt (throttle/s)",            effort["rms_dm2_dt"],           ".1f")

    print("\n  --- Inner Loop (hold window) ---")
    _row("Rate tracking RMS (°/s)", inner["rate_tracking_rms"])

    print("\n  --- Windup (whole-run) ---")
    _row("Angle windup events",  windup["ang_windup_events"],  ".0f")
    print(f"  {'Angle windup threshold':<38} {windup['ang_windup_threshold']:>10.1f}")
    _row("Rate windup events",   windup["rate_windup_events"], ".0f")
    print(f"  {'Rate windup threshold':<38} {windup['rate_windup_threshold']:>10.1f}")

    print("=" * w)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: python .claude/skills/analyse-flight/scripts/profile_flight.py "
                 "test_runs/flights/<flight_id>")

    cols, config, label = load_run(sys.argv[1])

    p        = Path(sys.argv[1])
    run_dir  = p if p.is_dir() else p.parent
    setpoint = load_setpoint(run_dir)

    kpis           = compute_kpis(cols_to_samples(cols), setpoint)
    hold_start_idx = kpis["hold_start_idx"] if kpis else None

    rate, sensor, hold, effort, inner, windup = compute_stats(
        cols, config, setpoint, hold_start_idx=hold_start_idx,
    )

    print(f"Loaded {label}: {rate['n_samples']} samples, {rate['duration_s']:.1f}s\n")
    print_profile(label, rate, sensor, hold, effort, inner, windup, config, kpis, setpoint)


if __name__ == "__main__":
    main()