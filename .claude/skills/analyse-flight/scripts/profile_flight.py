"""
Step 4 of the analyse-flight pipeline. Run only on flights that passed score_flight.py.

Prints grouped diagnostic metrics to console. Reads config.json alongside log.csv;
config.json is mandatory — the script exits with an error if it is absent or if any
required field is missing.

Metric groups:
  Canonical KPIs    — HoldMAE, T→setpoint, T@setpoint from score_flight.compute_kpis()
  Sample Rate       — Achieved Hz; should match 1000 / (INNER_INTERVAL_MS × TELEMETRY_SAMPLE_EVERY)
  Sensor Health     — IMU-vs-encoder MAE, bias, Pearson r, trail %
  (IMU vs ENC)
  Setpoint Error    — Whole-run ENC MAE against setpoint. Includes the rise — not comparable to HoldMAE.
  Correlation &     — Pearson r, IMU trailing encoder.
  Tracking
  Oscillation &     — osc_freq_hz: sign-change rate of IMU-vs-encoder tracking error. Windup
  Windup              events: ticks where |I-term| ≥ 50% of iterm_limit from config.json.

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
    p = Path(path_str)
    csv_path = (p / "log.csv") if p.is_dir() else p
    cfg_path = (p / "config.json") if p.is_dir() else (p.parent / "config.json")
    label = p.name if p.is_dir() else p.stem

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


def get_iterm_limit(config, pid_key):
    try:
        return float(config["vehicle"][pid_key]["iterm_limit"])
    except (KeyError, TypeError):
        sys.exit(f"config.json missing vehicle.{pid_key}.iterm_limit — required for windup computation")


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def compute_stats(cols, config, setpoint):
    n = len(cols["T_MS"])
    t_ms = cols["T_MS"]
    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0
    actual_hz = (n - 1) / duration_s if duration_s > 0 else 0.0
    dts = np.diff(t_ms)

    errors = imu_roll - enc_roll
    abs_errors = np.abs(errors)

    enc_vel = np.abs(np.diff(enc_roll)) / (dts / 1000.0)
    vel_order = np.argsort(enc_vel)
    quarter = len(vel_order) // 4
    slow_idx = vel_order[:quarter] + 1
    fast_idx = vel_order[-quarter:] + 1

    motion_dir = np.diff(enc_roll)
    imu_offset = enc_roll[1:] - imu_roll[1:]
    moving = np.abs(motion_dir) > 1.0
    trail_pct = 0.0
    if np.sum(moving) > 0:
        trailing = (motion_dir[moving] * imu_offset[moving]) > 0
        trail_pct = float(np.sum(trailing) / np.sum(moving) * 100)

    sign_changes = np.diff(np.sign(errors))
    osc_freq = float(np.sum(sign_changes != 0)) / (2.0 * duration_s) if duration_s > 0 else 0.0

    ang_iterm_limit  = get_iterm_limit(config, "angle_pid")
    rate_iterm_limit = get_iterm_limit(config, "rate_pid")

    sensor = {
        "mae":                float(np.mean(abs_errors)),
        "rms_error":          float(np.sqrt(np.mean(errors ** 2))),
        "max_ae":             float(np.max(abs_errors)),
        "bias":               float(np.mean(errors)),
        "mae_fast":           float(np.mean(abs_errors[fast_idx])) if len(fast_idx) > 0 else 0.0,
        "mae_slow":           float(np.mean(abs_errors[slow_idx])) if len(slow_idx) > 0 else 0.0,
        "correlation":        float(np.corrcoef(enc_roll, imu_roll)[0, 1])
                              if np.std(enc_roll) > 0 and np.std(imu_roll) > 0 else 0.0,
        "trail_pct":          trail_pct,
        "enc_range":          float(np.ptp(enc_roll)),
        "imu_range":          float(np.ptp(imu_roll)),
        "whole_run_enc_mae":  float(np.mean(np.abs(enc_roll - setpoint))),
        "enc_bias_setpoint":  float(np.mean(enc_roll - setpoint)),
        "enc_max_ae_setpoint":float(np.max(np.abs(enc_roll - setpoint))),
    }

    control = {
        "osc_freq_hz":           osc_freq,
        "ang_windup_events":     int(np.sum(np.abs(cols["ANG_I"])  >= ang_iterm_limit  * 0.5)),
        "ang_windup_threshold":  ang_iterm_limit  * 0.5,
        "rate_windup_events":    int(np.sum(np.abs(cols["RATE_I"]) >= rate_iterm_limit * 0.5)),
        "rate_windup_threshold": rate_iterm_limit * 0.5,
        "imu_mae_setpoint":      float(np.mean(np.abs(imu_roll - setpoint))),
    }

    rate = {
        "n_samples":    n,
        "duration_s":   duration_s,
        "actual_hz":    actual_hz,
        "dt_mean_ms":   float(np.mean(dts)),
        "dt_median_ms": float(np.median(dts)),
    }

    return rate, sensor, control


# ---------------------------------------------------------------------------
# Printing
# ---------------------------------------------------------------------------

def _row(name, value, fmt=".2f"):
    print(f"  {name:<38} {value:>10{fmt}}")


def print_config_summary(config):
    vehicle = config.get("vehicle", {})
    angle = vehicle.get("angle_pid", {})
    rate  = vehicle.get("rate_pid", {})
    motor = vehicle.get("motor", {})
    imu   = vehicle.get("imu", {})
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


def print_canonical_kpis(kpis, setpoint):
    print("  --- Canonical KPIs (from score_flight) ---")
    print(f"  {'Reached setpoint':<38} {'YES' if kpis['reached'] else 'NO':>10}")
    t_to  = f"{kpis['time_to_s']:.1f}"  if kpis["reached"] else "-"
    h_mae = f"{kpis['hold_mae']:.2f}"   if kpis["reached"] else "-"
    print(f"  {'T->setpoint (s)':<38} {t_to:>10}")
    print(f"  {'HoldMAE (°), post-reach':<38} {h_mae:>10}")
    print(f"  {'T@setpoint (s)':<38} {kpis['time_at_s']:>10.1f}")


def print_profile(label, rate, sensor, control, config, kpis, setpoint):
    w = 58
    print("=" * w)
    print(f"  {label}")
    print("-" * w)
    print_config_summary(config)
    print()

    if kpis is not None:
        print_canonical_kpis(kpis, setpoint)
        print()

    print("  --- Sample Rate ---")
    _row("Samples",        rate["n_samples"],    ".0f")
    _row("Duration (s)",   rate["duration_s"],   ".1f")
    _row("Achieved Hz",    rate["actual_hz"],    ".1f")
    _row("Mean dt (ms)",   rate["dt_mean_ms"],   ".1f")
    _row("Median dt (ms)", rate["dt_median_ms"], ".1f")

    print("\n  --- Sensor Health (IMU vs ENC) ---")
    _row("MAE (overall)",     sensor["mae"])
    _row("MAE (fast motion)", sensor["mae_fast"])
    _row("MAE (slow motion)", sensor["mae_slow"])
    _row("Max AE",            sensor["max_ae"])
    _row("RMS Error",         sensor["rms_error"])
    _row("Bias (IMU-ENC)",    sensor["bias"])

    print(f"\n  --- Setpoint Error (vs {setpoint:+.0f}°) ---")
    _row("Whole-run ENC MAE (°)", sensor["whole_run_enc_mae"])
    print("  (includes the rise — not comparable to HoldMAE)")
    _row("IMU MAE (°)",           control["imu_mae_setpoint"])
    _row("ENC Bias (°)",          sensor["enc_bias_setpoint"])
    _row("ENC Max AE (°)",        sensor["enc_max_ae_setpoint"])

    print("\n  --- Correlation & Tracking ---")
    _row("Pearson r",             sensor["correlation"], ".4f")
    _row("IMU trails motion (%)", sensor["trail_pct"],   ".1f")
    _row("Encoder range (°)",     sensor["enc_range"],   ".1f")
    _row("IMU range (°)",         sensor["imu_range"],   ".1f")

    print("\n  --- Oscillation & Windup ---")
    print(f"  {'Oscillation freq (Hz)':<38} {control['osc_freq_hz']:>10.2f}")
    _row("Angle windup events",    control["ang_windup_events"],    ".0f")
    print(f"  {'Angle windup threshold':<38} {control['ang_windup_threshold']:>10.1f}")
    _row("Rate windup events",     control["rate_windup_events"],   ".0f")
    print(f"  {'Rate windup threshold':<38} {control['rate_windup_threshold']:>10.1f}")

    print("=" * w)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: python .claude/skills/analyse-flight/scripts/profile_flight.py "
                 "test_runs/flights/<flight_id>")

    cols, config, label = load_run(sys.argv[1])

    p = Path(sys.argv[1])
    run_dir = p if p.is_dir() else p.parent
    setpoint = load_setpoint(run_dir)

    rate, sensor, control = compute_stats(cols, config, setpoint)
    kpis = compute_kpis(cols_to_samples(cols), setpoint)

    print(f"Loaded {label}: {rate['n_samples']} samples, {rate['duration_s']:.1f}s\n")
    print_profile(label, rate, sensor, control, config, kpis, setpoint)


if __name__ == "__main__":
    main()