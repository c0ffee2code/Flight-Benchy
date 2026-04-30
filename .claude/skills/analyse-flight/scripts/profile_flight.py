"""
Step 3 of the analyse-flight pipeline. Run only on flights that passed score_flight.py.

Prints grouped diagnostic metrics to console. Reads config.yaml alongside log.csv;
config.yaml is mandatory — the script exits with an error if it is absent or if any
required field (iterm_limit) is missing.

Metric groups:
  Canonical KPIs         — echoed from score_flight.compute_kpis() when encoder present
  Sample Rate            — Achieved Hz; should match 1000 / (INNER_INTERVAL_MS × TELEMETRY_SAMPLE_EVERY)
  Sensor Health          — IMU-vs-encoder MAE, bias, Pearson r, trail %. Only emitted when
  (IMU vs ENC)             ENC_* columns are present in the CSV (omitted for pitch/yaw runs).
  Setpoint Error         — Whole-run ENC MAE against 0° when encoder present; IMU MAE otherwise.
  (vs 0°)                  "Whole-run ENC MAE" includes the rise — not comparable to HoldMAE.
  Correlation &          — Pearson r, IMU trailing encoder. Omitted when encoder absent.
  Tracking
  Oscillation &          — osc_freq_hz: sign-change rate of tracking error (ENC-based when encoder
  Windup                   present, IMU-based otherwise). Windup events: ticks where |I-term| ≥ 50%
                           of iterm_limit from config.yaml.

Run from project root:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs/flights/<flight_id>
"""

import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from score_flight import compute_kpis, HORIZONTAL_THRESHOLD_DEG  # noqa: E402

try:
    import yaml
except ImportError:
    sys.exit("pyyaml not installed — run: pip install pyyaml")


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_run(path_str):
    p = Path(path_str)
    csv_path = (p / "log.csv") if p.is_dir() else p
    cfg_path = (p / "config.yaml") if p.is_dir() else (p.parent / "config.yaml")
    label = p.name if p.is_dir() else p.stem

    if not csv_path.exists():
        sys.exit(f"log.csv not found: {csv_path}")
    if not cfg_path.exists():
        sys.exit(f"config.yaml not found in {p} — mandatory for profile")

    with open(cfg_path) as f:
        config = yaml.safe_load(f)

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
        return float(config[pid_key]["iterm_limit"])
    except (KeyError, TypeError):
        sys.exit(f"config.yaml missing {pid_key}.iterm_limit — required for windup computation")


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def compute_stats(cols, config):
    has_encoder = "ENC_QR" in cols
    n = len(cols["T_MS"])
    t_ms = cols["T_MS"]
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0
    actual_hz = (n - 1) / duration_s if duration_s > 0 else 0.0
    dts = np.diff(t_ms)

    sensor = None
    if has_encoder:
        enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
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

        osc_ref = errors
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
            "whole_run_enc_mae":  float(np.mean(np.abs(enc_roll))),
            "enc_bias_setpoint":  float(np.mean(enc_roll)),
            "enc_max_ae_setpoint":float(np.max(np.abs(enc_roll))),
        }
    else:
        osc_ref = imu_roll

    sign_changes = np.diff(np.sign(osc_ref))
    osc_freq = float(np.sum(sign_changes != 0)) / (2.0 * duration_s) if duration_s > 0 else 0.0

    ang_iterm_limit  = get_iterm_limit(config, "angle_pid")
    rate_iterm_limit = get_iterm_limit(config, "rate_pid")

    control = {
        "osc_freq_hz":           osc_freq,
        "ang_windup_events":     int(np.sum(np.abs(cols["ANG_I"])  >= ang_iterm_limit  * 0.5)),
        "ang_windup_threshold":  ang_iterm_limit  * 0.5,
        "rate_windup_events":    int(np.sum(np.abs(cols["RATE_I"]) >= rate_iterm_limit * 0.5)),
        "rate_windup_threshold": rate_iterm_limit * 0.5,
        "imu_mae_setpoint":      float(np.mean(np.abs(imu_roll))),
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
    angle = config.get("angle_pid", {})
    rate  = config.get("rate_pid", {})
    motor = config.get("motor", {})
    imu   = config.get("imu", {})
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


def print_canonical_kpis(kpis):
    print("  --- Canonical KPIs (from score_flight) ---")
    print(f"  {'Reached horizontal':<38} {'YES' if kpis['reached'] else 'NO':>10}")
    t_to  = f"{kpis['time_to_s']:.1f}"  if kpis["reached"] else "-"
    h_mae = f"{kpis['hold_mae']:.2f}"   if kpis["reached"] else "-"
    print(f"  {'T->0 (s)':<38} {t_to:>10}")
    print(f"  {'HoldMAE (°), post-reach':<38} {h_mae:>10}")
    print(f"  {'T@0 (s)':<38} {kpis['time_at_s']:>10.1f}")


def print_profile(label, rate, sensor, control, config, kpis):
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

    if sensor is not None:
        print("\n  --- Sensor Health (IMU vs ENC) ---")
        _row("MAE (overall)",     sensor["mae"])
        _row("MAE (fast motion)", sensor["mae_fast"])
        _row("MAE (slow motion)", sensor["mae_slow"])
        _row("Max AE",            sensor["max_ae"])
        _row("RMS Error",         sensor["rms_error"])
        _row("Bias (IMU-ENC)",  sensor["bias"])

        print("\n  --- Setpoint Error (vs 0°) ---")
        _row("Whole-run ENC MAE (°)", sensor["whole_run_enc_mae"])
        print("  (includes the rise — not comparable to HoldMAE)")
        _row("IMU MAE (°)",           control["imu_mae_setpoint"])
        _row("ENC Bias (°)",          sensor["enc_bias_setpoint"])
        _row("ENC Max AE (°)",        sensor["enc_max_ae_setpoint"])

        print("\n  --- Correlation & Tracking ---")
        _row("Pearson r",             sensor["correlation"], ".4f")
        _row("IMU trails motion (%)", sensor["trail_pct"],   ".1f")
        _row("Encoder range (°)",    sensor["enc_range"],   ".1f")
        _row("IMU range (°)",        sensor["imu_range"],   ".1f")
    else:
        print("\n  (Sensor Health skipped — no encoder columns in CSV)")
        print("\n  --- Setpoint Error (vs 0°) ---")
        _row("IMU MAE (°)", control["imu_mae_setpoint"])

    print("\n  --- Oscillation & Windup ---")
    osc_src = "ENC-based" if sensor is not None else "IMU-based"
    print(f"  {'Oscillation freq (Hz)':<38} {control['osc_freq_hz']:>10.2f}  ({osc_src})")
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
    rate, sensor, control = compute_stats(cols, config)
    kpis = compute_kpis(cols_to_samples(cols)) if "ENC_QR" in cols else None

    print(f"Loaded {label}: {rate['n_samples']} samples, {rate['duration_s']:.1f}s\n")
    print_profile(label, rate, sensor, control, config, kpis)


if __name__ == "__main__":
    main()