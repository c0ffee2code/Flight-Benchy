"""
Telemetry invariants gate -- run after plots, before verdict.

Answers: "Do sign and orientation contracts hold?"

Hard FAILs (I1-I6, I9, I11): sign/validity contracts; a violation makes KPIs meaningless.
WARNs only  (I7, I8, I10): quality/perf flags; KPIs remain valid even when these fire.

Exit 0 if no hard FAIL; exit 1 if any hard FAIL.
Writes analysis/invariants.json.

Run from project root:
  python pipelines/flight-analyser/scripts/invariants.py test_runs/flights/<flight_id>
"""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from configuration_loader import load_configuration  # noqa: E402
from flight_data_loader import (  # noqa: E402
    load_flight,
    detect_reach_event,
    detect_hold_window,
)
from specification_loader import load_specification  # noqa: E402


_TIMING_TOLERANCE    = 0.20   # 20% band around expected period
_P99_DT_FACTOR       = 3.0    # I6: FAIL if p99 dt exceeds this multiple of expected period
_MAX_DT_WARN_MS      = 25.0   # I6: WARN if any single cycle exceeds this (single spike)
_MIN_TRANSIENT_N     = 10     # skip I5 if transient shorter than this
_SAT_WARN_THRESHOLD  = 0.30   # I7: WARN if saturation fraction exceeds this
_BIAS_WARN_THRESHOLD = 1.5    # I8: WARN if hold-phase RMS exceeds this (deg)


# ---------------------------------------------------------------------------
# Result builders
# ---------------------------------------------------------------------------

def _r(name, status, value, threshold, detail=None):
    return {
        "name": name, "status": status,
        "value": round(float(value), 4) if value is not None else None,
        "threshold": threshold, "detail": detail,
    }


def _pass(name, value, threshold):
    return _r(name, "PASS", value, threshold)


def _fail(name, value, threshold, detail):
    return _r(name, "FAIL", value, threshold, detail)


def _warn(name, value, threshold, detail):
    return _r(name, "WARN", value, threshold, detail)


def _skip(name, threshold, detail):
    return _r(name, "SKIP", None, threshold, detail)


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _smooth(x, window=3):
    k = np.ones(window) / window
    return np.convolve(x, k, mode='same')


def _central_diff(values, times):
    d = np.empty_like(values)
    d[1:-1] = (values[2:] - values[:-2]) / (times[2:] - times[:-2])
    d[0]    = (values[1]  - values[0])   / (times[1]  - times[0])
    d[-1]   = (values[-1] - values[-2])  / (times[-1] - times[-2])
    return d


def _pearson(a, b):
    if len(a) < 2:
        return float("nan")
    v = float(np.corrcoef(a, b)[0, 1])
    return v if np.isfinite(v) else float("nan")


def _rms(a):
    return float(np.sqrt(np.mean(np.asarray(a, dtype=float) ** 2)))


# ---------------------------------------------------------------------------
# Invariant checks
# ---------------------------------------------------------------------------

def _i1(fd):
    """I1: IMU-encoder position agreement (FAIL)."""
    c = _pearson(fd.imu_roll, fd.enc_roll)
    threshold = "> +0.95"
    if np.isnan(c):
        return _skip("imu-enc-position-agreement", threshold,
                     "Zero variance -- could not compute.")
    if c <= 0.95:
        return _fail("imu-enc-position-agreement", c, threshold,
                     f"corr={c:+.3f}; expect > +0.95. "
                     f"Check imu_invert or encoder_invert in config.")
    return _pass("imu-enc-position-agreement", c, threshold)


def _i2(fd):
    """I2: Gyro rate sign contract (FAIL). Contract: gyro_x = -phi_dot."""
    times = fd.t_ms / 1000.0
    # Do not smooth before differentiating here: np.convolve zero-pads boundaries,
    # creating spurious velocity spikes at run edges that corrupt the correlation.
    # Signal SNR (~20:1 at 76ms sample interval) is sufficient without smoothing.
    denc  = _central_diff(fd.enc_roll, times)
    c     = _pearson(fd.gyro_x, denc)
    threshold = "< -0.4"
    if np.isnan(c):
        return _skip("gyro-rate-sign", threshold, "Could not compute correlation.")
    if c >= -0.4:
        return _fail("gyro-rate-sign", c, threshold,
                     f"corr={c:+.3f}; expect < -0.4. Contract: gyro_x = -phi_dot. "
                     f"Check imu_invert, encoder_invert, or gyro negation in flight.py.")
    return _pass("gyro-rate-sign", c, threshold)


def _i3(fd):
    """I3: IMU-encoder rate agreement (FAIL)."""
    times = fd.t_ms / 1000.0
    dimu  = _central_diff(_smooth(fd.imu_roll), times)
    denc  = _central_diff(_smooth(fd.enc_roll), times)
    c     = _pearson(dimu, denc)
    threshold = "> +0.5"
    if np.isnan(c):
        return _skip("imu-enc-rate-agreement", threshold, "Could not compute correlation.")
    if c <= 0.5:
        return _fail("imu-enc-rate-agreement", c, threshold,
                     f"corr={c:+.3f}; expect > +0.5. IMU rate diverges from encoder rate.")
    return _pass("imu-enc-rate-agreement", c, threshold)


def _i4(fd, setpoint):
    """I4: Angle error convention (FAIL). ang_err should track enc_roll - setpoint."""
    c         = _pearson(fd.ang_err, fd.enc_roll - setpoint)
    threshold = "> +0.9"
    if np.isnan(c):
        return _skip("angle-error-convention", threshold, "Could not compute correlation.")
    if c <= 0.9:
        return _fail("angle-error-convention", c, threshold,
                     f"corr={c:+.3f}; expect > +0.9. "
                     f"Setpoint sign regression (2026-05-02 bug class).")
    return _pass("angle-error-convention", c, threshold)


def _i5(fd, reach_event):
    """I5: Actuation direction over transient (FAIL). corr(m2-m1, d2(enc)) < 0."""
    end_idx   = reach_event.start_idx if reach_event is not None else len(fd.enc_roll) // 2
    threshold = "< 0 (transient only)"
    if end_idx < _MIN_TRANSIENT_N:
        return _skip("actuation-direction", threshold,
                     f"Transient only {end_idx} samples (< {_MIN_TRANSIENT_N}).")
    times  = fd.t_ms / 1000.0
    enc_s  = _smooth(fd.enc_roll)
    denc   = _central_diff(enc_s, times)
    d2enc  = _central_diff(_smooth(denc), times)
    diff_m = (fd.m2 - fd.m1)[:end_idx]
    c      = _pearson(diff_m, d2enc[:end_idx])
    if np.isnan(c):
        return _skip("actuation-direction", threshold, "Could not compute correlation.")
    if c >= 0:
        return _fail("actuation-direction", c, threshold,
                     f"corr(m2-m1, d2(enc))={c:+.3f}; expect < 0. "
                     f"Check mixer sign or motor wiring.")
    return _pass("actuation-direction", c, threshold)


def _i6(fd, rate_hz):
    """I6: Loop timing. Median and p99 are FAIL; max spike is WARN.

    p99 catches systematic tail jitter that degrades D-term accuracy.
    max catches isolated spikes (single GC pause) -- flagged but not blocking.
    """
    expected_ms  = 1000.0 / rate_hz
    median_dt    = float(np.median(fd.dt_ms))
    p99_dt       = float(np.percentile(fd.dt_ms, 99))
    max_dt       = float(np.max(fd.dt_ms))
    tol_lo       = expected_ms * (1.0 - _TIMING_TOLERANCE)
    tol_hi       = expected_ms * (1.0 + _TIMING_TOLERANCE)
    p99_ceiling  = expected_ms * _P99_DT_FACTOR
    threshold    = (
        f"median in [{tol_lo:.1f},{tol_hi:.1f}]ms, "
        f"p99 < {p99_ceiling:.1f}ms (FAIL), "
        f"max < {_MAX_DT_WARN_MS:.0f}ms (WARN)"
    )

    fail_issues = []
    if not (tol_lo <= median_dt <= tol_hi):
        fail_issues.append(
            f"median dt={median_dt:.1f}ms outside [{tol_lo:.1f},{tol_hi:.1f}]ms "
            f"(expected {expected_ms:.1f}ms for {rate_hz}Hz)"
        )
    if p99_dt >= p99_ceiling:
        fail_issues.append(
            f"p99 dt={p99_dt:.1f}ms >= {p99_ceiling:.1f}ms ({_P99_DT_FACTOR:.0f}x expected)"
        )
    if fail_issues:
        return _fail("loop-timing", round(p99_dt, 2), threshold, "; ".join(fail_issues))

    if max_dt >= _MAX_DT_WARN_MS:
        return _warn("loop-timing", round(max_dt, 2), threshold,
                     f"max dt={max_dt:.1f}ms >= {_MAX_DT_WARN_MS:.0f}ms "
                     f"(isolated spike; p99={p99_dt:.1f}ms is clean)")

    return _pass("loop-timing", round(median_dt, 2), threshold)


def _i7(fd, reach_event, hold_window, throttle_min, throttle_max, rate_output_limit):
    """I7: Saturation report (WARN only)."""
    if hold_window is not None:
        start_idx = hold_window.start_idx
        phase     = "hold"
    elif reach_event is not None:
        start_idx = reach_event.start_idx
        phase     = "post-reach"
    else:
        start_idx = 0
        phase     = "full-run"

    pid_out   = fd.pid_out[start_idx:]
    m1        = fd.m1[start_idx:]
    m2        = fd.m2[start_idx:]
    n         = len(pid_out)
    threshold = f"< {_SAT_WARN_THRESHOLD:.0%} (WARN)"

    if n == 0:
        return _skip("saturation-report", threshold, "Empty window.")

    pid_sat = float(np.mean(np.abs(pid_out) >= rate_output_limit * 0.99))
    m1_sat  = float(np.mean((m1 >= throttle_max * 0.99) | (m1 <= throttle_min * 1.01)))
    m2_sat  = float(np.mean((m2 >= throttle_max * 0.99) | (m2 <= throttle_min * 1.01)))
    max_sat = max(pid_sat, m1_sat, m2_sat)
    detail  = (
        f"pid_out_sat={pid_sat:.1%}, m1_sat={m1_sat:.1%}, m2_sat={m2_sat:.1%} "
        f"({phase}, {n} samples)"
    )
    if max_sat > _SAT_WARN_THRESHOLD:
        return _warn("saturation-report", round(max_sat, 4), threshold, detail)
    return _pass("saturation-report", round(max_sat, 4), threshold)


def _i8(fd, hold_window):
    """I8: IMU-encoder hold bias (WARN only)."""
    threshold = f"rms < {_BIAS_WARN_THRESHOLD}deg (WARN)"
    if hold_window is None:
        return _skip("imu-enc-hold-bias", threshold, "No confirmed hold window.")
    diff    = fd.imu_roll[hold_window.start_idx:] - fd.enc_roll[hold_window.start_idx:]
    rms_val = _rms(diff)
    bias    = float(np.mean(diff))
    detail  = f"rms={rms_val:.3f}deg, bias={bias:+.3f}deg ({len(diff)} samples in hold)"
    if rms_val > _BIAS_WARN_THRESHOLD:
        return _warn("imu-enc-hold-bias", round(rms_val, 4), threshold, detail)
    return _pass("imu-enc-hold-bias", round(rms_val, 4), threshold)


def _i9(fd, start_angle_deg):
    """I9: Start angle sign and magnitude agreement (FAIL)."""
    first     = float(fd.enc_roll[0])
    same_sign = (
        (first > 1.0  and start_angle_deg > 1.0)
        or (first < -1.0 and start_angle_deg < -1.0)
        or (abs(first) <= 1.0 and abs(start_angle_deg) <= 1.0)
    )
    within_10 = abs(first - start_angle_deg) <= 10.0
    threshold = "same sign and within 10deg of config start_angle_deg"
    issues    = []
    if not same_sign:
        issues.append(
            f"enc[0]={first:+.1f}deg vs start_angle={start_angle_deg:+.1f}deg: sign mismatch"
        )
    if not within_10:
        issues.append(
            f"enc[0]={first:+.1f}deg outside start_angle={start_angle_deg:+.1f}deg +/-10deg"
        )
    if issues:
        return _fail("start-angle-sign", round(first, 2), threshold, "; ".join(issues))
    return _pass("start-angle-sign", round(first, 2), threshold)


def _i10(fd, lead_ms):
    """I10: Feedforward direction (WARN only). Reports all three RMS variants."""
    threshold = "code sign RMS <= no-FF RMS (WARN)"
    if lead_ms is None or lead_ms <= 0:
        return _skip("feedforward-direction", threshold,
                     "feedforward_lead_ms not set or zero.")
    lead_s        = lead_ms / 1000.0
    rms_no_ff     = _rms(fd.imu_roll - fd.enc_roll)
    rms_code_sign = _rms(fd.imu_roll + fd.gyro_x * lead_s - fd.enc_roll)
    rms_flip_sign = _rms(fd.imu_roll - fd.gyro_x * lead_s - fd.enc_roll)
    detail        = (
        f"no-FF={rms_no_ff:.4f}, code(+)={rms_code_sign:.4f}, "
        f"flip(-)={rms_flip_sign:.4f} deg"
    )
    if rms_code_sign > rms_no_ff:
        return _warn("feedforward-direction", round(rms_code_sign, 4), threshold,
                     detail + ". Code sign (+) makes estimate worse. "
                     "See F1 in REVIEW-2026-06-10. "
                     "Fix: change (+) to (-) in feedforward_roll in flight.py.")
    return _pass("feedforward-direction", round(rms_code_sign, 4), threshold)


def _i11(fd):
    """I11: Mixer output sign (FAIL). corr(pid_out, m2-m1) > 0.9.

    LeverMixer: m2 = base + pid_out, m1 = base - pid_out, so m2-m1 = 2*pid_out exactly.
    Expected correlation ~1.0. Inverted mixer gives ~-1.0.
    Catches software mixer sign inversion -- the one root cause not covered by I1-I10.
    """
    c         = _pearson(fd.pid_out, fd.m2 - fd.m1)
    threshold = "> +0.9"
    if np.isnan(c):
        return _skip("mixer-output-sign", threshold, "Could not compute correlation.")
    if c <= 0.9:
        return _fail("mixer-output-sign", c, threshold,
                     f"corr(pid_out, m2-m1)={c:+.3f}; expect > +0.9. "
                     f"Correct LeverMixer gives ~+1.0; inverted mixer gives ~-1.0. "
                     f"Check mixer.py sign or motor wiring.")
    return _pass("mixer-output-sign", c, threshold)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: invariants.py test_runs/flights/<flight_id>")

    run_dir  = Path(sys.argv[1])
    csv_path = run_dir / "log.csv"

    cfg  = load_configuration(run_dir)
    spec = load_specification(run_dir)
    fd   = load_flight(csv_path)

    reach_event = detect_reach_event(fd, cfg.setpoint_roll_deg, spec.tolerance_deg)
    hold_window = detect_hold_window(fd, reach_event, cfg.setpoint_roll_deg, spec.tolerance_deg)

    results = [
        _i1(fd),
        _i2(fd),
        _i3(fd),
        _i4(fd, cfg.setpoint_roll_deg),
        _i5(fd, reach_event),
        _i6(fd, cfg.loops.rate.frequency_hz),
        _i7(fd, reach_event, hold_window,
            cfg.motor.throttle_min, cfg.motor.throttle_max,
            cfg.loops.rate.pid.output_limit),
        _i8(fd, hold_window),
        _i9(fd, cfg.start_angle_deg),
        _i10(fd, cfg.feedforward_lead_ms),
        _i11(fd),
    ]

    any_fail = any(r["status"] == "FAIL" for r in results)
    any_warn = any(r["status"] == "WARN" for r in results)

    output = {
        "passed":    not any_fail,
        "flight_id": run_dir.name,
        "invariants": results,
    }

    analysis_dir = run_dir / "analysis"
    analysis_dir.mkdir(exist_ok=True)
    out_path = analysis_dir / "invariants.json"
    out_path.write_text(json.dumps(output, indent=2), encoding="utf-8")

    if any_fail:
        fail_names = [r["name"] for r in results if r["status"] == "FAIL"]
        print(f"FAIL -- {', '.join(fail_names)} -- wrote {out_path}")
        sys.exit(1)

    if any_warn:
        warn_names = [r["name"] for r in results if r["status"] == "WARN"]
        print(f"WARN -- {', '.join(warn_names)} -- wrote {out_path}")
    else:
        print(f"PASS -- wrote {out_path}")
    sys.exit(0)


if __name__ == "__main__":
    main()
