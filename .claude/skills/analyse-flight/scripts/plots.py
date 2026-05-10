"""
Step 2 of the analyse-flight pipeline.

Dispatcher for all diagnostic figures. One flight folder in, one or more plot files out.

Currently supported plot types (--type argument, default: all):
  timeseries     — 5-subplot time-series view              → timeseries.png
  phase          — Phase portrait (angle vs angular rate)   → phase_portrait.png
  histogram      — Hold-error distribution (post-settle)    → hold_error_distribution.png
  spectrum       — PSD of hold-window error                 → spectrum.png
  step_response  — Annotated step response (control-theory) → step_response.png

Run from project root:
  python .claude/skills/analyse-flight/scripts/plots.py test_runs/flights/<flight_id>
  python .claude/skills/analyse-flight/scripts/plots.py test_runs/flights/<flight_id> --type phase

TODO: separate data computation from rendering. Currently each plot_* function loads,
slices, and transforms data inline. A cleaner architecture would have a compute layer
(pure functions, numpy in / dict out) and a render layer (takes computed dict, returns
fig). This makes each plot independently testable and makes the data available for
non-visual consumers (e.g. profile_flight.py, future comparison tools).
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.collections import LineCollection
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

sys.path.insert(0, str(Path(__file__).parent))
from score_flight import (  # noqa: E402
    HORIZONTAL_THRESHOLD_DEG,
    load_setpoint,
    load_motor_limits,
    compute_kpis,
)


def load_run(path_str):
    p = Path(path_str)
    csv_path = (p / "log.csv") if p.is_dir() else p
    label = p.name if p.is_dir() else p.stem
    if not csv_path.exists():
        sys.exit(f"CSV not found: {csv_path}")
    rows = []
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        sys.exit(f"Empty CSV: {csv_path}")
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    return csv_path, cols, label


def quat_to_roll(qr, qi):
    return np.degrees(2.0 * np.arctan2(qi, qr))


def _time_formatter(duration_s):
    if duration_s >= 60:
        def fmt(x, _):
            m, s = divmod(int(x), 60)
            return f"{m}:{s:02d}"
        return mticker.FuncFormatter(fmt), "Time (m:ss)"
    return None, "Time (s)"


# ---------------------------------------------------------------------------
# Timeseries
# ---------------------------------------------------------------------------

def plot_run(cols, label, setpoint, kpis, throttle_min, throttle_max):
    t_s = (cols["T_MS"] - cols["T_MS"][0]) / 1000.0
    duration_s = t_s[-1]
    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
    fig.suptitle(f"Flight Benchy — {label}  ({duration_s:.1f}s)", fontsize=13)

    ax1.axhspan(setpoint - HORIZONTAL_THRESHOLD_DEG,
                setpoint + HORIZONTAL_THRESHOLD_DEG,
                color="gray", alpha=0.08, zorder=0)
    ax1.axhline(setpoint, color="gray", linewidth=0.8, linestyle="--", label=f"Setpoint ({setpoint:+.0f}°)")
    ax1.plot(t_s, enc_roll, label="Encoder", linewidth=0.8)
    ax1.plot(t_s, imu_roll, label="IMU", linewidth=0.8, alpha=0.6)
    ax1.set_ylabel("Roll (deg)")
    ax1.set_title("Angle Tracking")
    ax1.grid(True, alpha=0.3)

    ax2.plot(t_s, cols["GYRO_X"], linewidth=0.6, color="tab:blue", alpha=0.7, label="Gyro X (actual)")
    ax2.plot(t_s, cols["RATE_SP"], linewidth=0.8, color="tab:orange", label="Rate setpoint")
    ax2.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax2.set_ylabel("Rate (deg/s)")
    ax2.set_title("Rate Tracking (setpoint vs actual)")
    ax2.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax2.grid(True, alpha=0.3)

    ax3.plot(t_s, cols["ANG_P"], label="P", linewidth=0.8, color="tab:blue")
    ax3.plot(t_s, cols["ANG_I"], label="I", linewidth=0.8, color="tab:orange")
    ax3.plot(t_s, cols["ANG_D"], label="D", linewidth=0.8, color="tab:green")
    ax3.set_ylabel("Output (deg/s)")
    ax3.set_title("Angle PID (outer loop)")
    ax3.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax3.grid(True, alpha=0.3)

    ax4.plot(t_s, cols["RATE_P"], label="P", linewidth=0.8, color="tab:blue")
    ax4.plot(t_s, cols["RATE_I"], label="I", linewidth=0.8, color="tab:orange")
    ax4.plot(t_s, cols["RATE_D"], label="D", linewidth=0.8, color="tab:green")
    ax4.set_ylabel("Output (throttle)")
    ax4.set_title("Rate PID (inner loop)")
    ax4.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax4.grid(True, alpha=0.3)

    ax5.plot(t_s, cols["M1"], label="M1", linewidth=0.8)
    ax5.plot(t_s, cols["M2"], label="M2", linewidth=0.8)
    ax5.axhline(throttle_max, color="red", linestyle=":", linewidth=0.6, alpha=0.5, zorder=0)
    ax5.axhline(throttle_min, color="red", linestyle=":", linewidth=0.6, alpha=0.5, zorder=0)
    ax5.set_ylabel("Throttle")
    ax5.set_title("Motor Output")
    ax5.grid(True, alpha=0.3)

    if kpis is not None:
        if kpis["hold_start_idx"] is not None:
            t_reach = (cols["T_MS"][kpis["hold_start_idx"]] - cols["T_MS"][0]) / 1000.0
            for ax in (ax1, ax2, ax3, ax4, ax5):
                ax.axvline(t_reach, color="tab:green", linewidth=0.6,
                           linestyle="--", alpha=0.5, zorder=0)
        if kpis["settling_time_s"] is not None:
            for ax in (ax1, ax2, ax3, ax4, ax5):
                ax.axvspan(kpis["settling_time_s"], t_s[-1],
                           color="tab:green", alpha=0.06, zorder=0)

    h1, _ = ax1.get_legend_handles_labels()
    h1 += [Patch(facecolor='gray', alpha=0.5, edgecolor='none',
                 label=f'±{HORIZONTAL_THRESHOLD_DEG:.0f}° band')]
    if kpis is not None:
        if kpis["hold_start_idx"] is not None:
            h1 += [Line2D([0], [0], color='tab:green', linewidth=0.8,
                          linestyle='--', alpha=0.7, label='T→SP')]
        if kpis["settling_time_s"] is not None:
            h1 += [Patch(facecolor='tab:green', alpha=0.4, edgecolor='none',
                         label='Settled hold')]
    ax1.legend(handles=h1, loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8)

    h5, _ = ax5.get_legend_handles_labels()
    h5 += [Line2D([0], [0], color='red', linewidth=0.8, linestyle=':',
                  alpha=0.7, label='Throttle limits')]
    ax5.legend(handles=h5, loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8)

    fmt, xlabel = _time_formatter(duration_s)
    ax5.set_xlabel(xlabel)
    if fmt:
        ax5.xaxis.set_major_formatter(fmt)

    fig.tight_layout(rect=[0, 0, 0.82, 1])
    return fig


# ---------------------------------------------------------------------------
# Phase portrait
# ---------------------------------------------------------------------------

def plot_phase_portrait(cols, label, setpoint, kpis):
    """
    Phase portrait: encoder angle (x-axis) vs angular rate (y-axis), coloured by time.

    Rate source: gyro_x is used directly rather than numerically differentiating the
    encoder angle. gyro_x is already logged, is exactly what the inner control loop
    sees, and avoids the noise amplification that finite differences introduce at the
    50 ms sample interval.

    Settled-zone shading: only the angle axis is shaded (±HORIZONTAL_THRESHOLD_DEG
    vertical band). A rate-axis threshold is not defined as a KPI and would be an
    arbitrary choice, giving a false impression of precision about what "settled" means
    in rate space.
    """
    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    gyro_x   = cols["GYRO_X"]
    t_s      = (cols["T_MS"] - cols["T_MS"][0]) / 1000.0

    points   = np.array([enc_roll, gyro_x]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm     = plt.Normalize(t_s[0], t_s[-1])
    lc       = LineCollection(segments, cmap="plasma", norm=norm, linewidth=0.8, alpha=0.8)
    lc.set_array(t_s[:-1])

    fig, ax = plt.subplots(1, 1, figsize=(7, 7))
    fig.suptitle(f"Phase Portrait — {label}", fontsize=13)

    ax.add_collection(lc)
    ax.autoscale_view()

    # Setpoint band — angle axis only (see docstring)
    ax.axvspan(setpoint - HORIZONTAL_THRESHOLD_DEG,
               setpoint + HORIZONTAL_THRESHOLD_DEG,
               color="gray", alpha=0.08, zorder=0)
    ax.axvline(setpoint, color="gray", linewidth=0.8, linestyle="--", alpha=0.6, zorder=1)
    ax.axhline(0,        color="gray", linewidth=0.8, linestyle="--", alpha=0.6, zorder=1)
    ax.plot(setpoint, 0, "k+", markersize=10, markeredgewidth=1.5, zorder=2,
            label=f"Target state ({setpoint:+.0f}°, 0 °/s)")

    if kpis is not None and kpis["hold_start_idx"] is not None:
        idx = kpis["hold_start_idx"]
        ax.plot(enc_roll[idx], gyro_x[idx], "o", color="tab:green",
                markersize=6, zorder=3, label="T→SP")

    cb = fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label("Time (s)", fontsize=8)

    ax.set_xlabel("Encoder angle (°)")
    ax.set_ylabel("Angular rate (°/s)")
    ax.set_title("Angle vs Angular Rate")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Hold-error distribution
# ---------------------------------------------------------------------------

def plot_hold_error_distribution(cols, label, setpoint, kpis):
    """
    Density histogram of encoder error over the confirmed settled-hold window.

    Returns None if settling was never confirmed (kpis["settling_time_s"] is None),
    since there is no valid hold window to characterise.

    Bin count: Freedman-Diaconis estimator, clipped to [10, 50]. The bare fd estimator
    can return too few bins on tightly-clustered distributions (making internal structure
    invisible) or too many on distributions with outliers (washing out the shape). The
    clip keeps the histogram interpretable across the sample sizes and error spreads
    typical of hold windows (~50–500 samples at 20 Hz).

    Gaussian overlay: a reference Gaussian with the same mean and std as the hold error
    is drawn alongside the histogram. Deviations from it — bimodal peaks, heavy tails,
    skew — are visually obvious against the smooth reference. Without it, these shapes
    are much harder to spot in a bare histogram.
    """
    if kpis is None or kpis["settling_time_s"] is None or kpis["settle_start_idx"] is None:
        return None

    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    hold_err = enc_roll[kpis["settle_start_idx"]:] - setpoint

    if len(hold_err) < 10:
        return None

    # Bin count with clip (see docstring)
    fd_edges = np.histogram_bin_edges(hold_err, bins="fd")
    n_bins   = int(np.clip(len(fd_edges) - 1, 10, 50))

    mu    = float(np.mean(hold_err))
    sigma = float(np.std(hold_err))
    p95   = float(np.percentile(np.abs(hold_err), 95))

    fig, ax = plt.subplots(1, 1, figsize=(8, 5))
    fig.suptitle(f"Hold-Error Distribution — {label}", fontsize=13)

    ax.hist(hold_err, bins=n_bins, density=True, alpha=0.65,
            color="tab:blue", label="Hold error (density)")

    # Gaussian overlay (see docstring)
    x_fit    = np.linspace(hold_err.min() - sigma, hold_err.max() + sigma, 300)
    gaussian = np.exp(-0.5 * ((x_fit - mu) / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi))
    ax.plot(x_fit, gaussian, color="tab:red", linewidth=1.5,
            label=f"Gaussian fit (μ={mu:+.2f}°, σ={sigma:.2f}°)")

    ax.axvline(0, color="tab:green", linewidth=1.2, linestyle="-",
               label=f"Setpoint ({setpoint:+.1f}°)")
    ax.axvline(mu, color="tab:orange", linewidth=1.2, linestyle="--",
               label=f"Bias ({mu:+.2f}°)")
    ax.axvspan(-HORIZONTAL_THRESHOLD_DEG, HORIZONTAL_THRESHOLD_DEG,
               color="gray", alpha=0.08, zorder=0,
               label=f"±{HORIZONTAL_THRESHOLD_DEG:.0f}° band")

    stats_text = f"n = {len(hold_err)}\nbias = {mu:+.2f}°\nstd = {sigma:.2f}°\nP95 = {p95:.2f}°"
    ax.text(0.97, 0.97, stats_text, transform=ax.transAxes, fontsize=8,
            va="top", ha="right",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    ax.set_xlabel("Hold error — enc_roll − setpoint (°)")
    ax.set_ylabel("Density")
    ax.set_title(f"Post-settle window: {len(hold_err)} samples")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Spectrum (PSD of hold-window error)
# ---------------------------------------------------------------------------

def welch_psd(x, fs, nperseg):
    """One-sided Welch PSD estimate using 50% overlapping Hanning windows."""
    noverlap = nperseg // 2
    step     = nperseg - noverlap
    win      = np.hanning(nperseg)
    win_pow  = np.sum(win ** 2)
    segments = [x[i:i + nperseg] for i in range(0, len(x) - nperseg + 1, step)]
    if not segments:
        segments = [x[:nperseg]]
    psds = []
    for seg in segments:
        sp = np.fft.rfft(seg * win, n=nperseg)
        ps = (np.abs(sp) ** 2) / (fs * win_pow)
        ps[1:-1] *= 2  # one-sided: double non-DC, non-Nyquist bins
        psds.append(ps)
    freqs = np.fft.rfftfreq(nperseg, d=1.0 / fs)
    return freqs, np.mean(psds, axis=0)


def plot_spectrum(cols, label, setpoint, kpis):
    """
    Power spectral density of the hold-window encoder error.

    Returns None if settling was never confirmed (no valid hold window).

    Welch's method (scipy.signal.welch) is used rather than a plain FFT because it
    averages overlapping windowed segments, reducing variance in the spectral estimate.
    For the short hold windows typical of these runs (50–500 samples at ~20 Hz) a plain
    FFT produces a noisy spectrum where individual bin heights are unreliable; Welch
    makes real peaks stand out against the noise floor more clearly.

    nperseg is capped at min(len(hold_err), 64) explicitly. The scipy default (256)
    exceeds the sample count for short hold windows, causing Welch to silently degrade
    to a plain FFT. The explicit cap keeps the averaging behaviour consistent and ensures
    the noise-floor estimate is always based on multiple segments where possible.

    The shaded region at low frequency marks one frequency bin (Δf = fs / nperseg) — any
    feature narrower than this cannot be resolved and should not be interpreted as a peak.
    """
    if kpis is None or kpis["settling_time_s"] is None or kpis["settle_start_idx"] is None:
        return None

    enc_roll  = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    hold_err  = enc_roll[kpis["settle_start_idx"]:] - setpoint
    hold_t_ms = cols["T_MS"][kpis["settle_start_idx"]:]

    if len(hold_err) < 10:
        return None

    dt_s    = float(np.mean(np.diff(hold_t_ms))) / 1000.0
    fs      = 1.0 / dt_s
    nperseg = min(len(hold_err), 64)

    freqs, psd = welch_psd(hold_err, fs=fs, nperseg=nperseg)

    # Exclude DC bin for peak / floor calculation
    nondc        = freqs > 0
    nondc_freqs  = freqs[nondc]
    nondc_psd    = psd[nondc]
    peak_idx     = int(np.argmax(nondc_psd))
    peak_freq    = float(nondc_freqs[peak_idx])
    median_floor = float(np.median(nondc_psd))
    freq_res     = fs / nperseg

    fig, ax = plt.subplots(1, 1, figsize=(8, 5))
    fig.suptitle(f"Hold-Error Spectrum — {label}", fontsize=13)

    ax.semilogy(nondc_freqs, nondc_psd, color="tab:blue", linewidth=1.0)

    ax.axhline(median_floor, color="gray", linewidth=0.8, linestyle="--", alpha=0.7,
               label=f"Median noise floor")
    ax.axvline(peak_freq, color="tab:red", linewidth=0.8, linestyle="--", alpha=0.8,
               label=f"Peak: {peak_freq:.3f} Hz")
    ax.axvspan(0, freq_res, color="gray", alpha=0.12, zorder=0,
               label=f"1 bin = {freq_res:.3f} Hz")

    n_samples = len(hold_err)
    hold_dur  = float(hold_t_ms[-1] - hold_t_ms[0]) / 1000.0
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Power spectral density (°²/Hz)")
    ax.set_title(f"{n_samples} samples, {hold_dur:.1f}s hold, fs={fs:.1f} Hz, nperseg={nperseg}")
    ax.set_xlim(0, fs / 2.0)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Step response
# ---------------------------------------------------------------------------

def plot_step_response(cols, label, setpoint, kpis):
    """
    Annotated step response: two stacked subplots.

    Top — full run duration with key milestones:
      T→SP (green dashed)  : first entry into the ±10° band
      T_s  (orange dashed) : confirmed settling (≥5s continuous hold)
      OS   (red marker)    : global extremum past setpoint after first crossing
                             (global extremum is appropriate here — it marks the
                             worst-case excursion visible on the full timeline)

    Bottom — transient zoom (first ~6–15s):
      10%/90% rise markers (purple dots + dotted guide lines, legend carries value)
      Overshoot peak marker if it falls within the zoom window
      T→SP repeated for alignment reference
    """
    enc_roll     = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    t_s          = (cols["T_MS"] - cols["T_MS"][0]) / 1000.0
    start_angle  = enc_roll[0]
    initial_step = setpoint - start_angle

    # --- Rise markers ---
    t_10 = t_90 = mark_10 = mark_90 = None
    if abs(initial_step) > 1e-6:
        mark_10 = start_angle + 0.10 * initial_step
        mark_90 = start_angle + 0.90 * initial_step
        for i, angle in enumerate(enc_roll):
            if t_10 is None and (
                    (initial_step < 0 and angle <= mark_10) or
                    (initial_step > 0 and angle >= mark_10)):
                t_10 = t_s[i]
            if t_90 is None and (
                    (initial_step < 0 and angle <= mark_90) or
                    (initial_step > 0 and angle >= mark_90)):
                t_90 = t_s[i]
            if t_10 is not None and t_90 is not None:
                break

    # --- Overshoot peak: global extremum past setpoint after first crossing ---
    peak_t = peak_angle = None
    if kpis is not None and kpis["overshoot_pct"] is not None and kpis["overshoot_pct"] > 0:
        crossed_idx = None
        for i, angle in enumerate(enc_roll):
            if (initial_step < 0 and angle <= setpoint) or \
               (initial_step > 0 and angle >= setpoint):
                crossed_idx = i
                break
        if crossed_idx is not None:
            post     = enc_roll[crossed_idx:]
            peak_idx = int(np.argmin(post) if initial_step < 0 else np.argmax(post))
            abs_idx  = crossed_idx + peak_idx
            peak_t     = t_s[abs_idx]
            peak_angle = enc_roll[abs_idx]

    # --- Zoom window for bottom subplot ---
    t_zoom = max(
        t_90 + 3.0 if t_90 is not None else 0.0,
        peak_t + 1.5 if peak_t is not None else 0.0,
        kpis["time_to_s"] + 2.0 if (kpis is not None and kpis["time_to_s"] is not None) else 0.0,
        6.0)
    t_zoom = min(t_zoom, 15.0, t_s[-1])
    zoom_mask = t_s <= t_zoom

    # --- Figure ---
    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, figsize=(11, 7),
        gridspec_kw={"height_ratios": [2, 1.3]})
    fig.suptitle(f"Step Response — {label}", fontsize=13)

    # ---- Top: full run with milestones ----
    fmt_top, xlabel_top = _time_formatter(t_s[-1])

    ax_top.axhspan(setpoint - HORIZONTAL_THRESHOLD_DEG,
                   setpoint + HORIZONTAL_THRESHOLD_DEG,
                   color="gray", alpha=0.08, zorder=0)
    ax_top.axhline(setpoint, color="gray", linewidth=0.8, linestyle="--", alpha=0.6)
    ax_top.plot(t_s, enc_roll, color="tab:blue", linewidth=0.9, label="Encoder")

    if kpis is not None and kpis["time_to_s"] is not None:
        ax_top.axvline(kpis["time_to_s"], color="tab:green", linewidth=0.9,
                       linestyle="--", alpha=0.8,
                       label=f"T→SP = {kpis['time_to_s']:.1f}s")

    if kpis is not None and kpis["settling_time_s"] is not None:
        ax_top.axvline(kpis["settling_time_s"], color="tab:orange", linewidth=0.9,
                       linestyle="--", alpha=0.8,
                       label=f"T_s = {kpis['settling_time_s']:.1f}s")

    if peak_t is not None:
        marker = "v" if initial_step < 0 else "^"
        ax_top.plot(peak_t, peak_angle, marker, color="tab:red",
                    markersize=8, zorder=3,
                    label=f"OS = {kpis['overshoot_pct']:.1f}%")

    if fmt_top:
        ax_top.xaxis.set_major_formatter(fmt_top)
    ax_top.set_xlabel(xlabel_top)
    ax_top.set_ylabel("Encoder angle (°)")
    ax_top.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax_top.grid(True, alpha=0.3)

    # ---- Bottom: transient zoom ----
    ax_bot.set_title(f"Transient zoom (first {t_zoom:.0f}s)", fontsize=9, pad=4)
    ax_bot.axhspan(setpoint - HORIZONTAL_THRESHOLD_DEG,
                   setpoint + HORIZONTAL_THRESHOLD_DEG,
                   color="gray", alpha=0.08, zorder=0)
    ax_bot.axhline(setpoint, color="gray", linewidth=0.8, linestyle="--", alpha=0.6)
    ax_bot.plot(t_s[zoom_mask], enc_roll[zoom_mask],
                color="tab:blue", linewidth=1.0)

    if kpis is not None and kpis["time_to_s"] is not None and \
            kpis["time_to_s"] <= t_zoom:
        ax_bot.axvline(kpis["time_to_s"], color="tab:green", linewidth=0.9,
                       linestyle="--", alpha=0.8)

    rise_legend_label = None
    for mark, t_mark in ((mark_10, t_10), (mark_90, t_90)):
        if mark is not None:
            ax_bot.axhline(mark, color="tab:purple", linewidth=0.6,
                           linestyle=":", alpha=0.5)
        if t_mark is not None and t_mark <= t_zoom:
            ax_bot.axvline(t_mark, color="tab:purple", linewidth=0.6,
                           linestyle=":", alpha=0.4)

    if t_10 is not None and t_90 is not None and \
            kpis is not None and kpis["rise_time_s"] is not None:
        rise_legend_label = f"Rise time 10→90% = {kpis['rise_time_s']:.1f}s"
        ax_bot.plot(t_10, mark_10, "o", color="tab:purple", markersize=6, zorder=3)
        ax_bot.plot(t_90, mark_90, "o", color="tab:purple", markersize=6, zorder=3,
                    label=rise_legend_label)

    if peak_t is not None and peak_t <= t_zoom:
        marker   = "v" if initial_step < 0 else "^"
        offset_y = -2 if initial_step < 0 else 2
        ax_bot.plot(peak_t, peak_angle, marker, color="tab:red",
                    markersize=8, zorder=3)
        ax_bot.annotate(f"OS = {kpis['overshoot_pct']:.1f}%",
                        xy=(peak_t, peak_angle),
                        xytext=(min(peak_t + 0.5, t_zoom * 0.88),
                                peak_angle + offset_y),
                        fontsize=8, color="tab:red",
                        arrowprops=dict(arrowstyle="->", color="tab:red", lw=0.7))

    ax_bot.set_xlim(0, t_zoom)
    ax_bot.set_xlabel("Time (s)")
    ax_bot.set_ylabel("Encoder angle (°)")
    ax_bot.grid(True, alpha=0.3)
    if rise_legend_label is not None:
        ax_bot.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)

    fig.tight_layout(rect=[0, 0, 0.87, 1])
    return fig


# ---------------------------------------------------------------------------
# Dispatcher
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        prog="plots.py",
        description="Generate diagnostic figures for a Flight Benchy run.")
    parser.add_argument("flight_folder",
                        help="Path to the run folder (test_runs/flights/<id>)")
    parser.add_argument("--type", dest="plot_type",
                        choices=["timeseries", "phase", "histogram", "spectrum", "step_response", "all"],
                        default="all",
                        help="Which figure(s) to generate (default: all)")
    args = parser.parse_args()

    p        = Path(args.flight_folder)
    run_dir  = p if p.is_dir() else p.parent
    setpoint = load_setpoint(run_dir)

    csv_path, cols, label = load_run(args.flight_folder)
    print(f"Loaded {label}: {len(cols['T_MS'])} samples")

    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    samples  = list(zip(cols["T_MS"].tolist(), enc_roll.tolist()))
    kpis     = compute_kpis(samples, setpoint)
    throttle_min, throttle_max = load_motor_limits(run_dir)

    do_all = args.plot_type == "all"

    if do_all or args.plot_type == "timeseries":
        fig = plot_run(cols, label, setpoint, kpis, throttle_min, throttle_max)
        out = csv_path.parent / "01_timeseries.png"
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved: {out}")

    if do_all or args.plot_type == "step_response":
        fig = plot_step_response(cols, label, setpoint, kpis)
        out = csv_path.parent / "02_step_response.png"
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved: {out}")

    if do_all or args.plot_type == "spectrum":
        fig = plot_spectrum(cols, label, setpoint, kpis)
        if fig is not None:
            out = csv_path.parent / "03_spectrum.png"
            fig.savefig(out, dpi=150, bbox_inches="tight")
            plt.close(fig)
            print(f"Saved: {out}")
        else:
            print("Skipped 03_spectrum.png — no confirmed settled hold.")

    if do_all or args.plot_type == "histogram":
        fig = plot_hold_error_distribution(cols, label, setpoint, kpis)
        if fig is not None:
            out = csv_path.parent / "04_hold_error_distribution.png"
            fig.savefig(out, dpi=150, bbox_inches="tight")
            plt.close(fig)
            print(f"Saved: {out}")
        else:
            print("Skipped 04_hold_error_distribution.png — no confirmed settled hold.")

    if do_all or args.plot_type == "phase":
        fig = plot_phase_portrait(cols, label, setpoint, kpis)
        out = csv_path.parent / "05_phase_portrait.png"
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved: {out}")


if __name__ == "__main__":
    main()
