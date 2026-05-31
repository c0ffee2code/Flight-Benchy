"""
Step 2 of the analyse-flight pipeline.

Dispatcher for all diagnostic figures. One flight folder in, one or more plot files out.

Currently supported plot types (--type argument, default: all):
  timeseries     -- 5-subplot time-series view              -> 01_timeseries.png
  step_response  -- Annotated step response (control-theory) -> 02_step_response.png
  spectrum       -- PSD of hold-window error                 -> 03_spectrum.png
  histogram      -- Hold-error distribution (post-settle)    -> 04_hold_error_distribution.png
  phase          -- Phase portrait (angle vs angular rate)   -> 05_phase_portrait.png

Run from project root:
  python .claude/skills/analyse-flight/scripts/plots.py test_runs/flights/<flight_id>
  python .claude/skills/analyse-flight/scripts/plots.py test_runs/flights/<flight_id> --type phase

Architecture: three-layer pipeline.
  Loading  -- load_flight() reads log.csv and produces a FlightData with pre-computed signals.
              detect_reach_event() and detect_hold_window() derive ReachEvent and HoldWindow.
  Compute  -- compute_*() functions are pure (numpy in, dataclass out) and own all data
             transformation. They are independent of matplotlib and can be called
             without generating any figures.
  Render   -- render_*() functions accept FlightData and computed structs and produce
             matplotlib figures. They contain no statistical logic.
"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.collections import LineCollection
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

sys.path.insert(0, str(Path(__file__).parent))
from specification_loader import load_specification                                           # noqa: E402
from configuration_loader import load_configuration                                          # noqa: E402
from flight_data_loader import load_flight, detect_reach_event, detect_hold_window, FlightData, ReachEvent, HoldWindow  # noqa: E402


# ---------------------------------------------------------------------------
# Compute-layer data containers
# ---------------------------------------------------------------------------

@dataclass
class HoldData:
    """
    Statistics over the confirmed settled-hold window, computed by compute_hold_window().

    Fields
    ------
    hold_err  : enc_roll[settle_start:] - setpoint, degrees. Shape (n,).
    hold_t_ms : T_MS[settle_start:], milliseconds. Shared with compute_spectrum. Shape (n,).
    n_samples : Number of samples in the hold window.
    n_bins    : Histogram bin count (FD estimator clipped to [10, 50]).
    mu        : Mean hold error, degrees.
    sigma     : Std of hold error, degrees.
    p95       : 95th percentile of |hold error|, degrees.
    x_fit     : x values for the Gaussian overlay curve. Shape (300,).
    gaussian  : Gaussian PDF evaluated at x_fit using mu and sigma. Shape (300,).
    """
    hold_err:  np.ndarray
    hold_t_ms: np.ndarray
    n_samples: int
    n_bins:    int
    mu:        float
    sigma:     float
    p95:       float
    x_fit:     np.ndarray
    gaussian:  np.ndarray


@dataclass
class SpectrumData:
    """
    One-sided Welch PSD of the hold-window error, computed by compute_spectrum().

    Fields
    ------
    freqs        : Non-DC frequency bins, Hz. Shape (n,).
    psd          : PSD at each non-DC frequency, deg^2/Hz. Shape (n,).
    peak_freq    : Frequency of the highest-power non-DC bin, Hz.
    median_floor : Median of the non-DC PSD -- proxy for the noise floor, deg^2/Hz.
    freq_res     : Width of one frequency bin = fs / nperseg, Hz.
    fs           : Sample rate of the hold window, Hz.
    n_samples    : Number of samples in the hold window.
    hold_dur     : Duration of the hold window, seconds.
    nperseg      : Window length used in the Welch estimate.
    """
    freqs:        np.ndarray
    psd:          np.ndarray
    peak_freq:    float
    median_floor: float
    freq_res:     float
    fs:           float
    n_samples:    int
    hold_dur:     float
    nperseg:      int


@dataclass
class StepData:
    """
    Pre-computed time/angle markers for the step response plot, from compute_step_response().

    Fields
    ------
    start_angle   : Encoder angle at t=0, degrees.
    initial_step  : setpoint - start_angle (negative for the standard start at +58 deg).
    mark_10       : Encoder angle at 10% of the step, degrees; None if step ~= 0.
    mark_90       : Encoder angle at 90% of the step, degrees; None if step ~= 0.
    t_10          : Time (s) when the 10% mark was first crossed; None if not reached.
    t_90          : Time (s) when the 90% mark was first crossed; None if not reached.
    rise_time_s   : t_90 - t_10, seconds; None if either mark not reached.
    overshoot_pct : Max excursion past setpoint as % of initial step; None if not reached.
    peak_t        : Time (s) of the global overshoot extremum; None if no overshoot.
    peak_angle    : Encoder angle (deg) at the overshoot extremum; None if no overshoot.
    t_zoom        : Right edge of the transient zoom subplot, seconds.
    """
    start_angle:   float
    initial_step:  float
    mark_10:       float | None
    mark_90:       float | None
    t_10:          float | None
    t_90:          float | None
    rise_time_s:   float | None
    overshoot_pct: float | None
    peak_t:        float | None
    peak_angle:    float | None
    t_zoom:        float


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _time_formatter(duration_s):
    if duration_s >= 60:
        def fmt(x, _):
            m, s = divmod(int(x), 60)
            return f"{m}:{s:02d}"
        return mticker.FuncFormatter(fmt), "Time (m:ss)"
    return None, "Time (s)"


# ---------------------------------------------------------------------------
# Compute layer -- pure (numpy in, dataclass out), no matplotlib
# ---------------------------------------------------------------------------

def _welch_psd(x, fs, nperseg):
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


def compute_hold_window(fd: FlightData, hold_window: HoldWindow | None, setpoint: float):
    """
    Statistics over the confirmed hold window (HoldWindow.start_idx onward).
    Returns None if hold was never confirmed or the window is too short.

    Bin count rationale: the bare FD estimator can return too few bins on tightly-
    clustered distributions (making internal structure invisible) or too many on
    distributions with outliers (washing out the shape). The clip keeps the histogram
    interpretable across the sample sizes and error spreads typical of hold windows
    (~50-500 samples at 20 Hz).
    """
    if hold_window is None:
        return None

    si        = hold_window.start_idx
    hold_err  = fd.enc_roll[si:] - setpoint
    hold_t_ms = fd.t_ms[si:]

    if len(hold_err) < 10:
        return None

    fd_edges = np.histogram_bin_edges(hold_err, bins="fd")
    n_bins   = int(np.clip(len(fd_edges) - 1, 10, 50))
    mu       = float(np.mean(hold_err))
    sigma    = float(np.std(hold_err))
    p95      = float(np.percentile(np.abs(hold_err), 95))
    x_fit    = np.linspace(hold_err.min() - sigma, hold_err.max() + sigma, 300)
    gaussian = np.exp(-0.5 * ((x_fit - mu) / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi))

    return HoldData(
        hold_err=hold_err,
        hold_t_ms=hold_t_ms,
        n_samples=len(hold_err),
        n_bins=n_bins,
        mu=mu,
        sigma=sigma,
        p95=p95,
        x_fit=x_fit,
        gaussian=gaussian,
    )


def compute_spectrum(hold_data: HoldData | None):
    """
    One-sided Welch PSD of the hold-window error.
    Returns None if hold_data is None or the window is too short.

    Takes hold_data from compute_hold_window -- reuses hold_err and hold_t_ms to
    avoid recomputing the same slice.

    nperseg is capped at min(len(hold_err), 64). The default of 256 would exceed the
    sample count for short hold windows, causing Welch to silently degrade to a plain
    FFT. The explicit cap keeps averaging behaviour consistent.

    The shaded 1-bin region in the rendered plot marks delta_f = freq_res -- any feature
    narrower than this cannot be resolved and should not be interpreted as a peak.
    """
    if hold_data is None:
        return None

    hold_err  = hold_data.hold_err
    hold_t_ms = hold_data.hold_t_ms

    if len(hold_err) < 10:
        return None

    dt_s    = float(np.mean(np.diff(hold_t_ms))) / 1000.0
    fs      = 1.0 / dt_s
    nperseg = min(len(hold_err), 64)

    freqs, psd   = _welch_psd(hold_err, fs=fs, nperseg=nperseg)
    nondc        = freqs > 0
    nondc_freqs  = freqs[nondc]
    nondc_psd    = psd[nondc]
    peak_idx     = int(np.argmax(nondc_psd))

    return SpectrumData(
        freqs=nondc_freqs,
        psd=nondc_psd,
        peak_freq=float(nondc_freqs[peak_idx]),
        median_floor=float(np.median(nondc_psd)),
        freq_res=fs / nperseg,
        fs=fs,
        n_samples=len(hold_err),
        hold_dur=float(hold_t_ms[-1] - hold_t_ms[0]) / 1000.0,
        nperseg=nperseg,
    )


def compute_step_response(
    fd: FlightData,
    reach_event: ReachEvent | None,
    hold_window: HoldWindow | None,
    setpoint: float,
) -> StepData:
    """
    Pre-compute all time/angle markers, overshoot, rise time, and the zoom window
    for the step response plot.

    Rise markers are derived directly from the encoder signal so the render layer
    has the raw timestamps (t_10, t_90) to draw guide lines and dot markers.

    Overshoot peak: global extremum past setpoint after first crossing. The global
    extremum is appropriate here -- it marks the worst-case excursion visible on
    the full timeline.
    """
    enc_roll     = fd.enc_roll
    t_s          = (fd.t_ms - fd.t_ms[0]) / 1000.0
    start_angle  = float(enc_roll[0])
    initial_step = setpoint - start_angle

    # --- Rise markers (10% / 90% of initial step) ---
    t_10 = t_90 = mark_10 = mark_90 = None
    rise_time_s = None
    if abs(initial_step) > 1e-6:
        mark_10 = start_angle + 0.10 * initial_step
        mark_90 = start_angle + 0.90 * initial_step
        for i, angle in enumerate(enc_roll):
            if t_10 is None and (
                    (initial_step < 0 and angle <= mark_10) or
                    (initial_step > 0 and angle >= mark_10)):
                t_10 = float(t_s[i])
            if t_90 is None and (
                    (initial_step < 0 and angle <= mark_90) or
                    (initial_step > 0 and angle >= mark_90)):
                t_90 = float(t_s[i])
            if t_10 is not None and t_90 is not None:
                break
        if t_10 is not None and t_90 is not None:
            rise_time_s = t_90 - t_10

    # --- Overshoot: global extremum past setpoint after first crossing ---
    overshoot_pct = None
    peak_t = peak_angle = None
    if reach_event is not None and abs(initial_step) > 1e-6:
        crossed_idx = None
        for i, angle in enumerate(enc_roll):
            if (initial_step < 0 and angle <= setpoint) or \
               (initial_step > 0 and angle >= setpoint):
                crossed_idx = i
                break
        if crossed_idx is not None:
            post     = enc_roll[crossed_idx:]
            peak_idx = int(np.argmin(post) if initial_step < 0 else np.argmax(post))
            max_exc  = float(
                setpoint - post[peak_idx] if initial_step < 0 else post[peak_idx] - setpoint
            )
            overshoot_pct = max(0.0, max_exc / abs(initial_step) * 100.0)
            if overshoot_pct > 0:
                abs_idx    = crossed_idx + peak_idx
                peak_t     = float(t_s[abs_idx])
                peak_angle = float(enc_roll[abs_idx])
        else:
            overshoot_pct = 0.0

    # --- Zoom window for the transient subplot ---
    t_zoom = max(
        t_90 + 3.0                    if t_90         is not None else 0.0,
        peak_t + 1.5                  if peak_t        is not None else 0.0,
        reach_event.start_time_s + 2.0 if reach_event is not None else 0.0,
        6.0)
    t_zoom = min(t_zoom, 15.0, float(t_s[-1]))

    return StepData(
        start_angle=start_angle,
        initial_step=initial_step,
        mark_10=mark_10,
        mark_90=mark_90,
        t_10=t_10,
        t_90=t_90,
        rise_time_s=rise_time_s,
        overshoot_pct=overshoot_pct,
        peak_t=peak_t,
        peak_angle=peak_angle,
        t_zoom=t_zoom,
    )


# ---------------------------------------------------------------------------
# Render layer -- matplotlib only, no data computation
# ---------------------------------------------------------------------------

def render_timeseries(
    fd: FlightData,
    label: str,
    setpoint: float,
    reach_event: ReachEvent | None,
    hold_window: HoldWindow | None,
    throttle_min: float,
    throttle_max: float,
    tolerance_deg: float,
):
    t_s        = (fd.t_ms - fd.t_ms[0]) / 1000.0
    duration_s = float(t_s[-1])

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
    fig.suptitle(f"Flight Benchy -- {label}  ({duration_s:.1f}s)", fontsize=13)

    ax1.axhspan(setpoint - tolerance_deg,
                setpoint + tolerance_deg,
                color="gray", alpha=0.08, zorder=0)
    ax1.axhline(setpoint, color="gray", linewidth=0.8, linestyle="--",
                label=f"Setpoint ({setpoint:+.0f} deg)")
    ax1.plot(t_s, fd.enc_roll, label="Encoder", linewidth=0.8)
    ax1.plot(t_s, fd.imu_roll, label="IMU", linewidth=0.8, alpha=0.6)
    ax1.set_ylabel("Roll (deg)")
    ax1.set_title("Angle Tracking")
    ax1.grid(True, alpha=0.3)

    ax2.plot(t_s, fd.gyro_x, linewidth=0.6, color="tab:blue",
             alpha=0.7, label="Gyro X (actual)")
    ax2.plot(t_s, fd.rate_sp, linewidth=0.8, color="tab:orange",
             label="Rate setpoint")
    ax2.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax2.set_ylabel("Rate (deg/s)")
    ax2.set_title("Rate Tracking (setpoint vs actual)")
    ax2.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax2.grid(True, alpha=0.3)

    ax3.plot(t_s, fd.ang_p, label="P", linewidth=0.8, color="tab:blue")
    ax3.plot(t_s, fd.ang_i, label="I", linewidth=0.8, color="tab:orange")
    ax3.plot(t_s, fd.ang_d, label="D", linewidth=0.8, color="tab:green")
    ax3.set_ylabel("Output (deg/s)")
    ax3.set_title("Angle PID (outer loop)")
    ax3.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax3.grid(True, alpha=0.3)

    ax4.plot(t_s, fd.rate_p, label="P", linewidth=0.8, color="tab:blue")
    ax4.plot(t_s, fd.rate_i, label="I", linewidth=0.8, color="tab:orange")
    ax4.plot(t_s, fd.rate_d, label="D", linewidth=0.8, color="tab:green")
    ax4.set_ylabel("Output (throttle)")
    ax4.set_title("Rate PID (inner loop)")
    ax4.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax4.grid(True, alpha=0.3)

    ax5.plot(t_s, fd.m1, label="M1", linewidth=0.8)
    ax5.plot(t_s, fd.m2, label="M2", linewidth=0.8)
    ax5.axhline(throttle_max, color="red", linestyle=":", linewidth=0.6, alpha=0.5, zorder=0)
    ax5.axhline(throttle_min, color="red", linestyle=":", linewidth=0.6, alpha=0.5, zorder=0)
    ax5.set_ylabel("Throttle")
    ax5.set_title("Motor Output")
    ax5.grid(True, alpha=0.3)

    if reach_event is not None:
        for ax in (ax1, ax2, ax3, ax4, ax5):
            ax.axvline(reach_event.start_time_s, color="tab:green", linewidth=0.6,
                       linestyle="--", alpha=0.5, zorder=0)
    if hold_window is not None:
        for ax in (ax1, ax2, ax3, ax4, ax5):
            ax.axvspan(hold_window.start_time_s, t_s[-1],
                       color="tab:green", alpha=0.06, zorder=0)

    h1, _ = ax1.get_legend_handles_labels()
    h1 += [Patch(facecolor='gray', alpha=0.5, edgecolor='none',
                 label=f'Settling window (+-{tolerance_deg:.0f} deg)')]
    if reach_event is not None:
        h1 += [Line2D([0], [0], color='tab:green', linewidth=0.8,
                      linestyle='--', alpha=0.7, label='T->SP')]
    if hold_window is not None:
        h1 += [Patch(facecolor='tab:green', alpha=0.4, edgecolor='none',
                     label='Hold window')]
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


def render_phase_portrait(
    fd: FlightData,
    label: str,
    setpoint: float,
    reach_event: ReachEvent | None,
    tolerance_deg: float,
):
    """
    Phase portrait: encoder angle (x-axis) vs angular rate (y-axis), coloured by time.

    Rate source: gyro_x is used directly rather than numerically differentiating the
    encoder angle. gyro_x is already logged, is exactly what the inner control loop
    sees, and avoids the noise amplification that finite differences introduce at the
    50 ms sample interval.

    Settled-zone shading: only the angle axis is shaded (+-tolerance_deg vertical band).
    A rate-axis threshold is not defined as a KPI and would be an arbitrary choice,
    giving a false impression of precision about what "settled" means in rate space.
    """
    enc_roll = fd.enc_roll
    gyro_x   = fd.gyro_x
    t_s      = (fd.t_ms - fd.t_ms[0]) / 1000.0

    points   = np.array([enc_roll, gyro_x]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm     = plt.Normalize(t_s[0], t_s[-1])
    lc       = LineCollection(segments, cmap="plasma", norm=norm, linewidth=0.8, alpha=0.8)
    lc.set_array(t_s[:-1])

    fig, ax = plt.subplots(1, 1, figsize=(7, 7))
    fig.suptitle(f"Phase Portrait -- {label}", fontsize=13)

    ax.add_collection(lc)
    ax.autoscale_view()

    ax.axvspan(setpoint - tolerance_deg,
               setpoint + tolerance_deg,
               color="gray", alpha=0.08, zorder=0)
    ax.axvline(setpoint, color="gray", linewidth=0.8, linestyle="--", alpha=0.6, zorder=1)
    ax.axhline(0,        color="gray", linewidth=0.8, linestyle="--", alpha=0.6, zorder=1)
    ax.plot(setpoint, 0, "k+", markersize=10, markeredgewidth=1.5, zorder=2,
            label=f"Target state ({setpoint:+.0f} deg, 0 deg/s)")

    if reach_event is not None:
        idx = reach_event.start_idx
        ax.plot(enc_roll[idx], gyro_x[idx], "o", color="tab:green",
                markersize=6, zorder=3, label="T->SP")

    cb = fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label("Time (s)", fontsize=8)

    ax.set_xlabel("Encoder angle (deg)")
    ax.set_ylabel("Angular rate (deg/s)")
    ax.set_title("Angle vs Angular Rate")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def render_hold_error_distribution(hold_data: HoldData, label: str, setpoint: float, tolerance_deg: float):
    """
    Density histogram of encoder error over the confirmed settled-hold window.

    Gaussian overlay: a reference Gaussian with the same mean and std as the hold error
    is drawn alongside the histogram. Deviations from it -- bimodal peaks, heavy tails,
    skew -- are visually obvious against the smooth reference. Without it, these shapes
    are much harder to spot in a bare histogram.
    """
    hold_err  = hold_data.hold_err
    n_bins    = hold_data.n_bins
    mu        = hold_data.mu
    sigma     = hold_data.sigma
    p95       = hold_data.p95
    x_fit     = hold_data.x_fit
    gaussian  = hold_data.gaussian
    n_samples = hold_data.n_samples

    fig, ax = plt.subplots(1, 1, figsize=(8, 5))
    fig.suptitle(f"Hold-Error Distribution -- {label}", fontsize=13)

    ax.hist(hold_err, bins=n_bins, density=True, alpha=0.65,
            color="tab:blue", label="Hold error (density)")

    ax.plot(x_fit, gaussian, color="tab:red", linewidth=1.5,
            label=f"Gaussian fit (mu={mu:+.2f} deg, sigma={sigma:.2f} deg)")

    ax.axvline(0, color="tab:green", linewidth=1.2, linestyle="-",
               label=f"Setpoint ({setpoint:+.1f} deg)")
    ax.axvline(mu, color="tab:orange", linewidth=1.2, linestyle="--",
               label=f"Bias ({mu:+.2f} deg)")
    ax.axvspan(-tolerance_deg, tolerance_deg,
               color="gray", alpha=0.08, zorder=0,
               label=f"Settling window (+-{tolerance_deg:.0f} deg)")

    stats_text = f"n = {n_samples}\nbias = {mu:+.2f} deg\nstd = {sigma:.2f} deg\nP95 = {p95:.2f} deg"
    ax.text(0.97, 0.97, stats_text, transform=ax.transAxes, fontsize=8,
            va="top", ha="right",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    ax.set_xlabel("Hold error -- enc_roll - setpoint (deg)")
    ax.set_ylabel("Density")
    ax.set_title(f"Hold window: {n_samples} samples")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def render_spectrum(spec_data: SpectrumData, label: str):
    freqs        = spec_data.freqs
    psd          = spec_data.psd
    peak_freq    = spec_data.peak_freq
    median_floor = spec_data.median_floor
    freq_res     = spec_data.freq_res
    fs           = spec_data.fs
    n_samples    = spec_data.n_samples
    hold_dur     = spec_data.hold_dur
    nperseg      = spec_data.nperseg

    fig, ax = plt.subplots(1, 1, figsize=(8, 5))
    fig.suptitle(f"Hold-Error Spectrum -- {label}", fontsize=13)

    ax.semilogy(freqs, psd, color="tab:blue", linewidth=1.0)

    ax.axhline(median_floor, color="gray", linewidth=0.8, linestyle="--", alpha=0.7,
               label="Median noise floor")
    ax.axvline(peak_freq, color="tab:red", linewidth=0.8, linestyle="--", alpha=0.8,
               label=f"Peak: {peak_freq:.3f} Hz")
    ax.axvspan(0, freq_res, color="gray", alpha=0.12, zorder=0,
               label=f"1 bin = {freq_res:.3f} Hz")

    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Power spectral density (deg^2/Hz)")
    ax.set_title(f"{n_samples} samples, {hold_dur:.1f}s hold, fs={fs:.1f} Hz, nperseg={nperseg}")
    ax.set_xlim(0, fs / 2.0)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    fig.tight_layout()
    return fig


def render_step_response(
    fd: FlightData,
    label: str,
    step_data: StepData,
    setpoint: float,
    reach_event: ReachEvent | None,
    hold_window: HoldWindow | None,
    tolerance_deg: float,
):
    """
    Annotated step response: two stacked subplots.

    Top -- full run duration with key milestones:
      T->SP (green dashed)  : first entry into the +-10 deg band
      T_s  (orange dashed) : confirmed settling (>=5s continuous hold)
      OS   (red marker)    : global extremum past setpoint after first crossing

    Bottom -- transient zoom (first ~6-15s):
      10%/90% rise markers (purple dots + dotted guide lines, legend carries value)
      Overshoot peak marker if it falls within the zoom window
      T->SP repeated for alignment reference
    """
    enc_roll     = fd.enc_roll
    t_s          = (fd.t_ms - fd.t_ms[0]) / 1000.0
    initial_step = step_data.initial_step
    mark_10      = step_data.mark_10
    mark_90      = step_data.mark_90
    t_10         = step_data.t_10
    t_90         = step_data.t_90
    rise_time_s  = step_data.rise_time_s
    overshoot_pct = step_data.overshoot_pct
    peak_t       = step_data.peak_t
    peak_angle   = step_data.peak_angle
    t_zoom       = step_data.t_zoom

    zoom_mask = t_s <= t_zoom

    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, figsize=(11, 7),
        gridspec_kw={"height_ratios": [2, 1.3]})
    fig.suptitle(f"Step Response -- {label}", fontsize=13)

    # ---- Top: full run with milestones ----
    fmt_top, xlabel_top = _time_formatter(float(t_s[-1]))

    ax_top.axhspan(setpoint - tolerance_deg,
                   setpoint + tolerance_deg,
                   color="gray", alpha=0.08, zorder=0)
    ax_top.axhline(setpoint, color="gray", linewidth=0.8, linestyle="--", alpha=0.6)
    ax_top.plot(t_s, enc_roll, color="tab:blue", linewidth=0.9, label="Encoder")

    if reach_event is not None:
        ax_top.axvline(reach_event.start_time_s, color="tab:green", linewidth=0.9,
                       linestyle="--", alpha=0.8,
                       label=f"T->SP = {reach_event.start_time_s:.1f}s")

    if hold_window is not None:
        ax_top.axvline(hold_window.start_time_s, color="tab:orange", linewidth=0.9,
                       linestyle="--", alpha=0.8,
                       label=f"T_s = {hold_window.start_time_s:.1f}s")

    if peak_t is not None and overshoot_pct is not None:
        marker = "v" if initial_step < 0 else "^"
        ax_top.plot(peak_t, peak_angle, marker, color="tab:red",
                    markersize=8, zorder=3,
                    label=f"OS = {overshoot_pct:.1f}%")

    if fmt_top:
        ax_top.xaxis.set_major_formatter(fmt_top)
    ax_top.set_xlabel(xlabel_top)
    ax_top.set_ylabel("Encoder angle (deg)")
    ax_top.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax_top.grid(True, alpha=0.3)

    # ---- Bottom: transient zoom ----
    ax_bot.set_title(f"Transient zoom (first {t_zoom:.0f}s)", fontsize=9, pad=4)
    ax_bot.axhspan(setpoint - tolerance_deg,
                   setpoint + tolerance_deg,
                   color="gray", alpha=0.08, zorder=0)
    ax_bot.axhline(setpoint, color="gray", linewidth=0.8, linestyle="--", alpha=0.6)
    ax_bot.plot(t_s[zoom_mask], enc_roll[zoom_mask],
                color="tab:blue", linewidth=1.0)

    if reach_event is not None and reach_event.start_time_s <= t_zoom:
        ax_bot.axvline(reach_event.start_time_s, color="tab:green", linewidth=0.9,
                       linestyle="--", alpha=0.8)

    rise_legend_label = None
    for mark, t_mark in ((mark_10, t_10), (mark_90, t_90)):
        if mark is not None:
            ax_bot.axhline(mark, color="tab:purple", linewidth=0.6,
                           linestyle=":", alpha=0.5)
        if t_mark is not None and t_mark <= t_zoom:
            ax_bot.axvline(t_mark, color="tab:purple", linewidth=0.6,
                           linestyle=":", alpha=0.4)

    if t_10 is not None and t_90 is not None and rise_time_s is not None:
        rise_legend_label = f"Rise time 10->90% = {rise_time_s:.1f}s"
        ax_bot.plot(t_10, mark_10, "o", color="tab:purple", markersize=6, zorder=3)
        ax_bot.plot(t_90, mark_90, "o", color="tab:purple", markersize=6, zorder=3,
                    label=rise_legend_label)

    if peak_t is not None and peak_t <= t_zoom and overshoot_pct is not None:
        marker   = "v" if initial_step < 0 else "^"
        offset_y = -2 if initial_step < 0 else 2
        ax_bot.plot(peak_t, peak_angle, marker, color="tab:red",
                    markersize=8, zorder=3)
        ax_bot.annotate(f"OS = {overshoot_pct:.1f}%",
                        xy=(peak_t, peak_angle),
                        xytext=(min(peak_t + 0.5, t_zoom * 0.88),
                                peak_angle + offset_y),
                        fontsize=8, color="tab:red",
                        arrowprops=dict(arrowstyle="->", color="tab:red", lw=0.7))

    ax_bot.set_xlim(0, t_zoom)
    ax_bot.set_xlabel("Time (s)")
    ax_bot.set_ylabel("Encoder angle (deg)")
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
                        choices=["timeseries", "phase", "histogram", "spectrum",
                                 "step_response", "all"],
                        default="all",
                        help="Which figure(s) to generate (default: all)")
    args = parser.parse_args()

    run_dir = Path(args.flight_folder)
    label   = run_dir.name

    cfg          = load_configuration(run_dir)
    spec         = load_specification(run_dir)
    sp           = cfg.setpoint_roll_deg
    throttle_min = cfg.motor.throttle_min
    throttle_max = cfg.motor.throttle_max

    tol     = spec.tolerance_deg
    fd      = load_flight(run_dir / "log.csv")
    reach_w = detect_reach_event(fd, sp, tol)
    hold_w  = detect_hold_window(fd, reach_w, sp, tol)
    print(f"Loaded {label}: {len(fd.t_ms)} samples")

    hold_data = compute_hold_window(fd, hold_w, sp)
    spec_data = compute_spectrum(hold_data)
    step_data = compute_step_response(fd, reach_w, hold_w, sp)

    analysis_dir = run_dir / "analysis"
    analysis_dir.mkdir(exist_ok=True)
    do_all = args.plot_type == "all"

    if do_all or args.plot_type == "timeseries":
        fig = render_timeseries(fd, label, sp, reach_w, hold_w, throttle_min, throttle_max, tol)
        out = analysis_dir / "01_timeseries.png"
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved: {out}")

    if do_all or args.plot_type == "step_response":
        fig = render_step_response(fd, label, step_data, sp, reach_w, hold_w, tol)
        out = analysis_dir / "02_step_response.png"
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved: {out}")

    if do_all or args.plot_type == "spectrum":
        if spec_data is not None:
            fig = render_spectrum(spec_data, label)
            out = analysis_dir / "03_spectrum.png"
            fig.savefig(out, dpi=150, bbox_inches="tight")
            plt.close(fig)
            print(f"Saved: {out}")
        else:
            print("Skipped 03_spectrum.png -- no confirmed settled hold.")

    if do_all or args.plot_type == "histogram":
        if hold_data is not None:
            fig = render_hold_error_distribution(hold_data, label, sp, tol)
            out = analysis_dir / "04_hold_error_distribution.png"
            fig.savefig(out, dpi=150, bbox_inches="tight")
            plt.close(fig)
            print(f"Saved: {out}")
        else:
            print("Skipped 04_hold_error_distribution.png -- no confirmed settled hold.")

    if do_all or args.plot_type == "phase":
        fig = render_phase_portrait(fd, label, sp, reach_w, tol)
        out = analysis_dir / "05_phase_portrait.png"
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved: {out}")


if __name__ == "__main__":
    main()
