# Tuning session — tolerance-band-9deg

## Objective

Tighten the acceptance band from ±10° to ±9°. `tolerance_deg` is a computational parameter — it defines the band used to compute T->SP, T_s, hold_duration_s, and HoldMAE. Narrowing it from 10.0 to 9.0 raises the bar for all KPIs simultaneously. Session goal: demonstrate that the current config meets all KPI pass thresholds when scored against the ±9° band.

- **Target parameter:** `specification.json` → `tolerance_deg`: 10.0 → 9.0
- **Why this matters:** ±9° confirmed hold establishes a tighter upper bound on steady-state error, one step toward the eventual target of ±7°.

## Starting state

**Frozen at session open. Do not modify mid-session.**

### Current configuration

- **angle_pid:** kp=3.0, ki=0.05, kd=0.5, iterm_limit=5.0
- **rate_pid:** kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
- **motor.base:** 500
- **feedforward.lead_ms:** 15
- **setpoint:** roll=0.0, pitch=0.0, yaw=0.0

### Prior session note

Session `2026-05-14-reduce-oscillation-hold-60s.md` (Met and confirmed). Objective: eliminate bimodal slow-settle mode (~1/3 of runs produced T_s 79–91s vs the usual 17–27s). Fix: `angle_pid.kd` 0.3→0.5, `iterm_limit` 100→5 — I-term accumulation during approach was driving overshoot ≥19% and a slow ring-down. Confirmed over 3 runs at ±10°: T_s 17–26s, overshoot 11–14%, HoldMAE_s 1.9–3.3°. Most relevant lesson: hold oscillation at current config has std 3.78–4.53° and max up to 15.03° within the ±10° window. That amplitude is the binding constraint for this session.

### Criteria snapshot (post-edit, tolerance_deg = 9.0)

| KPI | pass | good | excellent | Baseline level (at +/-10 deg) |
|-----|------|------|-----------|-------------------------------|
| hold_mae_deg | 5.0 deg | 3.5 deg | 2.5 deg | good (mean 2.73 deg) |
| time_to_sp_s | 10.0s | 5.0s | 3.0s | TBD (see note) |
| settling_time_s | 35.0s | 25.0s | 20.0s | TBD (see note) |
| hold_duration_s | 60.0s | 90.0s | 100.0s | TBD (see note) |
| overshoot_pct | 25% | 18% | 12% | good (~12–14%) |

*Note: T->SP, T_s, and hold_duration_s baseline levels at ±9° are not known from existing runs. Those runs were scored at ±10° and hold excursions regularly exceed ±9° (observed maxes: 9.84°, 10.02°, 15.03°). Envelope starting state for timing KPIs will be established in Iterations 1–3.*

**Envelope:** `tolerance_deg` is a computational parameter, not a scored KPI. The envelope check applies to all five KPIs in the table: any iteration that pushes a KPI below its `pass` threshold is a violation.

### Baseline KPIs (at tolerance_deg = 9.0)

*Established from Iterations 1–3. All runs at current config with tolerance_deg=9.0.*

| KPI | Mean | Std Dev | Range | Sample size |
|-----|------|---------|-------|-------------|
| Reached rate | 3/3 | — | — | 3 |
| T->SP (s) | 47.3 | 5.4 | 41.7–52.4 | 3 |
| HoldMAE_s (deg) | 5.21 | 2.07 | 3.29–7.35 | 3 |
| hold_duration_s (s) | 37.9 | 36.1 | 6.56–78.20 | 3 |
| settling_time_s (s) | 82.0 | 36.2 | 41.7–113.4 | 3 |
| overshoot_pct (%) | 3.0 | 2.8 | 0.0–5.6 | 3 |

Key observation: T->SP is 42–52s across all three runs — far from the 10s pass threshold. Rise time 43–69s across all three runs. The approach is heavily damped (zeta 0.677–>=1) with low overshoot (0–5.6%). T_s and hold_duration_s show very high variance (std ~36s) because they are governed by whether peak hold oscillations exceed +/-9 deg on a given run; when they do not (Iter 3), T_s = T->SP and hold_duration_s is long. The hold bias (+4.69 to +8.54 deg) is consistent and positive across all runs; the IMU-ENC bias is consistent at -1.60 to -1.92 deg.

### Prior baseline runs (at tolerance_deg = 10.0, reference only)

- 2026-05-14_16-16-07
- 2026-05-14_16-22-45
- 2026-05-14_16-28-32

## Iterations

---

## Iteration 1 — 2026-05-18

### Context

The three confirmation runs from the prior session (2026-05-14) were scored at ±10°. Hold oscillation in those runs had std 3.78–4.53° and max 9.84°, 10.02°, and 15.03°. At ±9°, any excursion past 9° breaks the confirmed-hold window — T_s cannot be declared and hold_duration_s resets to zero at that point. The existing runs cannot serve as this session's baseline: their KPIs were computed against a band that absorbed most of those excursions. Before any tuning change can be evaluated, the current config's performance at ±9° must be characterised. This iteration deploys specification.json with tolerance_deg=9.0 (already on disk) and no config change, and runs a single characterisation flight.

### Hypothesis

Baseline KPIs at tolerance_deg=9 are not yet established. The current config's hold oscillations exceed ±9° frequently enough that T_s and hold_duration_s at this band are unknown quantities.

### Proposed change

No config change. specification.json already edited: tolerance_deg 10.0 → 9.0. Deploy specification.json to Pico before run.

### Prediction

One of two outcomes is plausible. (a) hold oscillations stay below ±9° on this run → T_s, hold_duration_s, and HoldMAE_s are computable and broadly similar to the ±10° values. (b) at least one excursion exceeds ±9° → T_s is not achieved or is later in the run, hold_duration_s is shorter. The prior max of 15.03° suggests (b) is likely on at least some runs. Exact values are unknown — this is a characterisation run.

### Falsifier

N/A — characterisation run. No hypothesis under test; no outcome is disqualifying. This iteration captures one data point toward the ±9° baseline.

---

### Run

- **Run ID:** 2026-05-18_19-09-36
- **Status:** analysed

### Observed

*No ±9° baseline exists yet — Baseline mean and variance columns are N/A.*

| KPI | Observed | Baseline mean | Delta from baseline | Within baseline variance? |
|-----|----------|---------------|---------------------|---------------------------|
| Reached | YES | N/A | — | — |
| T->SP (s) | 52.4 | N/A | — | — |
| HoldMAE_s (deg) | 7.35 | N/A | — | — |
| hold_duration_s (s) | 6.56 | N/A | — | — |
| settling_time_s (s) | 113.4 | N/A | — | — |
| overshoot_pct (%) | 0.0 | N/A | — | — |

Additional profile: rise time 10-90% = 69.3s, zeta >= 1 (overdamped, no crossing of setpoint). Hold bias (encoder) = +8.54 deg, hold std = 2.21 deg. IMU-ENC bias = -1.92 deg (whole-run).

### Verdict

Inconclusive — characterisation run, no prior ±9° baseline to compare against. Data point obtained.

### Lessons

This run showed markedly different transient character from the prior confirmation runs at ±10° (T_s 17–26s, overshoot 11–14%, underdamped). Here: overdamped (zeta >= 1, 0% overshoot), rise time 69.3s, T_s 113.4s. The lever approached setpoint without crossing it, touched the ±9° boundary at 52.4s, exited (the band was broken between 52.4s and 113.4s), and re-entered for the final 6.56s. Hold bias of +8.54 deg with std 2.21 deg means the lever was near the band edge throughout the brief hold window rather than near setpoint. Run-to-run variance of this magnitude at identical config is the central open question — two more baseline flights are required before any characterisation of what the current config does at ±9°.

---

## Iteration 2 — 2026-05-18

### Context

Iteration 1 showed overdamped transient behavior (zeta >= 1, rise time 69.3s, T_s 113.4s) with 0% overshoot. This is qualitatively different from the prior ±10° confirmation runs, which showed underdamped approach (overshoot 11–14%, T_s 17–26s). The two behaviors are not obviously reconcilable from a single run: it is not yet clear whether Iteration 1 represents the typical behavior at ±9° scoring or an outlier. Hold bias of +8.54 deg in the short 6.56s hold window does not characterise steady-state hold quality — the lever was still approaching setpoint. A second characterisation flight at the same config provides the next data point. No change to config or specification.

### Hypothesis

Run-to-run variance at ±9° scoring is not yet characterised. A second run at identical config reveals whether the overdamped behavior in Iteration 1 was representative or an outlier.

### Proposed change

No config change. specification.json unchanged (tolerance_deg=9.0). Deploy before run.

### Prediction

Two plausible outcomes. (a) Second run also shows overdamped behavior, slow approach, short hold — confirming Iteration 1 as representative. (b) Second run shows underdamped approach (overshoot, faster T_s) similar to the prior ±10° runs, revealing Iteration 1 as an outlier. Either result informs the baseline characterisation.

### Falsifier

N/A — characterisation run. No outcome is disqualifying.

---

### Run

- **Run ID:** 2026-05-18_19-26-03 — DISCARDED (non-standard start: -50.4 deg, expected +58 +/-10 deg). Reset-position step did not place lever at restrictor.
- **Run ID:** 2026-05-18_20-29-15
- **Status:** analysed

### Observed

*No +/-9 deg baseline exists yet — Baseline mean and variance columns are N/A.*

| KPI | Observed | Baseline mean | Delta from baseline | Within baseline variance? |
|-----|----------|---------------|---------------------|---------------------------|
| Reached | YES | N/A | — | — |
| T->SP (s) | 47.9 | N/A | — | — |
| HoldMAE_s (deg) | 3.29 | N/A | — | — |
| hold_duration_s (s) | 28.91 | N/A | — | — |
| settling_time_s (s) | 91.0 | N/A | — | — |
| overshoot_pct (%) | 3.55 | N/A | — | — |

Additional profile: rise time 10-90% = 49.0s, zeta = 0.728. Hold bias = +4.69 deg, hold std = 3.13 deg, hold max = 10.81 deg. IMU-ENC bias = -1.72 deg.

### Verdict

Inconclusive — characterisation run. No prior +/-9 deg baseline.

### Lessons

Consistent with Iteration 1: slow approach (rise 49s vs 69s), late T_s (91s vs 113s), short hold (29s vs 7s). Both runs show hold oscillations reaching the +/-9 deg boundary (max 10.81 deg and 12.83 deg respectively), keeping T_s late. HoldMAE_s varies significantly between runs (3.29 deg vs 7.35 deg), driven by differing hold bias (+4.69 deg vs +8.54 deg) and hold window length. The slow approach in both runs (T->SP 48-52s) is structurally different from the prior +/-10 deg confirmation runs — this characterisation is not yet complete. One more baseline flight needed.

---

## Iteration 3 — 2026-05-18

### Context

Two baseline runs completed at tolerance_deg=9. Both show slow approach (rise time 49–69s, T->SP 48–52s) and late T_s (91–113s), leaving only 6–29s of confirmed hold. Hold oscillations reach the +/-9 deg boundary in both runs (max 10.81 deg and 12.83 deg), which is what keeps resetting the settled-hold clock. HoldMAE_s varies between runs (3.29 deg vs 7.35 deg), driven by differing hold bias and window length rather than oscillation amplitude — both runs have similar hold std (2.21–3.13 deg). A third run completes the baseline set.

### Hypothesis

Baseline characterisation at tolerance_deg=9 requires a third data point to establish mean and variance for all KPIs.

### Proposed change

No config change. specification.json unchanged (tolerance_deg=9.0). Deploy before run.

### Prediction

Third run will show the same pattern: slow approach (T->SP 40–60s), T_s 80–120s, hold oscillations that occasionally breach +/-9 deg, hold_duration_s 0–40s. HoldMAE_s likely 3–8 deg depending on how long the hold window runs and what the hold bias settles at.

### Falsifier

N/A — characterisation run.

---

### Run

- **Run ID:** 2026-05-18_20-52-51
- **Status:** analysed

### Observed

| KPI | Observed | Baseline mean | Delta from baseline | Within baseline variance? |
|-----|----------|---------------|---------------------|---------------------------|
| Reached | YES | 3/3 | — | — |
| T->SP (s) | 41.7 | 47.3 | -5.6 | yes (within 1 std) |
| HoldMAE_s (deg) | 4.98 | 5.21 | -0.23 | yes |
| hold_duration_s (s) | 78.20 | 37.9 | +40.3 | yes (high variance) |
| settling_time_s (s) | 41.7 | 82.0 | -40.3 | yes (high variance) |
| overshoot_pct (%) | 5.57 | 3.0 | +2.6 | yes |

Hold max 8.88 deg — just inside +/-9 deg band, which is why T_s = T->SP (no band exits after first entry). Hold bias +4.85 deg, std 2.24 deg. IMU-ENC bias -1.60 deg.

### Verdict

Inconclusive — characterisation run. Baseline now established at N=3.

### Lessons

Iter 3 confirms the pattern of slow approach (rise 42.9s, T->SP 41.7s) shared by all three baseline runs. The key structural finding: T_s outcome at +/-9 deg is governed by whether peak hold oscillations exceed 9 deg on a given run. When they do not (Iter 3, max 8.88 deg), T_s = T->SP and hold_duration is long. When they do (Iters 1–2, max 10.81–12.83 deg), the settled-hold clock resets and T_s is pushed to 91–113s. The hold std is consistent across all three runs (2.21–3.13 deg); the difference in T_s outcome is determined by individual-run variance in peak amplitude and by the hold bias level. The T->SP of 41–52s (pass threshold: 10s) is the session's binding constraint — the approach dynamics must change before this session can be Met.

---

## Iteration 4 — 2026-05-18

### Context

The three baseline runs establish that T->SP is 42–52s at kd=0.5 — 4–5x the 10s pass threshold. Rise time is 43–69s with zeta 0.68–>=1 and overshoot 0–5.6%. The prior +/-10 deg confirmation runs at the same kp/ki/iterm_limit produced T_s 17–26s and overshoot 11–14%, which required a fast underdamped approach. The sole config difference that affects approach damping is kd: it was raised from 0.3 to 0.5 on 2026-05-14 to eliminate I-term-driven slow settling. With iterm_limit now at 5 (down from 100), the original motivation for kd=0.5 is largely addressed — the I-term can no longer accumulate enough during approach to cause large overshoot. kd=0.4 is the midpoint: more damping than the kd=0.3 that produced the bimodal mode, less than the kd=0.5 that is over-damping the approach.

### Hypothesis

kd=0.5 is over-damping the approach relative to what the rig needs at current iterm_limit=5. Reducing kd to 0.4 will shorten rise time and T->SP by allowing more underdamped approach dynamics, while remaining more conservative than kd=0.3.

### Proposed change

`vehicle.angle_pid.kd`: 0.5 -> 0.4

### Prediction

T->SP drops below baseline mean (47.3s) by more than baseline std (5.4s) — i.e. T->SP < 42s, likely in the 10–25s range. Overshoot increases from baseline 0–5.6% toward 8–14%. T_s and hold_duration_s variance remains high, governed by hold oscillation amplitude. HoldMAE_s broadly similar to baseline mean (5.21 deg).

### Falsifier

If T->SP stays >= 42s (within baseline variance), kd is not the driver of the slow approach at current rig state — the hypothesis is wrong and a different variable must be investigated.

---

### Run

- **Run ID:** TBD
- **Status:** pending

### Observed

*To be filled after run.*

| KPI | Observed | Baseline mean | Delta from baseline | Within baseline variance? |
|-----|----------|---------------|---------------------|---------------------------|
| Reached | — | 3/3 | — | — |
| T->SP (s) | — | 47.3 | — | — |
| HoldMAE_s (deg) | — | 5.21 | — | — |
| hold_duration_s (s) | — | 37.9 | — | — |
| settling_time_s (s) | — | 82.0 | — | — |
| overshoot_pct (%) | — | 3.0 | — | — |

### Verdict

*To be filled after run.*

### Lessons

*To be filled after run.*

---

## Outcome

*To be filled at session close.*

*To be filled at session close.*
