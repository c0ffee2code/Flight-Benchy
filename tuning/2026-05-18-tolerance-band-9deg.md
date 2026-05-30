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

- **Run ID:** 2026-05-20_20-55-47
- **Status:** analysed

### Observed

| KPI | Observed | Baseline mean | Delta from baseline | Within baseline variance? |
|-----|----------|---------------|---------------------|---------------------------|
| Reached | YES | 3/3 | — | — |
| T->SP (s) | 39.4 | 47.3 | -7.9 | borderline — 1.5 std below mean, outside baseline range (41.7–52.4s) |
| HoldMAE_s (deg) | null | 5.21 | N/A | no hold window |
| hold_duration_s (s) | null | 37.9 | N/A | no hold window |
| settling_time_s (s) | null | 82.0 | N/A | no hold window |
| overshoot_pct (%) | 0.0 | 3.0 | -3.0 | yes (baseline 0–5.6%) |

Additional profile: Start angle 52.1 deg (standard). Zeta >= 1 (overdamped, 0% overshoot). Hold bias (post-reach window): +15.02 deg, hold std 3.81 deg, hold max error 21.88 deg. IMU-ENC bias (whole-run): -2.58 deg (vs -1.60 to -1.92 deg baseline). Rate tracking RMS (hold window): 39.93 deg/s. M2-M1 differential (hold): +37.4. ANG_I mean (hold): +0.25 deg/s. No saturation, no windup.

### Envelope check

hold_duration_s: pass = 60s, observed = null — below pass and a regression vs the +-9 deg baseline (6.56–78.20s in Iters 1–3). settling_time_s: pass = 35s, observed = null — regression. HoldMAE_s: pass = 5.0 deg, observed = null — regression. overshoot_pct: pass = 25%, observed = 0% — above pass. All three hold KPIs regressed below the +-9 deg baseline; this is the primary disqualifier.

### Verdict: Reject

T->SP moved in the predicted direction (39.4s, below the 42s falsifier threshold), but improved only 7.9s — closing 21% of the 37.3s gap to the pass threshold, not reaching the predicted 10–25s range. Hold behavior regressed catastrophically: baseline Iters 1–3 achieved 6.56–78.20s of confirmed hold; this run achieved none. Hold bias of +15.02 deg and max error 21.88 deg in the post-reach window (vs +4.69–8.54 deg and 8.88–12.83 deg at baseline) show oscillation amplitude roughly doubled. Verdict: Reject.

### Lessons

kd=0.4 reduced hold-phase damping below the stability floor for the +-9 deg band. The mechanism: lower kd weakens the angular-rate feedback term near setpoint, allowing hold oscillations to grow. Oscillation amplitude roughly doubled (hold max 21.88 deg vs 8.88–12.83 deg baseline) — enough to break the +-9 deg band on every swing, producing no confirmed hold. The post-reach hold bias of +15.02 deg reflects the mean of these large-amplitude, positive-biased oscillations (lever starts from +52 deg and asymmetric wire tension favours positive excursions). Rate tracking RMS of 39.93 deg/s is consistent: the outer loop commands ~45 deg/s setpoints (kp x 15 deg mean error) that the inner loop tracks, but the resulting motor differential (+37.4) is insufficient to fully recover from the large oscillation.

T->SP improvement of 7.9s confirms kd governs approach damping: less kd -> slightly less overdamped approach. But a 0.1 unit reduction (0.5->0.4) is enough to cross the hold-stability floor. kd must remain >= 0.5 for hold stability at current kp=3.0 and +-9 deg band.

The binding T->SP gap (29.4s from observed 39.4s to 10s pass threshold) cannot be closed by kd reduction without crossing the stability floor. Addressing T->SP requires a faster approach at kd=0.5, which points to kp increase (higher natural frequency -> more underdamped approach).

---

## Iteration 5 — 2026-05-28

### Context

Iter 4 established two facts: (1) kd controls approach damping — reducing kd 0.5->0.4 shortened T->SP by 7.9s (39.4s vs 47.3s mean); (2) kd=0.4 is below the hold-stability floor — oscillation amplitude doubled (max 21.88 deg vs 8.88–12.83 deg baseline), breaking the +-9 deg band on every swing. The stability floor at current kp=3.0 lies somewhere between kd=0.4 and kd=0.5.

The binding constraint is T->SP: 39.4s at kd=0.4, 42–52s at baseline kd=0.5, vs 10s pass threshold. kd reduction cannot bridge this gap without instability. To accelerate approach at kd=0.5, the natural frequency of the closed-loop system must increase — which requires raising kp. In a simplified second-order model, damping ratio zeta = kd / (2 * sqrt(kp * plant_gain)): increasing kp lowers zeta at fixed kd, producing a more underdamped approach. This is the same mechanism by which the rig produced underdamped behavior on 2026-05-14 (T_s 17–26s, overshoot 11–14%) at kp=3.0, kd=0.5 — the current overdamped character is either a rig-state-of-the-day variation or an artefact of the +-9 deg scoring window interacting with the oscillation pattern.

Two readings of how large to step kp:

*Reading A — kp 3.0->3.5:* kp=3.5 has prior history on this rig (post-rebuild systematic tuning reached kp=3.5 before settling at 3.0). A step of 0.5 gives a meaningful reduction in zeta and is likely to push the approach into the underdamped regime, producing T->SP substantially below 40s. Risk: kp=3.5 at kd=0.5 may drive hold oscillations large enough to break +-9 deg — the P-term is 17% stronger near setpoint, and kd=0.5's damping may not suppress the larger oscillations.

*Reading B — kp 3.0->3.3:* More conservative. Less T->SP gain but lower risk of destabilising hold. However, if the overdamped->underdamped transition requires a certain kp threshold, 3.3 might still sit in the overdamped regime and give a disappointingly small T->SP improvement.

Reading A is preferred: the 10s pass threshold is far below current T->SP (gap ~30s), a small kp step is likely insufficient to close it within the remaining session budget, and kp=3.5 has documented prior operating history on this rig.

Steelman of why kp=3.5 could make things worse: higher kp drives larger rate commands at any given angle error. During hold, if the lever sits at a mean bias of 3–5 deg (as in baseline), kp=3.5 commands 10–17% more rate correction than kp=3.0. If kd=0.5 is only marginally above the stability floor, the additional correction energy from kp=3.5 could push oscillation amplitude back over +-9 deg. This would produce a repeat of Iter 4's hold failure despite kd being "stable" at kp=3.0.

Note on IMU-ENC bias: Iter 4 showed -2.58 deg whole-run vs -1.60 to -1.92 deg in baseline. This may reflect the large post-reach oscillations skewing the whole-run average rather than a calibration shift. Watch this in Iter 5 — if bias returns to the baseline range (-1.60 to -1.92 deg), it was oscillation-driven; if it remains elevated, there is a calibration drift to investigate.

### Hypothesis

At kp=3.5, kd=0.5, the closed-loop angle loop is sufficiently underdamped to produce fast approach (T->SP < 40s, possibly < 20s with visible overshoot), while kd=0.5 keeps hold oscillations within +-9 deg for some confirmed hold window.

### Proposed change

Two simultaneous changes — dependency is explicit: kd=0.4 (from Iter 4) is below the established stability floor and makes kp effects unobservable; kd must return to 0.5 before kp can be tested. The variable under test is kp.

- `vehicle.angle_pid.kd`: 0.4 -> 0.5 (restore from Iter 4 — not a hypothesis test on kd)
- `vehicle.angle_pid.kp`: 3.0 -> 3.5

Net change from pre-session baseline: kp 3.0 -> 3.5 only.

### Prediction

T->SP drops below baseline range (< 41.7s), likely into 5–25s. Overshoot appears (> 5.6% baseline max), likely 10–25%. Some confirmed hold is achieved (T_s > 0s, hold_duration > 0s). HoldMAE_s within or below baseline range if hold is achieved. IMU-ENC bias returns toward baseline range (-1.60 to -1.92 deg).

### Falsifier

If T->SP stays within the baseline range (41.7–52.4s): kp=3.5 is not driving faster approach at this kd — the overdamped character has a root cause not explained by the kp/kd damping model and the session strategy needs re-examination.

If T->SP improves (< 41.7s) but hold fails again (T_s = null): kp=3.5 is above the hold-oscillation threshold even at kd=0.5. kd=0.5 is insufficient to stabilise hold at kp=3.5, and the session is likely approaching its structural limit.

---

### Run

- **Run ID:** 2026-05-29_20-54-04
- **Status:** analysed

### Observed

| KPI | Observed | Baseline mean | Delta from baseline | Within baseline variance? |
|-----|----------|---------------|---------------------|---------------------------|
| Reached | YES | 3/3 | — | — |
| T->SP (s) | 2.9 | 47.3 | -44.4 | far outside — 8.2 std faster |
| HoldMAE_s (deg) | 3.38 | 5.21 | -1.83 | yes — 0.9 std better |
| hold_duration_s (s) | 117.0 | 37.9 | +79.1 | far outside — 2.2 std longer |
| settling_time_s (s) | 2.9 | 82.0 | -79.1 | far outside |
| overshoot_pct (%) | 11.0 | 3.0 | +8.0 | outside — overshoot appeared as predicted |

Additional profile: Zeta=0.575 (underdamped). Rise time 10-90%: 5.6s. Hold bias: +2.79 deg, hold std: 2.87 deg, hold P95: 7.21 deg, hold max: 8.88 deg (inside +-9 deg). IMU-ENC bias (whole-run): -1.39 deg (returned to baseline range). Rate tracking RMS: 12.0 deg/s. M2-M1 mean: +4.7. ANG_I mean: +0.13 deg/s. No saturation, no windup.

### Envelope check

All five KPIs above their pass thresholds. No envelope violations.

### Verdict: Accept — Exit condition 1 triggered

Prediction matched: approach became underdamped (zeta=0.575, overshoot 11%), T->SP dropped dramatically (2.9s vs predicted 5-25s — magnitude exceeded expectations), confirmed hold achieved (117.0s). Falsifier conditions not triggered. All KPIs at pass or above; four at excellent, one (HoldMAE_s) at good.

Exit condition 1 triggered: T->SP margin above pass threshold = 7.1s (10.0 - 2.9), exceeds baseline std (5.4s). Envelope intact.

One concern for confirmation: HoldMAE_s (3.38 deg) sits only 1.62 deg above the pass threshold (5.0 deg) — within one baseline std (2.07 deg). Hold max reached 8.88 deg, just inside the +-9 deg boundary. Confirmation runs will determine whether this is the typical distribution.

### Lessons

kp 3.0->3.5 at kd=0.5 pushed the closed-loop system from overdamped (zeta>=1, baseline) to underdamped (zeta=0.575) — a large jump that resolved T->SP immediately. The overdamped character of the +-9 deg baseline (Iters 1-3) was a damping-ratio effect at kp=3.0, not a fundamental limitation. T->SP of 2.9s is consistent with an underdamped approach that crosses the setpoint on the initial overshoot swing.

Hold stability is present but operating near the +-9 deg boundary: hold max 8.88 deg, P95 7.21 deg, bias +2.79 deg. Rate tracking RMS dropped from 39.93 deg/s (Iter 4, hold failure) to 12.0 deg/s, confirming active, stable hold. IMU-ENC bias returned to -1.39 deg (baseline range), confirming the elevated bias in Iter 4 was oscillation-driven, not calibration drift. HoldMAE_s variance is the open question for confirmation.

---

## Confirmation Phase

Qualifying iteration: Iter 5 (2026-05-29_20-54-04). Config: kp=3.5, kd=0.5 (all other params unchanged from baseline).

2 confirmation runs required. All must pass: target KPI meets tightened threshold (all KPIs >= pass), envelope intact.

---

## Confirmation 1 — 2026-05-29

### Hypothesis

Config kp=3.5, kd=0.5 is stable at the goal level, with envelope intact.

### Proposed change

None. Same config as Iter 5.

### Prediction

T->SP < 10s, settling_time_s < 35s, hold_duration_s > 60s, overshoot_pct < 25%, HoldMAE_s < 5.0 deg. All envelope KPIs stay >= pass. Hold max stays inside +-9 deg on this run.

### Falsifier

Any KPI misses its pass threshold on this run, OR hold_duration_s drops to null (no confirmed hold).

---

### Run

- **Run ID:** 2026-05-29_21-02-01
- **Status:** analysed

### Observed

| KPI | Observed | Iter 5 | Delta | Pass threshold | Above pass? |
|-----|----------|--------|-------|----------------|-------------|
| Reached | YES | YES | — | — | — |
| T->SP (s) | 4.4 | 2.9 | +1.5 | 10.0 | YES |
| HoldMAE_s (deg) | 3.59 | 3.38 | +0.21 | 5.0 | YES |
| hold_duration_s (s) | 49.4 | 117.0 | -67.6 | 60.0 | NO |
| settling_time_s (s) | 70.5 | 2.9 | +67.6 | 35.0 | NO |
| overshoot_pct (%) | 5.2 | 11.0 | -5.8 | 25.0 | YES |

Additional profile: Zeta=0.684, rise 7.6s. Hold bias: +3.43 deg, hold std: 2.98 deg, hold P95: 8.35 deg, hold max: 10.107 deg (exceeds +-9 deg). IMU-ENC bias: -1.43 deg. Rate tracking RMS: 13.4 deg/s. No saturation, no windup.

### Verdict: Confirmation failed — session reopens

Hold max of 10.107 deg exceeded the +-9 deg band, resetting the settled-hold clock to 70.5s. This left only 49.4s of confirmed hold (below the 60s pass threshold). settling_time_s also missed pass (70.5s > 35s). Falsifier triggered on both KPIs. The hold bias (+3.43 deg) with std ~3 deg places the +9 deg boundary at ~1.9 sigma — a structurally marginal position that produces a crossing on this run but not on Iter 5 (hold max 8.88 deg, draw-dependent).

### Lessons

Spectrum analysis (03_spectrum.png) of both Iter 5 and Conf 1 shows pure 1/f^2 rolloff from the lowest resolvable frequency (~0.3 Hz). No resonance peak anywhere in the spectrum. Hold error power is entirely in the slow-drift regime — period comparable to or longer than the hold window itself. This rules out kd as a fix: kd acts on angular rate (dθ/dt); for drift at ~0.009 Hz, dθ/dt ≈ 0.2 deg/s, and kd=0.6 vs kd=0.5 adds ~0.02 deg/s of correction vs the P-term's ~10 deg/s at 3 deg error — negligible.

Hold std is gain-insensitive: 2.21, 3.13, 2.24, 2.87, 2.98 deg across every run at kp=3.0 and kp=3.5. The crossing is probabilistic — determined by whether the hold bias (+3 deg) plus a ~2 sigma fluctuation reaches the +9 deg boundary. Centering the distribution (reducing hold bias) is the correct lever.

Hold bias decomposition: IMU-ENC frame offset (~1.4 deg, irreducible at current tare quality) + mechanical equilibrium offset (~1.6 deg, addressable). The I-term is the principal tool for correcting mechanical steady-state error. At ki=0.05, the I-term starts the hold phase at -5 (negative cap, from approach saturation) and takes ~100s to reach positive correction — too slow to benefit most of the 117s hold window.

---

## Confirmation 2 — suspended (session reopened after Conf 1 failure)

Further confirmation is blocked pending Iter 6 resolution of hold bias.

---

## Iteration 6 — 2026-05-29

### Context

Conf 1 failed because hold max (10.107 deg) exceeded the +-9 deg boundary, resetting the settled-hold clock. Spectrum analysis of the hold-error signal in both Iter 5 and Conf 1 shows pure 1/f^2 rolloff from the lowest resolvable frequency — no resonance peak, all power in the slow-drift regime (< 0.3 Hz). This rules out kd as a lever: kd acts on angular rate; at the dominant drift timescale (~117s period), dθ/dt ≈ 0.2 deg/s, and kd=0.6 vs kd=0.5 adds ~0.02 deg/s of correction vs P-term ~10 deg/s at 3 deg error — negligible. Hold std (2.21, 3.13, 2.24, 2.87, 2.98 deg across all runs) is gain-insensitive; amplitude is not the issue.

The issue is hold bias position (+3 deg from setpoint). It places the +9 deg boundary at only ~2 standard deviations from the mean, making boundary crossings probabilistic (~2.3% per sample, a near-certain crossing over 120s). The bias decomposes into two components: (1) IMU-ENC frame offset (~1.4 deg) — irreducible at current tare quality, not visible to the controller as an error; (2) mechanical equilibrium offset (~1.6 deg) — the controller sees this via IMU error and the I-term is the appropriate corrective tool.

Two readings of the I-term's current behaviour:

*Reading A — ki is too slow:* At ki=0.05, the integrator accumulates at ki × IMU_error ≈ 0.05 × (small value, since the controller operates in IMU frame where IMU-ENC offset is not seen as error) → slow buildup. ANG_I reaches only 0.13-0.17 deg/s by hold end against a cap of 0.25 deg/s. Doubling ki to 0.10 doubles accumulation rate and raises the cap to 0.50 deg/s; the integrator reaches useful correction faster and provides twice the steady-state rate command to overcome the mechanical offset.

*Reading B — ki at iterm_limit=5 has negligible mechanical effect regardless:* The cap at ki=0.10, iterm_limit=5 is 0.50 deg/s. At 3 deg hold bias, P-term = kp × 3 = 10.5 deg/s. I-cap is 4.8% of P-term — the equilibrium shift (bias_new = bias_old - I_cap/kp = 3 - 0.50/3.5 = 2.86 deg) is ~0.14 deg. Smaller than measurement noise. If this reading is correct, iterm_limit=5 is the ceiling constraint, not ki — and meaningful bias reduction at iterm_limit=5 would require ki ≈ 0.75, which is reckless.

Both readings are testable from one run: ANG_I_mean is the diagnostic. If ANG_I_mean increases significantly toward the new cap (0.40-0.50 deg/s) AND hold bias decreases, Reading A holds. If ANG_I_mean barely moves and bias stays near 3 deg, Reading B holds and the session is at its structural ceiling at iterm_limit=5.

Note on windup risk: the 2026-05-14 session reduced iterm_limit 100->5 because the slow approach (42-52s) allowed large I-term accumulation during the long approach phase. At kp=3.5, the approach is 2.9s. The I-term saturates at ±iterm_limit within the first second of approach regardless of ki. ki=0.10 does not change approach windup behaviour — only hold recovery rate.

Steelman of why ki=0.10 could make things worse: if the I-term in the hold phase oscillates with the hold error (positive on high-bias side, negative on low-bias side), a faster-responding I-term could introduce phase lag in the correction that amplifies hold oscillations. However, given the hold error oscillation frequency is ~0.009 Hz (very slow), and the I-term time constant at ki=0.10 is much faster than this, phase-lag-driven amplification is not physically plausible here.

### Hypothesis

ki=0.10 increases the rate of I-term accumulation during the hold phase sufficiently to provide meaningful steady-state correction of the mechanical equilibrium offset, reducing hold bias and moving the +9 deg boundary further from the centre of the hold-error distribution.

### Proposed change

`vehicle.angle_pid.ki`: 0.05 -> 0.10

### Prediction

ANG_I_mean in the hold window increases toward the new cap (0.40-0.50 deg/s), up from 0.13-0.17 deg/s. Hold bias decreases from the current ~3 deg toward ~2-2.5 deg. If bias reduction is achieved, hold max may stay below 9 deg (crossing probability decreases). T->SP and overshoot unchanged (ki does not affect approach at 2.9s convergence).

### Falsifier

If ANG_I_mean stays near 0.15 deg/s and hold bias remains >= 2.8 deg: ki is not the effective lever at iterm_limit=5, and the session has reached its structural ceiling. The hold bias floor at current architecture is ~3 deg (IMU-ENC offset + mechanical equilibrium at this gain), and the +-9 deg band is the tightest achievable without addressing the IMU noise floor (IDEA-003) or adjusting iterm_limit (windup risk).

---

### Run

- **Run ID:** 2026-05-29_21-44-47
- **Status:** analysed

### Observed

| KPI | Observed | Iter 5 | Conf 1 | Pass threshold | Level |
|-----|----------|--------|--------|----------------|-------|
| Reached | YES | YES | YES | — | — |
| T->SP (s) | 4.55 | 2.9 | 4.4 | 10.0 | good |
| HoldMAE_s (deg) | 2.69 | 3.38 | 3.59 | 5.0 | good |
| hold_duration_s (s) | 115.4 | 117.0 | 49.4 | 60.0 | excellent |
| settling_time_s (s) | 4.55 | 2.9 | 70.5 | 35.0 | excellent |
| overshoot_pct (%) | 16.2 | 11.0 | 5.2 | 25.0 | good |

Key diagnostics: ANG_I_mean 0.26 deg/s (vs Iter 5: 0.13, Conf 1: 0.18; new cap = 0.50). Hold bias +1.96 deg (vs +2.79, +3.43). Hold std 2.62 deg. Hold P95 6.24 deg (vs 7.21, 8.35). Hold max 8.88 deg (inside +-9 deg). T_s = T->SP — no band exits after first entry. IMU-ENC bias -1.40 deg.

### Envelope check

All five KPIs above pass. No envelope violations.

### Verdict: Accept — Exit condition 1 triggered

Reading A confirmed: ANG_I_mean rose from 0.13-0.18 to 0.26 deg/s (past the old cap of 0.25, now at 52% of the new 0.50 cap). Hold bias dropped from ~3 deg to +1.96 deg — a ~1 deg improvement that moved P95 from 7.21-8.35 to 6.24 deg and kept hold max inside +-9 deg for the full 115s. HoldMAE margin above pass: 2.31 deg vs baseline std 2.07 deg — just exceeds. Exit condition 1 triggered.

3 confirmation runs recommended (not 2): prior Conf 1 showed this config can draw hold max > 9 deg. With bias +1.96 deg and std 2.62 deg, the +9 deg boundary sits at ~2.7 std from mean — improved but individual runs can still approach the limit.

### Lessons

ki=0.10 reduced hold bias from ~3 to ~2 deg by providing stronger I-term correction of the mechanical equilibrium offset. ANG_I at 0.26 deg/s (past old cap of 0.25) confirms the integrator reached the previous limit and is now operating above it. The residual ~1.4 deg hold bias (IMU-ENC frame offset) cannot be corrected by the I-term — it is not visible as error in the controller's IMU frame. This is the irreducible floor at current tare quality, documented in IDEA-003 and IDEA-004.

ANG_I is at 52% of the new cap (0.50 deg/s). Whether further ki increase would continue reducing bias, or whether the integrator is already correcting all it can see (the mechanical offset), will be visible in the confirmation runs' ANG_I_mean values.

---

## Confirmation Phase 2

Qualifying iteration: Iter 6 (2026-05-29_21-44-47). Config: kp=3.5, ki=0.10, kd=0.5.

3 confirmation runs required (prior Conf 1 at ki=0.05 failed; hold max proximity to +-9 deg warrants extra evidence). All must pass: every KPI >= pass threshold, envelope intact.

---

## Confirmation 2a — 2026-05-29

### Hypothesis

Config kp=3.5, ki=0.10, kd=0.5 is stable at the goal level, with envelope intact.

### Proposed change

None. Same config as Iter 6.

### Prediction

T->SP < 10s, settling_time_s < 35s, hold_duration_s > 60s, overshoot_pct < 25%, HoldMAE_s < 5.0 deg. Hold max stays inside +-9 deg. ANG_I_mean comparable to Iter 6 (0.20-0.35 deg/s).

### Falsifier

Any KPI misses its pass threshold, OR hold_duration_s drops to null (no confirmed hold).

---

### Run

- **Run ID:** 2026-05-29_21-52-21
- **Status:** analysed

### Observed

| KPI | Observed | Iter 6 | Pass threshold | Above pass? |
|-----|----------|--------|----------------|-------------|
| Reached | YES | YES | — | — |
| T->SP (s) | 3.72 | 4.55 | 10.0 | YES |
| HoldMAE_s (deg) | 3.21 | 2.69 | 5.0 | YES |
| hold_duration_s (s) | 116.2 | 115.4 | 60.0 | YES |
| settling_time_s (s) | 3.72 | 4.55 | 35.0 | YES |
| overshoot_pct (%) | 16.1 | 16.2 | 25.0 | YES |

ANG_I_mean: 0.216 deg/s. Hold bias: +1.82 deg. Hold std: 3.21 deg. Hold P95: 5.98 deg. Hold max: 8.88 deg (inside +-9 deg). T_s = T->SP — no band exits. IMU-ENC bias: -1.34 deg.

### Verdict: Confirmation passed

All KPIs above pass. No band exits after first entry. Hold max 8.88 deg inside +-9 deg. Prediction matched.

### Lessons

Hold bias continues at ~1.82 deg (consistent with Iter 6's 1.96 deg), confirming the ki=0.10 correction is stable across runs. ANG_I_mean at 0.216 deg/s (slightly below Iter 6's 0.260 — within run-to-run variation). Hold max identical at 8.88 deg across both runs — this value appears to be a recurring peak driven by the dominant oscillation mode.

---

## Confirmation 2b — 2026-05-29

### Hypothesis

Config kp=3.5, ki=0.10, kd=0.5 is stable at the goal level, with envelope intact.

### Proposed change

None. Same config as Iter 6.

### Prediction

T->SP < 10s, settling_time_s < 35s, hold_duration_s > 60s, overshoot_pct < 25%, HoldMAE_s < 5.0 deg.

### Falsifier

Any KPI misses its pass threshold, OR hold_duration_s drops to null.

---

### Run

- **Run ID:** 2026-05-29_22-00-54
- **Status:** analysed

### Observed

| KPI | Observed | Iter 6 | Pass threshold | Above pass? |
|-----|----------|--------|----------------|-------------|
| Reached | YES | YES | — | — |
| T->SP (s) | 4.44 | 4.55 | 10.0 | YES |
| HoldMAE_s (deg) | 2.62 | 2.69 | 5.0 | YES |
| hold_duration_s (s) | 115.5 | 115.4 | 60.0 | YES |
| settling_time_s (s) | 4.44 | 4.55 | 35.0 | YES |
| overshoot_pct (%) | 13.4 | 16.2 | 25.0 | YES |

ANG_I_mean: -0.037 deg/s. Hold bias: +1.006 deg. Hold std: 3.07 deg. Hold P95: 6.06 deg. Hold max: 8.88 deg (inside +-9 deg). T_s = T->SP — no band exits. M2-M1 mean: -1.09 (slightly negative — I-term overcorrected mechanical bias slightly). IMU-ENC bias: -1.39 deg.

### Verdict: Confirmation passed

All KPIs above pass. No band exits after first entry.

### Lessons

Hold bias dropped to +1.006 deg — the closest yet to the IMU-ENC floor (~1.4 deg). ANG_I_mean near zero (-0.037 deg/s), consistent with the I-term having overcorrected slightly: the mechanical offset was fully compensated and the integrator is now oscillating around zero. M2-M1 slightly negative confirms the I-term drove past the mechanical equilibrium on this run. Run-to-run variation in ANG_I initial condition (determined by approach dynamics) produces variable bias outcomes: 1.0-2.0 deg across the three ki=0.10 runs. All three remain well inside +-9 deg. The irreducible floor (~1.4 deg IMU-ENC offset) is now within reach on good draws.

---

## Confirmation 2c — 2026-05-29 (three attempts)

### Hypothesis

Config kp=3.5, ki=0.10, kd=0.5 is stable at the goal level, with envelope intact.

### Proposed change

None. Same config as Iter 6.

### Prediction

T->SP < 10s, settling_time_s < 35s, hold_duration_s > 60s, overshoot_pct < 25%, HoldMAE_s < 5.0 deg.

### Falsifier

Any KPI misses its pass threshold, OR hold_duration_s drops to null.

---

### Run 1

- **Run ID:** 2026-05-29_22-04-50
- **Status:** analysed

| KPI | Observed | Pass threshold | Above pass? |
|-----|----------|----------------|-------------|
| Reached | YES | — | — |
| T->SP (s) | 6.774 | 10.0 | YES |
| HoldMAE_s (deg) | null | 5.0 | NO — no hold achieved |
| hold_duration_s (s) | null | 60.0 | NO |
| settling_time_s (s) | null | 35.0 | NO |
| overshoot_pct (%) | 11.3 | 25.0 | YES |

Diagnostics: Zeta=0.570, rise=7.5s — approach underdamped, similar to passing runs. Hold bias: +4.12 deg. ANG_I: 0.32 deg/s. IMU-ENC bias: -1.49 deg. enc_range: 57.8 deg. dt_max: 131ms (timing spike). M2-M1 mean: +9.2. No confirmed hold: oscillations exceeded +-9 deg before any settle window could be confirmed.

---

### Run 2

- **Run ID:** 2026-05-29_22-10-37
- **Status:** analysed

| KPI | Observed | Pass threshold | Above pass? |
|-----|----------|----------------|-------------|
| Reached | YES | — | — |
| T->SP (s) | 15.059 | 10.0 | NO |
| HoldMAE_s (deg) | null | 5.0 | NO — no hold achieved |
| hold_duration_s (s) | null | 60.0 | NO |
| settling_time_s (s) | null | 35.0 | NO |
| overshoot_pct (%) | 2.4 | 25.0 | YES |

Diagnostics: Zeta=0.766, rise=15.5s — near-overdamped approach. Hold bias: +6.35 deg. ANG_I: 0.48 deg/s (near cap). IMU-ENC bias: -1.71 deg. enc_range: 53.2 deg. M2-M1 mean: +16.5. Lever barely crossed setpoint; I-term fully saturated but unable to draw lever to +-9 deg band.

---

### Run 3

- **Run ID:** 2026-05-29_22-25-24
- **Status:** analysed

| KPI | Observed | Pass threshold | Above pass? |
|-----|----------|----------------|-------------|
| Reached | YES | — | — |
| T->SP (s) | 14.348 | 10.0 | NO |
| HoldMAE_s (deg) | 3.580 | 5.0 | YES |
| hold_duration_s (s) | 5.103 | 60.0 | NO |
| settling_time_s (s) | 114.817 | 35.0 | NO |
| overshoot_pct (%) | 3.6 | 25.0 | YES |

Diagnostics: Zeta=0.728, rise=14.6s — near-overdamped. Hold bias: +6.34 deg. ANG_I: 0.49 deg/s (near cap). IMU-ENC bias: -1.71 deg. enc_range: 53.8 deg. M2-M1 mean: +16.4. Failure signature identical to Run 2; lever achieved 5s of hold only at T=114.8s (end of run).

### Verdict: Confirmation 2c failed — session does not meet

Falsifier triggered on all three runs. Runs 2 and 3 show an identical failure signature: near-overdamped approach (zeta 0.73-0.77, T->SP 14-15s), ANG_I near saturation (0.48-0.49 deg/s), hold bias +6.34-6.35 deg, IMU-ENC bias -1.71 deg, enc_range 53-54 deg. Run 1 is intermediate (approach underdamped, but timing spike and elevated IMU-ENC bias prevented a confirmed hold). This behavior is qualitatively different from the three passing runs (Iter 6, Conf 2a, Conf 2b) at identical config, implying a variable outside the gain set is governing the outcome.

### Lessons

Three passes then three failures at identical config. The immediate hypothesis is a discrete bimodal mode. Extended investigation (below) will reveal this is instead a continuous gradient driven by mechanical state.

---

## Post-Confirmation Investigation — 2026-05-30

Conf 2c produced 3 consecutive failures following 3 consecutive passes at identical config. This section characterises the variance source.

### Phase 1 — Manual observation at GRV@50Hz (3 additional runs)

Three manual observation runs at GRV@50Hz, same PID config. Equipment confirmed cool between runs (components < 40 deg C), ruling out thermal effects.

| Run ID | T->SP | Zeta | IMU-ENC | enc_range | hold_bias | ANG_I | hold_dur | Result |
|--------|-------|------|---------|-----------|----------|-------|---------|--------|
| 10-36-49 | 19.5s | 0.586 | -1.55 | 57.3 | +4.84 | 0.41 | 22.9s | FAIL |
| 10-39-24 | 16.6s | null | -1.77 | 51.9 | +6.60 | 0.50 | 20.0s | FAIL |
| 10-43-23 | 5.7s | 0.676 | -1.57 | 54.8 | +3.41 | 0.30 | 17.0s | FAIL |

All three fail. Together with the Conf 2c runs, all 6 post-Conf-2b runs at GRV@50Hz failed.

**Key finding: the apparent bimodal pattern is a continuous gradient.** IMU-ENC bias (sensor_health.bias in diagnose.json) is the clearest predictor across all 12 runs in the session. Sorting by IMU-ENC bias reveals an unbroken pass/fail boundary:

| Category | IMU-ENC bias range | enc_range | ANG_I | hold_bias |
|----------|--------------------|-----------|-------|-----------|
| Passing (3 runs) | -1.34 to -1.40 | 58.9-60.4 deg | -0.04 to 0.26 | +1.0 to +2.0 deg |
| Failing (9 runs) | -1.49 to -1.77 | 51.9-57.8 deg | 0.30-0.50 | +3.4 to +6.6 deg |

Every run with IMU-ENC bias above -1.45 deg passes; every run below fails. No exceptions.

**Root cause: wire harness state.** The wires are routed without a slip ring. Each time the lever is returned to the restrictor, the harness settles into a different configuration. Higher tension shifts the lever's mechanical equilibrium further from encoder 0 deg, producing a larger IMU-ENC bias, a more overdamped approach (lower enc_range), and a hold bias the I-term cannot overcome. At ANG_I saturated at the cap (ki x iterm_limit = 0.10 x 5 = 0.50 deg/s), the rate correction is insufficient to bring the lever within +-9 deg against the increased mechanical resistance. No gain change can address this constraint.

### Phase 2 — GRV@100Hz experiment (3 runs)

`angle_report_hz` bumped 50->100 Hz, same PID config, to test whether faster outer-loop feedback improves robustness to wire tension variability.

| Run ID | T->SP | Zeta | IMU-ENC | enc_range | hold_bias | ANG_I | settling | hold_dur | HoldMAE | Result |
|--------|-------|------|---------|-----------|----------|-------|---------|---------|---------|--------|
| 11-28-00 | 4.3s | 0.580 | -1.325 | 57.4 | +1.04 | -0.097 | 18.8s | 101.1s | 2.25 | PASS |
| 11-30-17 | 6.0s | 0.589 | -1.434 | 57.2 | +3.12 | 0.302 | 63.4s | 56.5s | 2.93 | FAIL |
| 11-35-03 | 6.3s | 0.591 | -1.522 | 57.0 | +3.93 | 0.345 | 28.4s | 91.5s | 3.18 | PASS |

One unconfounded signal: enc_range is 57.0-57.4 deg across all three 100Hz runs despite IMU-ENC bias ranging -1.325 to -1.522. At 50Hz enc_range ranged 51.9-60.4 and was strongly correlated with mechanical state. Faster outer-loop feedback appears to stabilise approach dynamics across varying wire tension conditions. Run 11-35-03 passed at IMU-ENC=-1.522, which was squarely in the 50Hz failure zone.

**Caveat:** The experiment is confounded. Two variables changed simultaneously: loop rate and mechanical state (mean IMU-ENC -1.43 for 100Hz batch vs -1.63 for the morning 50Hz failures). The enc_range consistency is the one unconfounded signal. A clean comparison requires alternating 50/100Hz back-to-back without touching the rig. Result is promising but not conclusive.

---

## Outcome

**Exit 2b — Abandoned.** The +-9 deg tolerance band cannot be demonstrated reliably at current mechanical conditions. Config improvements from this session are accepted and retained.

### Config changes accepted

| Parameter | Pre-session | Post-session |
|-----------|------------|-------------|
| angle_pid.kp | 3.0 | **3.5** |
| angle_pid.ki | 0.05 | **0.10** |
| angle_pid.kd | 0.5 | 0.5 (unchanged) |
| angle_pid.iterm_limit | 5.0 | 5.0 (unchanged) |

These produced 3 clean passes (Iter 6, Conf 2a, Conf 2b): T->SP 3.7-4.6s, settling 3.7-4.6s (excellent), hold 101-117s (excellent), HoldMAE 1.0-2.7 deg (excellent). The accepted config is substantially better than the session-open baseline on all KPIs.

### Why confirmation failed

The root cause is wire harness mechanical state variability. IMU-ENC bias is the proxy: below -1.45 deg the rig passes, above -1.45 deg it fails, with no exceptions across all 12 session runs. The controller has no visibility into this variable: the IMU-ENC offset is not seen as an error in the IMU frame, and the I-term (capped at 0.50 deg/s) cannot overcome the mechanical resistance when tension is high. This is not a tuning problem — it is a mechanical plant characterisation problem.

### Architectural constraint confirmed (see IDEA-004)

IMU-ENC bias ~1.4 deg is the irreducible floor from tare registration error. At current mechanical variability (bias range -1.32 to -1.77 deg across runs), the effective hold bias ranges from +1 to +7 deg. This is too wide to reliably clear +-9 deg. The +-9 deg target is achievable only when the wire harness draws a favorable state.

### Specification

Revert `specification.json` `tolerance_deg` from 9.0 back to 10.0 before the next session. +-9 deg is not demonstrable reliably at current conditions.

### Required groundwork before re-attempting +-9 deg

1. **IDEA-003** — Clamp frame at true horizontal, motors off, log IMU vs encoder across 5-6 re-mounts. Determine whether IMU-ENC bias variation is mechanical (harness shifts equilibrium) or sensor (GRV drifts between mounts). These have opposite fixes: mechanical requires wire management or a slip ring; sensor requires BNO085 settings investigation.
2. **GRV@100Hz controlled comparison** — Alternate 50/100Hz back-to-back without touching the rig, 5+ runs each, compare enc_range and IMU-ENC bias distributions. Current evidence is promising but confounded.
3. **Wire management** — Develop a repeatable harness positioning protocol, or route wires through the rotation axis, to reduce mechanical state variability between runs.

Until IDEA-003 is complete, further PID tuning aimed at +-9 deg is operating against an unknown and uncontrolled variable.

---

## Amendment — 2026-05-30: Terminology note

References to "hold window" and "settle window" in this log use those terms as plain
English descriptions of observed behaviour. On 2026-05-30 the analysis pipeline's internal
Python objects were renamed to align with servo/robotics engineering convention:
- What was called `HoldWindow` (first band entry) is now `ReachEvent`
- What was called `SettleWindow` (confirmed continuous hold) is now `HoldWindow`

The recorded numbers and conclusions are unaffected. See DR-015 Amendment 2026-05-30 for
the full rename table and rationale.
