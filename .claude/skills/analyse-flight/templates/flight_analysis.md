# Flight Analysis: {FLIGHT_ID}

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | {FLIGHT_ID} |
| Duration | {DURATION_S} s |
| Samples | {N_SAMPLES} |
| Start angle | {START_ANGLE}° |
| Standard start | {STANDARD_START_YN} |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp={ANGLE_KP}, ki={ANGLE_KI}, kd={ANGLE_KD}, iterm_limit={ANGLE_ITERM_LIMIT} |
| rate_pid | kp={RATE_KP}, ki={RATE_KI}, kd={RATE_KD}, iterm_limit={RATE_ITERM_LIMIT} |
| motor | base={BASE_THROTTLE}, min={THROTTLE_MIN}, max={THROTTLE_MAX} |
| feedforward_lead_ms | {FF_LEAD_MS} |
| angle_report | {ANGLE_REPORT} @ {ANGLE_REPORT_HZ} Hz |
| rate_report | {RATE_REPORT} @ {RATE_REPORT_HZ} Hz |

---

## Raw Tool Output

### score_flight.py

```
{SCORE_OUTPUT_VERBATIM}
```

### profile_flight.py

```
{PROFILE_OUTPUT_VERBATIM}
```

### Plot

`test_runs/flights/{FLIGHT_ID}/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | {REACHED_YN} |
| T→0 (s) | {T_TO_REACH_S} |
| HoldMAE (°), post-reach | {HOLD_MAE_DEG} |
| T@0 (s) | {T_AT_ZERO_S} |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | {ACTUAL_HZ} |
| IMU-ENC MAE (°) | {IMU_ENC_MAE} |
| IMU-ENC bias (°) | {IMU_ENC_BIAS} |
| Pearson r | {PEARSON_R} |
| IMU trails motion (%) | {TRAIL_PCT} |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | {OSC_FREQ_HZ} | |
| Whole-run ENC MAE (°) | {WHOLE_RUN_ENC_MAE} | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | {ANG_WINDUP_EVENTS} | |
| Rate windup events | {RATE_WINDUP_EVENTS} | |

## Observations

{One bullet per diagnostic area. Use only the areas that have something worth
saying — omit a bullet if there is nothing to add beyond what the tables show.
Suggested leads (adapt or add as needed):

- **Hold accuracy** — tie HoldMAE to a visible plot feature; note whether the
  error is bias-dominated (lever hovering off-zero) or oscillation-dominated
  (symmetric wobble). Mention oscillation frequency only if it explains HoldMAE.

- **Control loop** — flag unexpected I-term behaviour in either direction: buildup
  that should not be there, or absence despite a persistent offset. Explain the
  arithmetic when the windup count is zero but the numbers look suspicious.

- **Motor balance** — note M1/M2 asymmetry during hold and what it is compensating
  for. State what can and cannot be concluded without thrust bench data.

- **Sensor health** — IMU-ENC bias, Pearson r, trail %. Call out anything that
  deviates from "small, consistent, well-correlated".

- **Mechanical / timeline artefacts** — gaps, spikes, sudden mode changes, anything
  that does not fit the categories above.

Rules: tie every number to a plot feature or an arithmetic explanation. Apply the
absence lens (missing signals are as diagnostic as unexpected ones). No tuning
advice; no comparisons to other runs.}