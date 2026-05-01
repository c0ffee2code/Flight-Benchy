---
name: analyse-flight
description: Analyse a single Flight Benchy stabilisation run end-to-end — generate the diagnostic plot, compute KPIs (HoldMAE, T→0, T@0), profile sensor and control-loop health, and write a structured analysis.md report. Use this whenever the user points at a single run folder under test_runs/ (typically a YYYY-MM-DD_hh-mm-ss timestamp) and wants to know how it went. Triggers on phrases like "analyse flight", "check this flight". Does NOT cover comparison across multiple runs, PID tuning recommendations, or general drone control theory — for those, answer directly without invoking this skill.
---

# Analyse Flight

Procedural skill for evaluating a single Flight Benchy stabilisation run. Every flight goes through the same three-step pipeline and produces a consistent `flight_analysis.md`, so runs stay directly comparable over time.

## What is a flight

A flight is one stabilisation session stored in `test_runs/flights/YYYY-MM-DD_hh-mm-ss/`. The folder contains:

- `log.csv` — 22-column telemetry, typically ~50 ms / 20 Hz (inner loop runs faster; `TELEMETRY_SAMPLE_EVERY` decimates).
- `config.json` — full system snapshot at run time: PID gains, motor limits, IMU rates, feedforward, encoder config.

Standard test convention: M1-end resting on the restrictor at ≈ **+58°**, algorithm lifts the lever to within ±10° of horizontal (0°) and holds. Non-standard starts can still be analysed, but their KPIs are not comparable to standard runs and the report should flag this.

## Pipeline contract

The three scripts are designed to be run in order:

```
plot.py           → plot.png
score_flight.py   → stdout (KPI pass/fail table);  compute_kpis() importable by profile_flight.py
profile_flight.py → imports compute_kpis directly;  prints canonical KPIs then grouped diagnostics
```

Run all three before writing the report.

## Pre-flight checks

Before invoking the pipeline, behave as follows:

| Situation | Behaviour |
|-----------|-----------|
| `log.csv` missing or fewer than ~5 rows | Tell the user, stop. Don't fabricate a report. |
| `config.json` missing | Tell the user, stop. `config.json` is mandatory — the pipeline cannot run without it. |
| Start angle outside ±10° of +58° | Run anyway. Flag prominently in the report's Run Identity section ("Standard start: NO"); state that hold-quality KPIs aren't comparable to standard runs. |

## Step 1 — Plot

```
python .claude/skills/analyse-flight/scripts/plot.py test_runs/flights/<flight_id>
```

Produces `plot.png`. Five subplots, top to bottom: angle tracking (ENC vs IMU), rate tracking (setpoint vs gyro), outer PID terms, inner PID terms, motor outputs. The plot's docstring has the full per-subplot guide.

**After running, view the PNG.** Use the `view` tool on `test_runs/flights/<flight_id>/plot.png` and form a short visual impression — this feeds into the Observations section later. Things actually worth noting:

- Does the encoder line settle near 0° or hover off-zero? (off-zero → tare or steady disturbance)
- Does the IMU line track the encoder cleanly, or is there visible bias / lag?
- Is the rate setpoint being chased, or does the gyro lag noticeably?
- Are outer-loop I-terms drifting up over the hold? (sustained → steady-state disturbance)
- Do M1 and M2 sit symmetrically during hold? (asymmetry → mechanical bias or I-term compensation)
- Anything weird that doesn't fit a category — gaps, spikes, sudden mode changes.

The plot is the richest signal in the whole pipeline. The numbers in steps 2 and 3 will confirm or contradict what you saw — both directions are useful information.

**After the visual pass — check in with the user before proceeding.**
Share what you see in 2–3 sentences and ask if anything unusual happened
during the session: mechanical changes, a power supply hitting its limit,
a loose connection, an unusual start. The user is right there at the bench
and can resolve in one sentence what no metric can. Surface anomalies as
questions, not conclusions — "I see M1 running ~50 units higher than M2
throughout the hold — is the frame balanced, or is there a known asymmetry?"
Don't interpret a hardware problem as a control problem in silence.

## Step 2 — Score

```
python .claude/skills/analyse-flight/scripts/score_flight.py test_runs/flights/<flight_id>
```

Computes the KPIs and prints them to stdout. KPI definitions live in the script's docstring. The big one is **HoldMAE** — mean absolute encoder deviation from 0° over all post-reach samples. That's the headline hold-accuracy number; everything else contextualises it.

If `Reached = NO`, skip step 3. In the report, state the fact plainly: "Did not reach horizontal." The plot and KPI table show what happened — no further interpretation needed in Observations.

## Step 3 — Deep-dive stats

```
python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs/flights/<flight_id>
```

Prints grouped diagnostics: sample rate, sensor tracking (IMU vs ENC), setpoint error over the whole run, correlation, oscillation frequency, windup events. Group meanings are in the script's docstring.

Two things to watch for in the output:

- The "Whole-run MAE includes the rise" note. Whole-run ENC MAE and HoldMAE measure different things; don't conflate them in the report.
- Windup event counts are computed against the `iterm_limit` values from `config.json`, which is always present. The counts are always config-derived and can be trusted.

## Writing the report

1. Read the template at `.claude/skills/analyse-flight/templates/flight_analysis.md`.
2. Fill each placeholder from `score_flight.py` stdout, the `profile_flight.py` stdout, and `config.json`. The profile output echoes canonical KPIs at the top — use that for the KPI Scorecard section. Placeholder names are unique — if the same value appears twice, fill both.
3. Save the completed report to `test_runs/flights/<flight_id>/analysis.md`, overwriting any existing file.
4. Tell the user where the file was saved.

The Raw Output section must contain script stdout verbatim — that's the ground-truth record. Don't paraphrase, don't reformat tables, don't drop columns. Future-you will rerun a script and compare against this; tampering breaks that.

The Observations section is the part that earns its keep. Aim for a short paragraph (3–6 sentences) that says what *this specific run* is doing. Useful patterns:

- Tie a KPI to a visual feature ("HoldMAE of 1.2° matches the visible 1° low-frequency wobble in subplot 1").
- Note when something looks fine but a number disagrees, or vice versa ("plot looks clean but oscillation frequency is 2.3 Hz — likely the small high-frequency ripple in subplot 4").
- Flag mechanical or sensor signals ("M1/M2 asymmetric by ~50 throttle units during hold — frame is unbalanced or angle I-term is compensating for it").
- Call out anything that should not be there: gaps in the timeline, a spike, an unexpected mode.
- Apply the same lens to absences: no I-term buildup despite a persistent offset, symmetric motors when asymmetry is expected — missing signals are as diagnostic as unexpected ones.

Avoid: tuning recommendations ("try lowering ki"), comparisons to other runs (out of scope for this skill), and restating numbers that are already in the tables above.