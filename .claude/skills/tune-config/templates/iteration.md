## Iteration <N> — <YYYY-MM-DD HH:MM>

### Context

<Free prose. Three things, in any order or interleaving:>
<  1. What the data shows (recent iterations in this session + Starting State baseline).>
<  2. What stands out about it — alternative readings if multiple are plausible.>
<  3. Why the proposed change follows from that reading.>

<Standard: the human can follow the reasoning and verify it against their own mental model.>
<Silence on visible signals is failure. Either incorporate or explicitly dismiss with reasoning.>

### Hypothesis

<One sentence. The mental model being tested.>

### Proposed change

<Exact config diff. One variable.>
<Format: field-path: old → new>
<Example: vehicle.angle_pid.iterm_limit: 100 → 200>

### Prediction

<What the KPIs should look like if the hypothesis is right.>
<Be specific. "HoldMAE should improve" is not specific. "HoldMAE should drop below 4.5°" is.>

### Falsifier

<What would prove the hypothesis wrong. Concrete and measurable.>
<Example: "If HoldMAE worsens by more than baseline std dev (>0.4°), the hypothesis is wrong.">

---

<Below this line: filled after the run completes.>

### Run

- **Run ID:** <run_id>
- **Status:** <analysed | failed | unanalysed>

### Observed

<KPIs from the new run, pulled from analysis.md. Direct values, no interpretation.>

| KPI | Observed | Baseline mean | Δ from baseline | Within baseline variance? |
|---|---|---|---|---|
| Reached | <yes/no> | <baseline> | — | — |
| T→SP (s) | <value> | <value> | <±value> | <yes/no> |
| HoldMAE_s (°) | <value> | <value> | <±value> | <yes/no> |
| T@SP (s) † | <value> | <value> | <±value> | <yes/no> |
| Oscillation (Hz) | <value> | <value> | <±value> | <yes/no> |

*† T@SP = run duration − T→SP. Overstates time in band on runs where the lever left and re-entered the ±10° band after first reach. Check the profile section for re-excursion before treating this value as face value.*

### Verdict

<One of: Accept / Reject / Inconclusive.>
<Reasoning explicitly references the Prediction and the Falsifier above.>
<Verdict rules:>
<  - Accept: observed matches Prediction AND falls outside baseline variance in the predicted direction.>
<  - Reject: observed matches Falsifier criterion, OR moves opposite to Prediction by more than baseline variance.>
<  - Inconclusive: observed is within baseline variance of prior state, OR does not clearly match either Prediction or Falsifier.>

<If an exit condition is triggered (goal met with margin > baseline std dev / 3-iteration budget exhausted / goal unreachable), include the recommendation here.>

### Lessons

<Mandatory. What this iteration taught us about the system.>
<"Nothing new — prediction matched cleanly" is acceptable.>
<Empty is not.>
