# Tuning session — <objective slug>

## Objective

<One paragraph stating what we are trying to achieve.>

- **Target KPI:** <e.g. T@0>
- **Threshold:** <e.g. ≥ 30s>
- **Why this matters:** <one sentence — what does success unlock>

## Starting state

**Frozen at session open. Do not modify mid-session.**

### Current configuration

- **angle_pid:** kp=<value>, ki=<value>, kd=<value>, iterm_limit=<value>
- **rate_pid:** kp=<value>, ki=<value>, kd=<value>, iterm_limit=<value>
- **motor.base:** <value>
- **feedforward.lead_ms:** <value or "absent">
- **setpoint:** roll=<value>, pitch=<value>, yaw=<value>

### Baseline KPIs

Computed across N baseline runs at the current configuration:

| KPI | Mean | Std Dev | Range | Sample size |
|---|---|---|---|---|
| Reached rate | <fraction> | — | — | <N> |
| T→0 (s) | <mean> | <std> | <min>–<max> | <N> |
| HoldMAE (°) | <mean> | <std> | <min>–<max> | <N> |
| T@0 (s) | <mean> | <std> | <min>–<max> | <N> |
| Oscillation (Hz) | <mean> | <std> | <min>–<max> | <N> |

### Baseline runs

- <run_id_1>
- <run_id_2>
- <run_id_3>
- <...>

## Iterations

<Each iteration follows the iteration template — see templates/iteration.md.>
<Iterations are appended in order. The session continues until an exit condition is met.>

## Outcome

<Filled at session close. One of the three states below; remove the others.>

### Met and confirmed

- **Final configuration:** <config diff from Starting State>
- **Qualifying iteration:** <iteration N>
- **Confirmation runs:** <run_id_1>, <run_id_2>, [<run_id_3>]
- **Final KPI:** <observed value> (goal: <threshold>, baseline mean: <value>, margin: <value>)

### Abandoned

- **Reason:** <budget exhausted | goal unreachable>
- **Reasoning:** <one paragraph — what the iterations showed, why the agent recommended abandonment>
- **Iterations completed:** <N>

### Failed

- **Active iteration when failure occurred:** <iteration N>
- **Failed run ID:** <run_id>
- **Postmortem reference:** <link to postmortem.md once produced, or "pending">
- **Warning:** Tuning conclusions from this session are not trusted pending postmortem.
- **Final lesson (added after postmortem):** <proximate cause from postmortem, plus what it tells us about the tuning direction>
