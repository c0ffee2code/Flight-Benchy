---
name: tune-config
description: Pair-tune the Flight Benchy control algorithm toward a stated objective by adjusting config.json values only. Reads recent run history, identifies a baseline, proposes config changes with full reasoning, and tracks the session as a structured markdown log. Use when the user names a tuning objective like "reduce HoldMAE", "increase hold time", "fix the residual oscillation" — anything where the goal is to improve a measurable KPI through iterative config changes. Triggers on phrases like "tune", "tune config", "pair tune", "let's improve <KPI>", "tuning session for <objective>", "new tuning session". Does NOT cover source code changes (e.g. editing pid.py or mixer.py — out of scope, the skill only modifies config.json), one-off config edits without a tracked objective, fully autonomous tuning (this skill keeps the human in the loop on every iteration), or comparing two specific runs (out of scope until spectator exists). The skill produces a session log under tuning/ and is the only producer of those files.
---

## Arguments

```
<objective in plain language>
```

The objective is given in natural language at session open. The skill is responsible for translating it into a measurable form (KPI + threshold) and confirming with the user before any iteration begins.

# Tune Flight

Pair-tuning the Flight Benchy with a human in the loop on every iteration. The agent reads, reasons, drafts, and proposes. The human reviews, approves, edits, or rejects. Flights happen between checkpoints. The session log captures every hypothesis, prediction, falsifier, observation, verdict, and lesson — so weeks later you can read back what was tried, what was learned, and where reasoning was sound vs not.

## Why pair-tuning, not autopilot

Two reasons.

First: the failures on this rig are physical and human-resolvable. When the PSU latches into over-current protection, no agent cleverness restarts it. When a wire chafes through, no telemetry repairs it. The agent's best action in those situations is "stop and tell the human," and once you accept that, asking the human between every iteration stops looking like overhead and starts looking like *using the human well*.

Second: the bottleneck isn't decision speed. A flight is two minutes; the bench runs on wall power; iterations cost real time only in the flight itself. The cognitive work — *what should we try next and why* — is where this skill earns its keep. Automating the human's role saves almost nothing. Automating the *reasoning* role gives the human a calibrated second opinion at every iteration.

The discipline this skill imposes — predict before observing, name falsifiers, surface alternative readings — is harder to maintain when one person is doing everything. Pair-tuning makes it the default.

## Why this skill exists

A tuning session without structure produces a sequence of changes that locally make sense but accumulate into something nobody can audit. Three runs from now you cannot tell whether the current config is good *because of* the last change or *despite* it. The session log is the audit trail. Every iteration explains itself before the run, and verifies itself after.

The goal is not to produce optimal configs faster. The goal is to produce understanding that compounds — so that the tenth session's iteration 1 starts from a richer mental model than the first session's did.

## Hard rules — apply throughout the session

- **Predict before observing.** Every iteration writes Prediction and Falsifier *before* the run. Post-hoc narrative is the failure mode this rule exists to prevent. If you find yourself wanting to revise a prediction after seeing the result, stop — the original prediction stays, and the discrepancy becomes the lesson.

- **One variable per iteration.** The session's whole point is cause-and-effect attribution. Changing two variables at once destroys that. If the agent believes two changes are jointly necessary, it must propose them as separate iterations with explicit reasoning for the dependency.

- **Every claim names a telemetry observation that supports it.** Same rule as in `postmortem-flight`. "The rate loop is undertuned" is not a claim; "gyro_x shows a 0.05 Hz residual oscillation of ±2°/s during the hold phase" is a claim.

- **No comparison to other tuning sessions during iteration reasoning.** The current session's reasoning lives on its own evidence. If a prior session is relevant — same objective attempted before, similar starting state — the agent surfaces that at session *open* (as part of Starting State), not mid-session. **[CHECK — this mirrors the postmortem Phase 1/Phase 2 split. I think it's right but it's the most likely rule to chafe in practice.]**

- **The agent recommends; the human decides.** Every checkpoint and every exit recommendation is a draft for human review. The agent never deploys config or closes the session unilaterally.

- **Strategic communication is the auditability mechanism.** Every iteration's Context field must do three things: summarise the relevant data, name what stands out about it, and justify why the proposed change follows from that reading. Vague or data-free Context is grounds for the human to reject the proposal and ask for re-reasoning. The standard is: *the human can follow the reasoning and verify it against their own mental model.*

## The session lifecycle

### Session open

The user states the objective in plain language. The agent:

1. **Proposes the measurable form.** Translates the natural-language objective into a KPI threshold. Examples: "increase hold window from 20s to 30s" → "T@SP ≥ 30s"; "reduce HoldMAE_s below 5°" → "HoldMAE_s ≤ 5°". The agent confirms the mapping with the user before proceeding. If the objective is ambiguous ("make it more stable"), the agent asks what KPI to target.

   KPI vocabulary matches `analysis.md` output: `T→SP` (time to first reach), `HoldMAE_s` (encoder MAE post-settle), `T@SP` (derived: run duration − T→SP). Note on `T@SP`: it is a proxy, not a direct measurement — it assumes the lever stays in band after first reach. On runs with significant re-excursion, `T@SP` overstates actual hold time. When `T@SP` is the session's target KPI, the agent must check for re-excursion in the profile section before interpreting `T@SP` values as face value.

2. **Pulls recent project history.** Runs the history reader:
   ```
   python .claude/skills/tune-config/scripts/history_reader.py --n 10
   ```
   Reads the structured output. This is the raw material for everything that follows in this session.

3. **Identifies the baseline.** From the history reader output, the "Baseline analysis" section names which runs match the current config. The agent checks baseline sufficiency:

   - **Sufficient (≥3 analysed runs at current config):** proceed to step 4.
   - **Insufficient (<3 analysed runs at current config):** the agent's *first proposal* is "run K baseline runs at current config before any tuning iteration." K = (3 − existing count). The agent presents this as the iteration 1 proposal with the hypothesis "baseline KPIs are not yet established; we cannot measure tuning impact against undefined noise." The user approves; baseline runs happen; *then* the session continues with the real iteration 1.

4. **Writes the Starting State block.** From the history reader's Baseline analysis section. This block is **frozen** for the rest of the session — it is the reference point against which all subsequent results are measured. Includes:
   - Current config (all PID gains, motor base, feedforward lead if present).
   - Baseline KPI means and standard deviations across the baseline runs.
   - List of baseline run IDs.

5. **Drafts iteration 1.** Following the iteration template. Presents to user for pre-run review.

6. **Creates the session file.** Path: `tuning/<YYYY-MM-DD>-<objective-slug>.md`. The slug is a short kebab-case description of the objective ("reduce-holdmae", "increase-hold-window"). If a file with the same name exists, append `-2`, `-3`, etc. Use the session template at `.claude/skills/tune-config/templates/session.md` as the starting structure. Initial contents: Objective, Starting State (frozen), and iteration 1 draft (status: pending pre-run approval).

### Per-iteration loop

Each iteration follows the same shape (see `.claude/skills/tune-config/templates/iteration.md`), with two human checkpoints.

**Pre-run draft.** The agent fills Context / Hypothesis / Proposed change / Prediction / Falsifier.

The Context standard from the hard rules applies in full. See "The Context standard" section below for principles and examples.

The agent presents the draft in chat, labeled `**Iteration N, pre-run review:**`. No summary above the draft — show the full content. The human approves, edits via reply, or rejects with reasoning.

On approval, the agent:

1. Writes the iteration's pre-run fields to the session file — this must happen before the run starts. The file write is the enforcement mechanism for the predict-before-observing rule: predictions exist on disk before the result does.
2. Deploys the proposed config and runs the flight by invoking `/run-flight` (which handles reset, deploy, run, fetch, analyse).
3. Waits for the run to complete and `analyse-flight` to produce the new `analysis.md`.

**Failure detection.** If `run-flight` reports the run did not complete normally — smoke check failed, mpremote error, premature termination — the session immediately enters the **Failed** state. The agent does *not* attempt to recover or rerun. See "Exit: Failed" below.

**Post-run draft.** Once the analysis is in, the agent reads the new run's `analysis.md`, re-invokes the history reader for updated context, and fills:

- **Observed** — KPIs from the new run, pulled from analysis.md. Direct values, no interpretation yet.
- **Verdict** — *Accept*, *Reject*, or *Inconclusive*. With reasoning that explicitly references the iteration's Prediction and Falsifier. Verdict rules:
  - *Accept* — observed result matches Prediction AND falls outside baseline variance in the predicted direction.
  - *Reject* — observed result matches the Falsifier criterion, OR moves opposite to the Prediction by more than baseline variance.
  - *Inconclusive* — observed result is within baseline variance of the prior state, OR does not clearly match either Prediction or Falsifier.
- **Lessons** — mandatory. What this iteration taught us about the system. "Nothing new — prediction matched cleanly" is acceptable. Empty is not. The act of writing it surfaces whether iterations are still informative.

After the post-run draft, the agent also assesses whether any **exit condition** has been triggered (see below) and surfaces it as part of the Verdict if so.

Presented in chat as `**Iteration N, post-run review:**`. Same approval mechanic as pre-run.

On approval, the agent writes the post-run fields to the session file and (if no exit condition triggered) drafts iteration N+1.

### Exit conditions

The agent recommends; the human confirms. Three exits:

**Exit 1 — Met and confirmed.**

Triggered when an iteration's observed KPI clears the goal threshold *by more than baseline standard deviation* in the right direction. Example: goal is HoldMAE_s ≤ 5°, baseline std dev is 0.4°, observed is 4.5°. Margin = 0.5°, which exceeds 0.4° → eligible for confirmation. Observed 4.7° would *not* be eligible — within noise.

When triggered, the agent's Verdict on the qualifying iteration includes: "Goal met, exceeds baseline variance by <margin>. Recommend transitioning to confirmation runs."

On user approval, the session transitions into **confirmation phase**:

- Run **2 additional iterations at the same config** as the qualifying iteration (no change). 3 additional if the qualifying margin is within 20% of the goal threshold — i.e. the win is narrow and needs more evidence.
- Each confirmation iteration uses the same format as a normal iteration, but:
  - Hypothesis: "Config X is stable at the goal level."
  - Proposed change: none.
  - Prediction: KPIs match the qualifying iteration within baseline variance.
  - Falsifier: any KPI misses the goal threshold.
- All confirmation runs must pass for the session to close as **Met and confirmed**. If any confirmation run misses the threshold, the win was a fluke — the session reopens for further iteration. **[CHECK — alternative is "majority must pass." I lean strict, but flagging.]**

On successful confirmation, the session's Outcome section is filled and the file is committed.

**Exit 2 — Abandoned.**

Two sub-triggers:

*(a) Budget exhausted.* After 3 iterations (not counting confirmation runs, since those don't happen for an abandoned session), the cumulative movement of the target KPI toward the goal is within baseline variance. The strategy isn't working. The agent's Verdict on iteration 3 includes: "3 iterations completed, no meaningful movement toward goal. Recommend abandoning this session and regrouping."

*(b) Goal structurally unreachable.* At any iteration's post-run checkpoint, the agent may conclude — with reasoning — that the goal is below the achievable floor of the current system. Example: targeting HoldMAE ≤ 0.5° when GRV sensor floor is ~0.8°. The Verdict surfaces this as a recommendation: "Goal appears unreachable for reasons X, Y, Z. Recommend abandoning and revising the objective."

In both cases, on user approval, the session's Outcome is filled with reason and the file is committed.

**Exit 3 — Failed.**

Triggered by `run-flight` not completing normally — smoke check failure, mpremote error, or any indication the run did not produce a valid analysis. The agent does *not* try to recover.

The session immediately enters Failed state. The agent's actions:

1. Record the failure in the current iteration (it will be incomplete — no Observed/Verdict/Lessons).
2. Write the Outcome section as **Failed**, including:
   - Iteration number that was active.
   - Reference to the failed run ID.
   - Note: "Tuning conclusions from this session are not trusted pending postmortem."
3. Tell the user: "Session entered Failed state. Run `/postmortem-flight <run_id>` to investigate. After postmortem, append the proximate cause as a final lesson to this session before closing."

The postmortem hand-off is a *workflow step*, not an automated action. The agent surfaces the next step and stops. The user runs postmortem manually, then returns to the session file and adds the proximate cause as a final lesson — this is the link that prevents tuning lessons from getting stranded inside postmortem reports.

## The Context standard

Context is the load-bearing field of every iteration. Vague Context produces unverifiable reasoning and rubber-stamp approvals. The standard is: **the human can follow the reasoning and verify it against their own mental model.**

Four principles guide the Context field:

1. **Surface alternative readings of the data, not just the first one that comes to mind.** Forward-reasoning analogue of postmortem's two-chains rule. Honest investigation considers more than one story.

2. **Do not ignore signals in the data, even when they don't fit the current hypothesis.** Either incorporate the signal as evidence, or explicitly dismiss it with reasoning. Silence is failure.

3. **Prefer diagnostic experiments over extrapolation.** When the data supports multiple readings, the next change should *distinguish* them. "More of what worked last time" is fine when you understand why it works; it's lazy when you don't.

4. **Pre-name what each possible outcome would mean.** Not just "what would prove me wrong" but "what would each possible result tell me." This makes the post-run Verdict mechanical instead of narrative.

### Example: weak Context (what we don't want)

> The last three iterations increased angle_kp from 4.0 to 5.0 and HoldMAE_s improved from 6.2° to 5.0°. The trend is positive but appears to be slowing. I propose increasing angle_kp further to 5.5 to continue the improvement.

Why this is weak:

- Data summary present but interpretation is shallow.
- Justification is "more of the same" — extrapolation, not understanding.
- Ignores motor asymmetry visible in the data — either it's relevant and was missed, or it's irrelevant and silence still fails.
- Predicts nothing the next run could disprove. "Continue the improvement" is vague.

This is Context that *looks* reasonable on first read and is doing zero reasoning work.

### Example: strong Context

> Iterations 1–3 increased angle_kp from 4.0 → 4.5 → 5.0 and HoldMAE_s improved 6.2° → 5.4° → 5.1° → 5.0°. The improvement is decelerating sharply — the first 0.5 step gave 0.8° improvement, the last gave 0.1°. Two readings of this:
>
> (a) We're approaching the gain ceiling for this loop — higher kp would start exciting oscillation, and the diminishing return is the warning sign before that happens. Oscillation freq is still 0.05 Hz across all three iterations, which is consistent with "approaching but not yet at" instability.
>
> (b) angle_kp isn't the binding constraint anymore. Something else is now setting the floor. The persistent motor asymmetry (M2 ~30 units above M1 during hold) suggests the angle PID is fighting a steady disturbance, and the I-term should be absorbing that — but iterm_limit=100 caps the I contribution at 5 deg/s, which may not be enough.
>
> Reading (b) is more interesting because it points at a different mechanism, and the diagnostic is cheap: if the I-term is consistently pegged at its limit during hold, raising iterm_limit should help; if it isn't pegged, raising it does nothing and we know the asymmetry is from somewhere else.
>
> I propose: hold angle_kp at 5.0, raise iterm_limit from 100 to 200, single variable. Prediction: if (b) is correct, HoldMAE_s drops below 4.5° and M1/M2 asymmetry narrows. If (a) is correct (kp ceiling), HoldMAE_s moves by less than 0.1° and asymmetry persists. Either outcome is informative.

Why this is strong:

- Two readings, evaluated against each other.
- The second signal in the data (motor asymmetry) is named and used as evidence.
- The proposed experiment is chosen because the two readings predict different outcomes — diagnostic, not extrapolative.
- Both outcomes are pre-named. The post-run Verdict becomes mechanical: compare observed to the two predictions, see which one fits.

## What to avoid

- **Multi-variable changes.** Even when two changes "obviously go together," they go in separate iterations. Cause-and-effect attribution is the session's whole purpose.

- **Declaring victory on a single run.** A KPI that clears the goal threshold once is not a win. The baseline-variance gate + confirmation runs exist precisely because single-run results can be noise.

- **Approving rubber-stamps.** If the agent's Context doesn't actually justify the proposed change, the human's job is to reject and ask for re-reasoning. Approving thin Context defeats the whole skill.

- **Reasoning across sessions.** This session lives on its own evidence. Patterns across sessions are a different skill (spectator, not yet built).

- **Tuning recommendations after Failed exit.** Once a session enters Failed state, no further config conclusions are drawn from it until postmortem. The session's iterations may have produced misleading evidence; that's exactly what postmortem will determine.

---

## Open questions for review

Two points where I'd like your read before this is final:

1. **The "no comparison to other sessions" hard rule.** Strict version above. The cost: if you remember a prior session mid-iteration, the skill tells you to ignore that memory until session open of the *next* session. Right discipline, awkward in practice?

2. **Confirmation runs: all-pass vs majority-pass.** Strict version above (any miss reopens the session). Alternative is majority-must-pass. Strict matches "the win was real" more honestly; majority forgives single-run variance even at confirmed config.
