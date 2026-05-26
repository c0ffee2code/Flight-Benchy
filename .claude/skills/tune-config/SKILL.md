---
name: tune-config
description: Pair-tune the Flight Benchy control algorithm to meet a tightened acceptance threshold in specification.json, by adjusting config.json values only. The user opens a session by asking to tighten one threshold in specification.json; that tightening becomes the session objective. The skill reads run history, identifies a baseline, proposes config changes iteratively with full reasoning, and tracks the session as a structured markdown log. Triggers on phrases like "new tuning session". Does NOT cover source code changes (only edits config.json and specification.json), one-off config edits without a tracked session, fully autonomous tuning (human-in-the-loop on every iteration), or cross-session comparison. Produces a session log under tuning/ and is the only producer of those files.
---

## Arguments

```
<criteria tightening request in plain language>
```

The user opens a session by requesting a tightening of one threshold in `specification.json`. Examples:

- "tighten HoldMAE pass to 4°"
- "push hold_time_s excellent from 60 to 90"
- "tighten T_s good to 8s"

The request names a KPI, a threshold level (`pass`, `good`, or `excellent`), and the new value. The agent:

1. Reads the current `specification.json`.
2. Confirms the parsed `(kpi, level, new_value)` with the user before any file edit.
3. On confirmation, edits specification.json — one threshold, one parameter (the same one-variable-at-a-time discipline that governs iterations).
4. That tightening *is* the session objective. The just-edited KPI is the session target; meeting the new threshold value is the goal.

If the user's request is ambiguous ("let's get HoldMAE under 4°" — which level?), the agent asks for clarification before editing. Loose phrasing is fine if the agent confirms its interpretation.

If `specification.json` does not yet exist in the project, see the **Missing specification.json** subsection below.

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

## The specification.json model

`specification.json` is the slowly-evolving spec of what success means for this rig. It lives at the project root next to `config.json` and is copied into each run folder by `SdSink`. Each KPI has three thresholds: `pass`, `good`, and `excellent`.

Two files; two rates of change:

- **`config.json`** changes every iteration — that's the search.
- **`specification.json`** changes once per session, at session open — that's the bar.

A session is the act of pulling config up to a tightened criteria bar. The discipline is recursive: one config parameter changes per iteration; one criteria threshold tightens per session. The session opens with a criteria edit, closes when config has caught up (Met) or when the tightening has been shown unreachable (Abandoned, recommend revert).

### Envelope semantics

When the session targets KPI X by tightening threshold X.level to a new value, **every other KPI's `pass` threshold remains an envelope constraint**:

- An iteration that improves X but pushes another KPI *below its pass threshold* → **Reject**, regardless of how well X moved.
- An iteration that improves X and another KPI drops a level *without going below pass* (e.g. excellent → good) → the Verdict is judged on X alone, but the trade-off must be named in Lessons. This is normal optimisation; future sessions need to see the trade-off was made deliberately, not silently.

The session's target KPI is exempt from envelope check on itself: tightening HoldMAE.pass from 5° to 4° means HoldMAE temporarily sits below the new pass threshold by definition — that's the whole point of the session.

### Missing specification.json

If `specification.json` does not exist when the session opens, the agent surfaces this and asks:

> "No specification.json found. Three options: (a) create one now with default thresholds and tighten as you described; (b) proceed in legacy mode — natural-language objective, no envelope check, hardcoded ±10° tolerance; (c) stop and create specification.json manually first. Which?"

The agent does not silently fall back to legacy mode; the choice must be explicit. Legacy mode disables the envelope check entirely and is intended only for projects that haven't adopted IDEA-001 yet.

### Outcome and the specification.json undo

The Outcome section of every session records what happened to specification.json:

- **Met and confirmed** → specification.json tightening stands; the session log is the record of how the rig got there.
- **Abandoned** → the agent recommends reverting the specification.json edit (or loosening it to a reachable level, with a specific value backed by session evidence). The human decides; the agent does not auto-revert. The session's Outcome includes the recommended new value and the evidence supporting it.
- **Failed** → specification.json state is whatever it was at session open; the session never produced evidence to support either keeping or reverting. Postmortem first, then revisit the tightening as a fresh session.

This is the engineering loop the skill enforces: *propose a tighter bar → prove you can meet it → keep the tighter bar; or prove you can't → revert with evidence.*



- **Predict before observing.** Every iteration writes Prediction and Falsifier *before* the run. Post-hoc narrative is the failure mode this rule exists to prevent. If you find yourself wanting to revise a prediction after seeing the result, stop — the original prediction stays, and the discrepancy becomes the lesson.

- **One variable per iteration.** The session's whole point is cause-and-effect attribution. Changing two variables at once destroys that. If the agent believes two changes are jointly necessary, it must propose them as separate iterations with explicit reasoning for the dependency.

- **Every claim names a telemetry observation that supports it.** Same rule as in `postmortem-flight`. "The rate loop is undertuned" is not a claim; "gyro_x shows a 0.05 Hz residual oscillation of ±2°/s during the hold phase" is a claim. The evidence base is the full `analysis.md` of the run, not just the headline KPI dict surfaced by the flight-runner subagent — secondary signals in the analysis prose count as telemetry observations too.

- **Derived KPIs must state their derivation on first use.** When a derived quantity (damping ratio ζ, T_s, overshoot %, windup integral, settling-band definition, etc.) appears in iteration reasoning for the first time in a session, its extraction method or formula must be stated or cited inline. "Damping ratio = 0.466" is not a claim until the reader can see how 0.466 was computed — log-decrement on which peaks, fit to which model, computed by which function in `analyse-flight`. Subsequent uses in the same session can rely on the first definition. The skill's analysis pipeline may evolve and KPI definitions may shift; freezing the definition in the session prevents silent drift in reasoning across time.

- **No comparison to other tuning sessions during iteration reasoning.** The current session's reasoning lives on its own evidence. If a prior session is relevant — same objective attempted before, similar starting state — the agent surfaces that at session *open* (as part of Starting State's prior-session field), not mid-session.

- **The agent recommends; the human decides.** Every checkpoint and every exit recommendation is a draft for human review. The agent never deploys config or closes the session unilaterally.

- **Strategic communication is the auditability mechanism.** Every iteration's Context field must do three things: summarise the relevant data, name what stands out about it, and justify why the proposed change follows from that reading. Vague or data-free Context is grounds for the human to reject the proposal and ask for re-reasoning. The standard is: *the human can follow the reasoning and verify it against their own mental model.* See `.claude/skills/tune-config/context-standard.md` for principles and worked examples — load that file when drafting any iteration's Context.

## Tools this skill uses

Two Python scripts do the mechanical work. The parent calls them via Bash and parses their
stdout as JSON. Context stays clean — script progress goes to stderr only.

- **`flight-runner`** (`python .claude/agents/flight-runner/run.py`) — at each iteration,
  executes the hardware pipeline: check_config → reset → deploy → run → pull. Stdout JSON:
  `completed` (with run_id) or `failed` (with stage, run_id if assigned, rig_state,
  error_summary). Contract documented in `.claude/agents/flight-runner/flight-runner.md`.

- **`flight-analyser`** (`python .claude/agents/flight-analyser/run.py <run_id>`) — after
  flight-runner completes, runs the analysis pipeline: gate → plots → verdict → diagnose →
  report. Stdout JSON: `completed` (with run_id, summary_md_path, headline_kpis) or `failed`
  (with stage, partial_summary_path, error_summary). On `completed`, the parent reads
  `summary.md` directly. Contract documented in `.claude/agents/flight-analyser/flight-analyser.md`.

- **`prior-session-finder`** (model: Haiku — pure file scan, no reasoning) — at session open,
  scans `tuning/` for prior sessions matching the current objective. Returns either a structured
  prior-session summary or null. Invoked once per session in step 4 of Session open.

If a script returns an unexpected JSON shape or exits unexpectedly, treat it as a failed
contract: do not paper over with assumptions, surface the discrepancy to the human.

## The session lifecycle

### Session open

The user states the tightening request in plain language. The agent:

1. **Parses and confirms the tightening.** Extracts `(kpi, level, new_value)` from the user's request. Reads current `specification.json` to see the current value. Presents the parsed edit to the user for confirmation:

   > "You want to tighten `specification.json` → `<kpi>.<level>` from `<current_value>` to `<new_value>`. The session target will be: get `<kpi>` to meet `<new_value>` on every confirmation run. Confirm?"

   On confirmation, the agent edits `specification.json` — one threshold, one parameter. This edit happens *before* any iteration; the criteria edit and the session are linked atomically (Met keeps it, Abandoned recommends revert).

   If `specification.json` is missing, see "Missing specification.json" in the model section above.

   If the user's request is ambiguous, the agent asks for clarification before editing. The agent never edits specification.json without explicit confirmation of the exact `(kpi, level, new_value)` triple.

2. **Pulls recent project history.** Runs the history reader:
   ```
   python .claude/skills/tune-config/scripts/history_reader.py --n 10
   ```
   Reads the structured output. This is the raw material for everything that follows in this session.

3. **Identifies the baseline.** From the history reader output, the "Baseline analysis" section names which runs match the current config. The agent checks baseline sufficiency:

   - **Sufficient (≥3 analysed runs at current config):** proceed to step 4.
   - **Insufficient (<3 analysed runs at current config):** the agent's *first proposal* is "run K baseline runs at current config before any tuning iteration." K = (3 − existing count). The agent presents this as the iteration 1 proposal with the hypothesis "baseline KPIs are not yet established; we cannot measure tuning impact against undefined noise." The user approves; baseline runs happen; *then* the session continues with the real iteration 1.

4. **Surfaces prior same-KPI sessions, if any.** Invokes the `prior-session-finder` subagent with the structured tightening (kpi, level, new_value, prior_value) and the `tuning/` directory. If the subagent returns a match, the agent writes a single-paragraph **Prior session note** in Starting State summarising: prior session date, prior Outcome (Met / Abandoned / Failed), final config delta from baseline at that time, and the single most relevant lesson — all sourced from the subagent's structured output. Strongest matches are prior sessions that targeted the same KPI, regardless of level or value. This is the one-time prior at session open — mid-session iterations do not reference it again. If the subagent returns null, omit the field entirely (do not write "none found"; absence is the default).

5. **Writes the Starting State block.** From the history reader's Baseline analysis section and the current `specification.json`. This block is **frozen** for the rest of the session — it is the reference point against which all subsequent results are measured. Includes:
   - Current config (all PID gains, motor base, feedforward lead if present).
   - Prior session note (if step 4 found one).
   - **Criteria snapshot:** the post-edit `specification.json` thresholds for all KPIs, with the session target line marked. For each non-target KPI, the agent computes which level (`pass`, `good`, `excellent`, or `below pass`) the baseline currently sits in — this is the envelope starting state and what Verdict checks against.
   - Baseline KPI means and standard deviations across the baseline runs.
   - List of baseline run IDs.

6. **Drafts iteration 1.** Following the iteration template. Presents to user for pre-run review.

7. **Creates the session file.** Path: `tuning/<YYYY-MM-DD>-<tightening-slug>.md`. The slug is a short kebab-case description of what was tightened — e.g. `holdmae-pass-4deg`, `holdtime-excellent-90s`, `ts-good-8s`. If a file with the same name exists, append `-2`, `-3`, etc. Use the session template at `.claude/skills/tune-config/templates/session.md` as the starting structure. Initial contents: Objective (the criteria edit), Starting State (frozen, including criteria snapshot), and iteration 1 draft (status: pending pre-run approval).

### Per-iteration loop

Each iteration follows the same shape (see `.claude/skills/tune-config/templates/iteration.md`), with two human checkpoints.

**Pre-run draft.** The agent loads `context-standard.md` and fills Context / Hypothesis / Proposed change / Prediction / Falsifier following its principles.

The agent presents the draft in chat, labeled `**Iteration N, pre-run review:**`. No summary above the draft — show the full content. The human approves, edits via reply, or rejects with reasoning.

On approval, the agent:

1. Writes the iteration's pre-run fields to the session file — this must happen before the run starts. The file write is the enforcement mechanism for the predict-before-observing rule: predictions exist on disk before the result does.
2. Runs `python .claude/agents/flight-runner/run.py` via Bash. Parses stdout as JSON:
   - **On `completed`:** `{status, run_id}`.
   - **On `failed`:** `{status, stage, run_id, error_summary, rig_state}`. → Session enters Failed state; see below.
3. Runs `python .claude/agents/flight-analyser/run.py <run_id>` via Bash. Parses stdout as JSON:
   - **On `completed`:** `{status, run_id, summary_md_path, headline_kpis}`.
   - **On `failed`:** `{status, stage, partial_summary_path, error_summary, rig_state}`. → Session enters Failed state; see below.
4. The parent does not see mpremote output, pull logs, or script progress (all on stderr). Only the parsed JSON from each script reaches the parent's context.

**Post-run draft (on `completed`).** The agent reads `analysis.md` directly from `analysis_md_path` — the headline KPI dict is for triage, but the analysis prose is the evidence base. The agent then re-invokes the history reader for updated context and fills:

- **Observed** — KPIs from the new run, pulled from analysis.md. Direct values, no interpretation yet. Includes the session target KPI *and* all other KPIs that have thresholds in specification.json (the envelope).
- **Envelope check** — for each non-target KPI: did the new run's value stay ≥ that KPI's `pass` threshold? Any KPI that went below pass is an envelope violation and forces Reject regardless of how the target moved. Level drops that stay above pass (e.g. excellent → good) are *not* envelope violations but must be named in Lessons as a deliberate trade-off.
- **Verdict** — *Accept*, *Provisionally consistent*, *Reject*, or *Inconclusive*. With reasoning that explicitly references the iteration's Prediction and Falsifier. Verdict rules:
  - *Accept* — observed result matches Prediction on the target KPI AND falls outside baseline variance in the predicted direction, **and** the Prediction was about a single measurement (not a distribution-level claim), **and** no envelope violations.
  - *Provisionally consistent* — observed target matches a Prediction that makes a claim about *all runs*, *every run*, or a change in distribution shape ("the slow mode is eliminated", "T_s consistently ≤ X"), **and** no envelope violations. N=1 cannot accept a distributional claim. This Verdict advances the iteration toward confirmation phase if exit-condition-1 also triggers, but is weaker than Accept and must not be quoted as "the change worked" in subsequent iteration Context.
  - *Reject* — observed result matches the Falsifier criterion, OR moves opposite to the Prediction by more than baseline variance, OR any envelope violation occurred (even if the target KPI moved as predicted). Envelope-driven Reject is a distinct sub-case: the change *did* help the target but broke spec elsewhere, and Lessons must name what broke.
  - *Inconclusive* — observed result is within baseline variance of the prior state on the target KPI, OR does not clearly match either Prediction or Falsifier. Envelope must still be intact; an envelope violation overrides Inconclusive to Reject.
- **Lessons** — mandatory. What this iteration taught us about the system. "Nothing new — prediction matched cleanly" is acceptable. Empty is not. If a non-target KPI dropped a level (even staying above pass), the Lesson must name the trade-off explicitly. The act of writing it surfaces whether iterations are still informative.

After the post-run draft, the agent also assesses whether any **exit condition** has been triggered (see below) and surfaces it as part of the Verdict if so.

Presented in chat as `**Iteration N, post-run review:**`. Same approval mechanic as pre-run.

On approval, the agent writes the post-run fields to the session file and (if no exit condition triggered) drafts iteration N+1.

**On `failed` from either agent.** Session immediately enters Failed state per Exit 3 — see below. The structured failure info (stage, run_id, rig_state, error_summary) goes verbatim into the iteration's incomplete record and the Outcome section. The agent does not attempt to recover or rerun; rig_state from the agent informs whether to tell the user "rig needs human attention" vs "rig is in a safe state, postmortem can proceed." A failure from `flight-analyser` carries `rig_state: ok` — hardware completed normally; only the analysis failed.

### Exit conditions

The agent recommends; the human confirms. Three exits:

**Exit 1 — Met and confirmed.**

Triggered when an iteration's observed target KPI clears the tightened criteria threshold *by more than baseline standard deviation* in the right direction, **and** the envelope check passed. Example: tightened HoldMAE.pass to 4°, baseline std dev is 0.4°, observed is 3.5°, all other KPIs stayed ≥ pass. Margin = 0.5°, exceeds 0.4° → eligible for confirmation. Observed 3.7° would *not* be eligible — within noise.

When triggered, the agent's Verdict on the qualifying iteration includes: "Tightened threshold met, exceeds baseline variance by `<margin>`, envelope intact. Recommend transitioning to confirmation runs." Note that this is compatible with a *Provisionally consistent* Verdict on a distributional claim — exit-condition-1 advances to confirmation regardless of Accept vs Provisionally consistent.

On user approval, the session transitions into **confirmation phase**:

- Run **2 additional iterations at the same config** as the qualifying iteration (no change). 3 additional if the qualifying margin is within 20% of the goal threshold — i.e. the win is narrow and needs more evidence.
- Each confirmation iteration uses the same format as a normal iteration, but:
  - Hypothesis: "Config X is stable at the goal level, with envelope intact."
  - Proposed change: none.
  - Prediction: target KPI meets the tightened threshold within baseline variance, all envelope KPIs stay ≥ pass.
  - Falsifier: target KPI misses the tightened threshold on any run, OR any envelope KPI drops below pass on any run.
- All confirmation runs must pass for the session to close as **Met and confirmed**. If any confirmation run misses the target threshold OR breaks the envelope, the win was a fluke — the session reopens for further iteration.

On successful confirmation, the session's Outcome section is filled (specification.json tightening stands) and the file is committed.

**Exit 2 — Abandoned.**

Two sub-triggers:

*(a) Budget exhausted.* After 3 iterations (not counting confirmation runs, since those don't happen for an abandoned session), the cumulative movement of the target KPI toward the tightened threshold is within baseline variance. The strategy isn't working. The agent's Verdict on iteration 3 includes: "3 iterations completed, no meaningful movement toward `<kpi>.<level>` = `<new_value>`. Recommend abandoning this session and reverting the specification.json tightening."

*(b) Tightening structurally unreachable.* At any iteration's post-run checkpoint, the agent may conclude — with reasoning — that the tightened threshold is below the achievable floor of the current system. Example: tightened HoldMAE.pass to 0.5° when GRV sensor floor is ~0.8°. The Verdict surfaces this as a recommendation with a specific loosened value: "Tightening to `<new_value>` appears unreachable for reasons X, Y, Z. Recommend reverting to `<prior_value>`, or loosening to `<suggested_value>` (justified by observed floor of ~`<observed_min>`)."

In both cases, on user approval, the session's Outcome is filled with:
- Reason for abandoning.
- Recommended action on specification.json: revert to prior value, or loosen to a specific new value with evidence.
- The agent does **not** auto-edit specification.json on Abandoned — the human applies the recommendation.

The file is then committed.

**Exit 3 — Failed.**

Triggered by `flight-runner` returning `failed`. The agent does *not* try to recover.

The session immediately enters Failed state. The agent's actions:

1. Record the failure in the current iteration (it will be incomplete — no Observed/Verdict/Lessons). Include the subagent's `stage`, `run_id`, `partial_analysis_path`, `error_summary`, and `rig_state` verbatim.
2. Write the Outcome section as **Failed**, including:
   - Iteration number that was active.
   - Reference to the failed run ID (or null if no run was assigned before failure).
   - Stage of failure from the subagent.
   - Note: "Tuning conclusions from this session are not trusted pending postmortem."
3. Tell the user: "Session entered Failed state at <stage>. Rig state: <rig_state>. Run `/postmortem-flight <run_id>` to investigate. After postmortem, append the proximate cause as a final lesson to this session before closing." If `rig_state` is `needs_human`, prepend "Rig needs human attention before any further runs."

The postmortem hand-off is a *workflow step*, not an automated action. The agent surfaces the next step and stops. The user runs postmortem manually, then returns to the session file and adds the proximate cause as a final lesson — this is the link that prevents tuning lessons from getting stranded inside postmortem reports.

## What to avoid

- **Multi-variable changes.** Even when two changes "obviously go together," they go in separate iterations. Cause-and-effect attribution is the session's whole purpose.

- **Declaring victory on a single run.** A KPI that clears the tightened criteria threshold once is not a win. The baseline-variance gate + confirmation runs exist precisely because single-run results can be noise. Distributional claims ("eliminates the slow mode", "every run will pass") are *Provisionally consistent* at best on N=1 — never *Accept*.

- **Silent envelope trade-offs.** If a non-target KPI drops a level — even staying above pass — the trade-off must be named in Lessons. Tuning sessions that quietly burn through the envelope produce configs that meet the latest tightening but are worse overall than where the project started. The envelope check is the mechanism; Lessons-naming is the audit trail.

- **Auto-editing specification.json mid-session or on Abandoned.** The agent edits specification.json *only* at session open, with explicit user confirmation, for the one tightening that defines the session. On Abandoned, the agent recommends a revert or loosening with a specific value — but the human applies it.

- **Approving rubber-stamps.** If the agent's Context doesn't actually justify the proposed change, the human's job is to reject and ask for re-reasoning. Approving thin Context defeats the whole skill.

- **Reasoning across sessions.** This session lives on its own evidence. Patterns across sessions are a different skill (spectator, not yet built). The prior-session note in Starting State is a one-time prior, not material for mid-session reasoning.

- **Tuning recommendations after Failed exit.** Once a session enters Failed state, no further config conclusions are drawn from it until postmortem. The session's iterations may have produced misleading evidence; that's exactly what postmortem will determine.

- **Continuous fixes for discrete symptoms without justification.** A bimodal symptom and a continuous-mechanism hypothesis is a mismatch the Context must address directly, not paper over. (See `context-standard.md` for the bimodal-symptoms guidance.)

- **Reasoning from headline KPIs alone.** The `flight-runner` subagent surfaces headline KPIs for triage, but the full `analysis.md` is the evidence base. Verdict and Lessons that reference only the headline dict are missing half the run.
