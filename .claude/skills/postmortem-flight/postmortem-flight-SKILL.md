---
name: postmortem-flight
description: Investigate a Flight Benchy run that failed smoke checks. Performs structured root-cause analysis using the 5-whys method with mandatory falsifiers, then scans prior postmortems for recurrence. The investigation's why-chains terminate at config changes or code changes only — items requiring physical inspection (mechanical, electrical, hardware) are tagged and surfaced separately but are not chain terminals. Use when a run failed smoke (run-flight or analyse-flight stopped with a smoke check name and detail) and you want to understand why — not just verify it failed. Triggers on phrases like "postmortem", "investigate this failure", "5 whys on this run", "why did <run_id> fail", "dig into <run_id>". Does NOT cover successful runs (use analyse-flight), tuning recommendations (the skill identifies that tuning is needed but does not propose values — that is tune-config's job), pattern detection across successful runs (spectator territory), or modifying any files outside the run folder (read-only on project sources, writes only postmortem.md to the run folder). The skill produces postmortem.md in the run folder and is the only producer of that file.
---

## Arguments

```
<flight_id>
```

`<flight_id>` — required. The run folder name under `test_runs/flights/`. The run must have failed smoke; if it passed, refuse and point the user at `analyse-flight`.

# Postmortem Flight

Root-cause analysis for a failed Flight Benchy run. Two phases, in order, never interleaved:

1. **Investigation** — why did *this* run fail, on its own evidence.
2. **Recurrence check** — does this proximate cause appear in prior postmortems.

The phase boundary is hard. Phase 2 cannot revise Phase 1 conclusions; it can only contextualise them. Reading prior postmortems before the proximate cause is locked in biases the investigation toward the most-recent failure mode and prevents finding novel causes. This ordering is the whole point of the skill.

## Why this skill exists

A failed smoke check tells you *what* tripped, not *why*. Stopping at "you forgot reset-position" or "the rate loop went unstable" hides the more interesting question: what in our config or our code made this failure possible, likely, or undetected — and what changes would make it less likely next time?

A good postmortem produces both a proximate cause (what telemetry shows) and systemic observations (what change to config, code, or pipeline tooling would harden the project against this class of failure). Lookups produce neither. Investigation does.

The scope is deliberately narrow: this skill investigates the project's source-controlled artifacts — `config.json`, `src/`, the pipeline scripts. Failures rooted in hardware, electrical, or mechanical state are surfaced for the operator but are not where the investigation tries to *conclude*. The why-chains dig until they hit something the codebase can change, or until telemetry is exhausted and physical inspection is required.

## Hard rules — apply throughout investigation and report

- **Every claim names a telemetry observation that supports it.** Bare assertions are not allowed. "The rate loop went unstable" is not a claim; "gyro_x oscillated between ±400 deg/s with period ~30ms starting at T=4.2s" is a claim.
- **Unsupported claims are tagged, not banned.** If the most likely explanation isn't visible in telemetry — a loose connector, a weakened magnet, a wire chafing — the claim is recorded explicitly as "requires physical inspection: <what to check>" and the chain continues on what *is* visible. The investigation does not stall on unobservables; it routes around them.
- **No premature closure.** A chain does not terminate at the first plausible cause. It terminates at an *actionable* node — and actionable here means specifically: a **config change** (a value in `config.json`) or a **code change** (a modification to project source, including pipeline scripts like `smoke.py` or `run-flight`). Hardware changes, electrical changes, mechanical changes, and operator-process changes are NOT valid chain terminals — they go in "Requires physical inspection" or in systemic observations. "Requires physical inspection: <what>" is also acceptable as a terminal node when telemetry is genuinely exhausted.
- **No comparison to other runs during Phase 1.** None. Not "this looks like the run last week." Phase 2 exists for that. **[CHECK — strict version. I think it's right but flagging it.]**
- **The four hard rules from analyse-flight still apply** to language about causes outside telemetry: no battery/power speculation, no mechanical speculation, no IMU speculation, no run-to-run attribution. The "requires physical inspection" tag is how we acknowledge those domains exist without speculating about them.

## Phase 1 — Investigation

### Step 1.1 — Identify the failure

Re-run smoke to get the failed check name and detail (the user's invocation may have been hours after the failure; don't trust memory).

```
python .claude/skills/analyse-flight/scripts/smoke.py test_runs/flights/<flight_id>
```

Record the check name and the exact stdout in the report's "Failure summary" section.

### Step 1.2 — Choose chain structure

| Failed check | Chains required |
|---|---|
| `power-cut` | **Two chains, mandatory.** |
| `loop-meltdown` | **Two chains, mandatory.** |
| `start-angle` | One chain acceptable, with a one-line justification of why a second wasn't pursued. |
| `truncated` | One chain acceptable, with justification. |
| `missing` | One chain acceptable, with justification. |

The mandatory two-chain rule on `power-cut` and `loop-meltdown` exists because both failure modes have multiple plausible mechanisms and single-chain investigation reliably picks the most familiar one. The other three have narrower causal surfaces; one chain is usually honest. The justification line on the single-chain cases is *itself* the discipline — "I couldn't think of a second chain because X" is a real answer; "I didn't bother" isn't.

### Step 1.3 — Starting points (domain hints)

These are seed hypotheses for the *first* chain only. They are not answers; they are places to start digging. The second chain must come from somewhere else — different starting hypothesis, different telemetry slice, different mechanism.

- **`power-cut`** — first chain seed: the controller commanded a thrust profile the supply couldn't deliver (full-throttle slams, rapid M1/M2 swings, sustained high baseline). Look at M1, M2 over time; look for fast transients and sustained high values immediately preceding the failure point.
- **`loop-meltdown`** — first chain seed: rate-loop instability from over-aggressive gains or sign error. Look at gyro_x oscillation amplitude and period; look at rate_sp vs gyro_x phase relationship; look at sign correlation between angle error and rate setpoint.
- **`start-angle`** — first chain seed: reset-position step skipped or failed. Look at first encoder reading and at recent run history to see if the bench was left in an off-restrictor state.
- **`truncated`** — first chain seed: write interrupted before flush. Look at last T_MS, last row contents, file size.
- **`missing`** — first chain seed: SD session never opened. Look at whether the folder was created; look at config.json presence.

For the second chain on `power-cut` / `loop-meltdown`: pursue something genuinely different. Examples of *different* (non-exhaustive): a sensor producing bad data that the controller responded to correctly given the bad input; a code path entered that wasn't expected (mode change, error handler, init sequence); a timing issue (loop period drift, missed iterations). The point isn't to enumerate possibilities — it's to ensure the second chain doesn't just rephrase the first.

### Step 1.4 — Run the chains

Each "why" is a node with three fields:

- **Claim** — what is true at this level.
- **Evidence** — the specific telemetry observation supporting the claim. Column names, timestamps, values.
- **Falsifier** — what observation, if present, would refute this claim. If you can't write a falsifier, the claim isn't testable and the chain doesn't advance.

Advance to the next "why" only when the current node has all three. Continue until reaching an actionable terminal node — a **config change** (value in `config.json`) or a **code change** (modification to project source or pipeline scripts) — or "requires physical inspection: <what>" when telemetry is exhausted. Hardware/electrical/mechanical/process conclusions are not valid chain terminals; they belong in "Requires physical inspection" or in systemic observations.

**[CHECK — the falsifier requirement is the load-bearing rule of the whole skill. It's also the one most likely to feel pedantic in practice. I want to flag that I'm proposing it as a hard rule, not a soft one. If it produces friction in the first real postmortem we can soften it then.]**

### Step 1.5 — Reconcile chains (when there are two)

After both chains terminate independently, compare endpoints:

- **Convergent** — both chains terminate at the same actionable node. Confidence high; record as the proximate cause.
- **Divergent** — chains terminate at different nodes. The failure has multiple contributing factors. Record both as contributing proximate causes; do *not* pick a winner.
- **One chain dead-ends in "requires physical inspection," the other terminates actionably** — the actionable one is the proximate cause; the inspection item is a systemic observation.

### Step 1.6 — Lock in the proximate cause

Write the **Proximate Cause** section using this exact header format (the recurrence check in Phase 2 greps on this):

```
## Proximate Cause

<one-line summary, machine-greppable>

<paragraph: full explanation with evidence>
```

The one-line summary is what future postmortems will match against. Keep it descriptive and stable: "encoder reading flat during active motor commands" is good; "weird thing happened with the encoder" isn't.

## Phase 2 — Recurrence check

Only after the Proximate Cause is locked in. If you find yourself wanting to revise Phase 1 based on Phase 2 findings, stop and ask the user — that's a sign the recurrence biased the investigation, which is exactly what this ordering exists to prevent.

### Step 2.1 — Scan prior postmortems

```
find test_runs/flights -name postmortem.md -not -path "*<flight_id>*"
```

For each prior postmortem, extract the one-line summary under `## Proximate Cause`. Match against the current run's summary using semantic similarity (not string equality — "encoder reading flat" and "encoder went flat during run" are the same finding).

### Step 2.2 — Report findings

Even if there are no prior postmortems, write the section with explicit "no prior postmortems on file" — silence is worse than empty output, because the operator needs to know the check ran.

If matches exist, list them with date, run ID, and prior systemic recommendations. **Especially** flag prior recommendations that were not implemented; those are the project's accumulated technical debt surfacing.

## Output: postmortem.md

Read the template at `.claude/skills/postmortem-flight/templates/postmortem.md` and fill it. Save to `test_runs/flights/<flight_id>/postmortem.md`. Tell the user where it was saved.

Sections, in order:

1. **Failure summary** — failed smoke check name + raw smoke stdout, verbatim.
2. **Investigation chains** — one or two chains, each as a nested structure with claim/evidence/falsifier per node, terminating at an actionable node or inspection tag.
3. **Reconciliation** (only when two chains) — convergent / divergent / mixed, with reasoning.
4. **Proximate Cause** — fixed-format header, machine-greppable one-liner, paragraph explanation.
5. **Systemic observations** — the section that asks not "what went wrong" but "why was the system shaped such that this could go wrong." Questions of the form: "why was this failure possible / likely / undetected?" Concrete suggestions where they exist, prioritised by what the codebase can change: new smoke check (code change to `smoke.py`), new `run-flight` precondition (code change), new config default or validation (config or code change), or code hardening in a specific module. Process changes affecting the operator (new manual step, new convention) also belong here when no code/config encoding is feasible — but the preference is always: can this be encoded? This section is the long-term value of the skill — proximate causes get fixed once; systemic observations harden the project against whole classes of failure.
6. **Requires physical inspection** — items the operator should check on the bench that telemetry could not resolve.
7. **Recurrence check** — Phase 2 output. Always present, even when empty.

## What to avoid

- **Naming the human as the cause.** "Operator forgot reset-position" is not a root cause; it's a symptom of a system where forgetting reset-position is possible. The systemic question is "why is reset-position skippable, and what would catch it?"
- **Stopping at the first plausible answer.** The chain only terminates at an *actionable* config or code node, or at "requires physical inspection." "Mechanical issue" isn't actionable; "requires physical inspection: check encoder magnet adhesion" is.
- **Treating hardware/electrical/mechanical conclusions as chain terminals.** They are valid findings but they are not where the chain *concludes*. If the chain reaches "the magnet is weak," continue the why-chain: is there a config or code change that would have detected or compensated for a weak magnet? If yes, that's the terminal. If no, the magnet weakness goes in "Requires physical inspection" and the chain ends there with telemetry exhausted.
- **Comparison to other runs in Phase 1.** Phase 2 exists. Use it.
- **Tuning recommendations with specific values.** Out of scope. The systemic section can say "current rate gains may be too aggressive — recommend tune-config session" but does not propose specific gain values. Proposing values is `tune-config`'s job.
- **Modifying files outside the run folder.** This skill is read-only on project sources and writes only `postmortem.md` to the run folder. Any suggested config or code change goes in the report as a recommendation, never as a direct edit.

---

## Open questions for review

Three points I want your read on before writing the `investigate.py` helper and the template:

1. **The falsifier requirement (Step 1.4).** Hard rule or soft? Hardest constraint in the skill, does the most work against premature closure, also most likely to feel pedantic the first time you live through it.

2. **No comparison to other runs in Phase 1, full stop.** Strict version above. The cost: if you remember a prior postmortem mid-investigation, the skill tells you to ignore that memory until Phase 2. Right discipline, awkward first time.

3. **Systemic section's stance on operator error.** The framing above treats "operator forgot X" as a symptom of a system that allows X to be forgotten, not as the root cause itself. Confirm this matches the voice you want the skill to speak in.
