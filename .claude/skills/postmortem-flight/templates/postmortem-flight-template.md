# Postmortem — <run_id>

**Date:** <YYYY-MM-DD>
**Failed check:** <smoke check name — e.g. power-cut, loop-meltdown, start-angle, truncated, missing>

## Failure summary

<Raw smoke stdout, verbatim. Do not paraphrase.>

```
<smoke check output>
```

## Investigation chains

<One or two chains. Two are mandatory for power-cut and loop-meltdown.>
<For single-chain cases (start-angle, truncated, missing): include a one-line justification of why a second chain wasn't pursued.>

### Chain 1 — <short label of starting hypothesis>

**Starting hypothesis:** <where this chain begins — see SKILL.md Step 1.3 for domain hints>

#### Why 1: <claim at this level>

- **Claim:** <what is true at this level>
- **Evidence:** <specific telemetry observation — column names, timestamps, values>
- **Falsifier:** <what observation, if present, would refute this claim>

#### Why 2: <next level claim>

- **Claim:** <...>
- **Evidence:** <...>
- **Falsifier:** <...>

<Continue until the chain reaches an actionable terminal node — a config change or a code change — or "requires physical inspection: <what>" when telemetry is exhausted.>
<Hardware/electrical/mechanical/process conclusions are NOT valid terminals. If the chain reaches one, keep asking "is there a config or code change that would detect or compensate for this?">

#### Terminal: <config change | code change | requires physical inspection>

<One of:>
<- "Config change: <field-path>, <current value> → <suggested value or 'requires tune-config session'>">
<- "Code change: <file:function or area>, <description of the change>">
<- "Requires physical inspection: <what to check on the bench>">

### Chain 2 — <short label of starting hypothesis>

<Only for power-cut and loop-meltdown, or when otherwise pursued.>
<Must start from a genuinely different hypothesis than Chain 1 — different telemetry slice, different mechanism, different starting point. Not a rephrasing.>

**Starting hypothesis:** <...>

#### Why 1: <...>

- **Claim:** <...>
- **Evidence:** <...>
- **Falsifier:** <...>

<...continues like Chain 1...>

#### Terminal: <...>

### Single-chain justification

<Only for start-angle, truncated, missing — if only one chain was pursued.>
<One line: "Second chain not pursued because <reason>." "I didn't bother" is not a valid reason.>

## Reconciliation

<Only when there are two chains. Remove this section entirely if there is only one.>

<One of three outcomes:>

**Convergent** — Both chains terminate at the same actionable node. <Restate the shared terminal. Confidence high.>

**Divergent** — Chains terminate at different nodes. <List both. The failure has multiple contributing factors; do not pick a winner.>

**Mixed** — One chain dead-ends in "requires physical inspection," the other terminates actionably. <Identify which is which. The actionable one is the proximate cause; the inspection item moves to systemic observations.>

## Proximate Cause

<This header format is fixed. The line immediately below the header is the machine-greppable summary used by Phase 2 recurrence checks in future postmortems. Keep it descriptive and stable.>

<One-line summary — e.g. "encoder reading flat during active motor commands">

<Paragraph: full explanation with evidence. Reference specific telemetry observations from the chains above. If there are multiple contributing proximate causes (divergent chains), include them both as separate one-line summaries on consecutive lines before the paragraph.>

## Systemic observations

<This section answers: why was this failure possible / likely / undetected, and what change to config, code, or pipeline tooling would harden the project against this class of failure?>

<Concrete suggestions where they exist, prioritised by what the codebase can change:>

<- **New smoke check** — code change to `analyse-flight/scripts/smoke.py`. What signature in telemetry would have caught this earlier?>
<- **New run-flight precondition** — code change to the run-flight skill or its scripts. What state should have been verified before the flight started?>
<- **New config default or validation** — config or code change. Is there a value that should be guarded against, or a sensible default that was missing?>
<- **Code hardening in a specific module** — what file, what function, what change?>
<- **Process change for the operator** — only when no code/config encoding is feasible. The preference is always: can this be encoded?>

<Empty acceptable only if the proximate cause is genuinely a one-off with no class-of-failure implications. Justify the emptiness explicitly: "No systemic observations: <reason>.">

## Requires physical inspection

<Items the operator should check on the bench that telemetry could not resolve.>
<Empty acceptable. If empty, omit the section entirely rather than writing "none.">

- <Inspection item 1: what to check, where, what would resolve the question.>
- <Inspection item 2: ...>

## Recurrence check

<Phase 2 output. Always present, even when empty. The recurrence check runs AFTER the Proximate Cause is locked in — Phase 2 cannot revise Phase 1.>

<One of:>

**No prior postmortems on file.** <First postmortem in the project. Phase 2 ran but had nothing to compare against.>

**No matching prior postmortems found.** <Other postmortems exist but none share this proximate cause signature. List the postmortems scanned for the paper trail: <run_id_1>, <run_id_2>, ...>

**Matching prior postmortems found:** <N matches.>

| Run ID | Date | Prior systemic recommendations | Status |
|---|---|---|---|
| <run_id_1> | <YYYY-MM-DD> | <one-line summary of prior recommendations> | <implemented / not implemented / partially implemented> |
| <run_id_2> | <YYYY-MM-DD> | <...> | <...> |

<If any prior recommendations are marked "not implemented" — call this out prominently. This is the project's accumulated technical debt surfacing. Example phrasing: "Recurring failure: this is the 3rd encoder-flat postmortem in 6 weeks. Prior systemic recommendation from <run_id> (add a stale-encoder smoke check) has not been implemented. Recommend escalating.">
