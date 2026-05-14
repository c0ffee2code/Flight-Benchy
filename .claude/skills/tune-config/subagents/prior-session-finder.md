# Subagent: prior-session-finder

Scans the `tuning/` directory for prior tuning sessions whose objective matches the current one, and returns a structured summary of the most relevant match (or null). Used by `tune-config` at session open to surface a one-time prior — the parent never sees the full content of other session files.

## When the parent invokes this

Once per session, in step 4 of Session open. The parent does not invoke this mid-session; the no-cross-session-reasoning rule still applies after session open.

## Input

```
{
  "objective_text": "<the natural-language objective the user gave>",
  "objective_kpi": "<the KPI + threshold the agent translated to, e.g. 'T_s ≤ 60s on every run'>",
  "tuning_dir": "tuning/"
}
```

The parent passes both the raw text and the translated KPI. Match against either: same KPI direction (T_s reduction, HoldMAE_s reduction, etc.) counts as a match even if thresholds differ.

## Output

On match:

```
{
  "found": true,
  "session_path": "tuning/2026-04-22-reduce-holdmae.md",
  "date": "2026-04-22",
  "outcome": "Met" | "Abandoned" | "Failed",
  "config_delta": "<short description of how final config differed from starting config, e.g. 'angle_kp 4.0→5.0, iterm_limit 100→200'>",
  "key_lesson": "<single most relevant lesson from that session, ~1-2 sentences, in your own words synthesised from the session's Lessons fields>"
}
```

On no match:

```
{
  "found": false
}
```

If multiple prior sessions match, return the **most recent successful** one. If none succeeded, return the most recent **completed** one (Met or Abandoned, not Failed). Failed sessions are surfaced only if there is no other match — and the parent should be told the prior session failed so it doesn't lean on its lessons.

## How to do the work

1. List all `.md` files in `tuning_dir`. Skip ones that don't match the session-file format (`YYYY-MM-DD-<slug>.md`).
2. For each candidate, read the Objective section and the Outcome section (these are at known positions in the template — top and bottom). Skip the iterations themselves; they're not needed for the synthesis.
3. Score matches:
   - Strong match: same KPI, same direction, same operating regime (e.g. both reducing HoldMAE).
   - Weak match: same KPI different direction, or related KPIs (T_s and T@SP are related; HoldMAE and oscillation Hz are not).
   - No match: different objective entirely.
4. Pick the best match per the precedence rule above (most recent Met > most recent Abandoned > most recent Failed > nothing).
5. For the picked session, read the Starting State to get the starting config and read all Lessons fields across iterations. Synthesise the single most-relevant lesson into `key_lesson` — your own words, not a quote. Compute `config_delta` by diffing final iteration's config against Starting State's config.
6. Return the structured result.

## Guarantees the parent depends on

- `key_lesson` is your synthesis, ~1-2 sentences. Not a verbatim quote, not a multi-paragraph summary. The parent will paste this into a single-paragraph Prior session note.
- `config_delta` lists only the parameters that changed. Empty parameters don't appear.
- You never return a partial result. Either `{found: true, ...}` with all fields populated, or `{found: false}`. If you can't read a candidate file, skip it; don't fail the whole call.
- You do not read or reason about iterations' Context/Hypothesis/Prediction fields. Only Objective, Outcome, Starting State, and Lessons. This bounds your working memory and keeps you fast.

## What you do NOT do

- You do not compare the *current* iteration plan to the prior session — you have no current iteration plan, only the objective. Comparison is the parent's job and the skill forbids it mid-session.
- You do not return more than one match. The parent wants a single prior to anchor against, not a survey.
- You do not editorialise on whether the prior session was "well-run." Just report what it concluded.
