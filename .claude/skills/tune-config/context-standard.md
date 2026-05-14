# The Context standard

Load this file when drafting any iteration's Context field. It is the reference for *how to reason*, not part of the session lifecycle procedure. The parent agent does not need this file in context for post-run review, exit-condition checking, or session-open routines.

Context is the load-bearing field of every iteration. Vague Context produces unverifiable reasoning and rubber-stamp approvals. The standard is: **the human can follow the reasoning and verify it against their own mental model.**

## Five principles

Five principles guide the Context field:

1. **Surface alternative readings of the data, not just the first one that comes to mind.** Forward-reasoning analogue of postmortem's two-chains rule. Honest investigation considers more than one story.

2. **Do not ignore signals in the data, even when they don't fit the current hypothesis.** Either incorporate the signal as evidence, or explicitly dismiss it with reasoning. Silence is failure.

3. **Prefer diagnostic experiments over extrapolation.** When the data supports multiple readings, the next change should *distinguish* them. "More of what worked last time" is fine when you understand why it works; it's lazy when you don't.

4. **Pre-name what each possible outcome would mean.** Not just "what would prove me wrong" but "what would each possible result tell me." This makes the post-run Verdict mechanical instead of narrative.

5. **Steelman the change failing.** Before proposing, write one sentence describing a plausible mechanism by which the proposed change makes the target KPI worse, or makes it better for the wrong reason. If you cannot write such a sentence, you do not yet understand the change well enough to propose it. This rule exists because alternative-readings-of-the-data (principle 1) does not catch sign-flip mistakes in the *proposed mechanism* — only adversarial review of the change itself does. Symmetric output shaping that removes both drive *and* braking authority near setpoint is the canonical example: reading the data correctly doesn't help if the mechanism's symmetry isn't examined.

## Bimodal symptoms

When the symptom distribution is bimodal or discrete — runs cluster into two T_s groups with nothing between, overshoot is either ~14% or ~19% with no intermediate values, runs either fully succeed or fully fail — the search for causes should be **biased toward bimodal or discrete mechanisms** rather than continuous ones:

- Event-triggered behavior (scheduler missed a tick, sensor glitch, sample-time drift).
- Threshold crossings (integrator hitting a clamp, motor saturating, state-dependent gain switch).
- Initial-condition dependence (where the lever started, prior actuator state, thermal).
- Race conditions in the control loop.

Continuous mechanisms — gain too low, limit too loose, time constant too short — produce *continuous* degradation, not bimodal. A gain change applied to a bimodal-cause symptom may *appear* to help when the underlying mode-trigger simply didn't fire during the test runs; the next session reopens with the same symptom. Iterations that propose continuous fixes for clearly bimodal symptoms should name in their Context *why* the continuous mechanism is expected to break the discrete pattern. If no such argument is available, the iteration is probably the wrong shape and the session may need to recommend a non-config investigation (see SKILL.md Exit 2b).

## Example: weak Context (what we don't want)

> The last three iterations increased angle_kp from 4.0 to 5.0 and HoldMAE_s improved from 6.2° to 5.0°. The trend is positive but appears to be slowing. I propose increasing angle_kp further to 5.5 to continue the improvement.

Why this is weak:

- Data summary present but interpretation is shallow.
- Justification is "more of the same" — extrapolation, not understanding.
- Ignores motor asymmetry visible in the data — either it's relevant and was missed, or it's irrelevant and silence still fails.
- Predicts nothing the next run could disprove. "Continue the improvement" is vague.
- No steelman of how 5.5 could be worse than 5.0.

This is Context that *looks* reasonable on first read and is doing zero reasoning work.

## Example: strong Context

> Iterations 1–3 increased angle_kp from 4.0 → 4.5 → 5.0 and HoldMAE_s improved 6.2° → 5.4° → 5.1° → 5.0°. The improvement is decelerating sharply — the first 0.5 step gave 0.8° improvement, the last gave 0.1°. Two readings of this:
>
> (a) We're approaching the gain ceiling for this loop — higher kp would start exciting oscillation, and the diminishing return is the warning sign before that happens. Oscillation freq is still 0.05 Hz across all three iterations, which is consistent with "approaching but not yet at" instability.
>
> (b) angle_kp isn't the binding constraint anymore. Something else is now setting the floor. The persistent motor asymmetry (M2 ~30 units above M1 during hold) suggests the angle PID is fighting a steady disturbance, and the I-term should be absorbing that — but iterm_limit=100 caps the I contribution at 5 deg/s, which may not be enough.
>
> Reading (b) is more interesting because it points at a different mechanism, and the diagnostic is cheap: if the I-term is consistently pegged at its limit during hold, raising iterm_limit should help; if it isn't pegged, raising it does nothing and we know the asymmetry is from somewhere else.
>
> Steelman of how raising iterm_limit could be worse: if the asymmetry is from a transient disturbance rather than a steady one, a higher I-term cap allows more windup during the transient, slowing recovery. We'd see this as worse T_s or larger excursion on initial reach. The Falsifier covers this.
>
> I propose: hold angle_kp at 5.0, raise iterm_limit from 100 to 200, single variable. Prediction: if (b) is correct, HoldMAE_s drops below 4.5° and M1/M2 asymmetry narrows. If (a) is correct (kp ceiling), HoldMAE_s moves by less than 0.1° and asymmetry persists. Either outcome is informative.

Why this is strong:

- Two readings, evaluated against each other.
- The second signal in the data (motor asymmetry) is named and used as evidence.
- The proposed experiment is chosen because the two readings predict different outcomes — diagnostic, not extrapolative.
- Both outcomes are pre-named. The post-run Verdict becomes mechanical: compare observed to the two predictions, see which one fits.
- The steelman names a specific mechanism by which the change could make things worse, and connects it to the Falsifier.
