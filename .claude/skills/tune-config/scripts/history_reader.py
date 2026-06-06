"""
history_reader.py — history reader for the tune-config skill.

Reads the last N run folders from test_runs/flights/, extracts config and KPIs
from each run's config.json / analysis/verdict.json / analysis/diagnose.json /
postmortem.md, then emits a structured Markdown summary to stdout.

The tune-config agent uses this output to:
  - Identify which runs share the current config (baseline candidates).
  - Compute baseline KPI statistics (mean, std dev, range).
  - Detect failed or unanalysed runs in the window.

Output format: Markdown with fenced sections:
  Recent runs | Current configuration | Baseline analysis | Failed runs | Summary flags

Run from project root:
    python .claude/skills/tune-config/scripts/history_reader.py [--n N] [--flights-dir PATH]
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

# Ensure unicode output works on Windows terminals with non-UTF-8 default encoding
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')


# ---------------------------------------------------------------------------
# Config extraction
# ---------------------------------------------------------------------------

def _get(d, *keys, default=None):
    """Navigate a nested dict safely; return default on any missing key."""
    for k in keys:
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def extract_config(cfg):
    """
    Extract all tuning-relevant fields from a parsed config.json dict.
    Missing fields are represented as None; absent feedforward block is flagged
    with ff_absent=True.
    """
    ap = _get(cfg, 'vehicle', 'loops', 'angle', 'pid') or {}
    rp = _get(cfg, 'vehicle', 'loops', 'rate', 'pid') or {}
    mo = _get(cfg, 'vehicle', 'motor') or {}
    ff = _get(cfg, 'vehicle', 'feedforward')
    sp = _get(cfg, 'session', 'setpoint') or {}

    return {
        'angle_kp':    ap.get('kp'),
        'angle_ki':    ap.get('ki'),
        'angle_kd':    ap.get('kd'),
        'angle_iterm': ap.get('iterm_limit'),
        'rate_kp':     rp.get('kp'),
        'rate_ki':     rp.get('ki'),
        'rate_kd':     rp.get('kd'),
        'rate_iterm':  rp.get('iterm_limit'),
        'base':        mo.get('base_throttle'),
        'lead_ms':     ff.get('lead_ms') if isinstance(ff, dict) else None,
        'ff_absent':   ff is None,
        'sp_roll':     sp.get('roll_deg'),
        'sp_pitch':    sp.get('pitch_deg'),
        'sp_yaw':      sp.get('yaw_deg'),
    }


def configs_match(a, b):
    """
    True if two extracted config dicts share the same tuning dimensions.
    Feedforward: both absent → match; one absent → mismatch; both present →
    compare lead_ms.
    """
    dims = [
        'angle_kp', 'angle_ki', 'angle_kd', 'angle_iterm',
        'rate_kp', 'rate_ki', 'rate_kd', 'rate_iterm',
        'base', 'sp_roll', 'sp_pitch', 'sp_yaw',
    ]
    for d in dims:
        if a.get(d) != b.get(d):
            return False
    if a['ff_absent'] != b['ff_absent']:
        return False
    if not a['ff_absent'] and a.get('lead_ms') != b.get('lead_ms'):
        return False
    return True


# ---------------------------------------------------------------------------
# analysis JSON parsing
# ---------------------------------------------------------------------------

def _safe_get(d, *keys):
    """Navigate a nested dict safely; return None on any missing key."""
    for k in keys:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def parse_analysis(verdict_path, diagnose_path, run_id):
    """
    Parse KPIs from analysis/verdict.json and analysis/diagnose.json.
    Returns a dict with keys:
      reached (bool), t_to_s (float), holdmae (float),
      t_at_s (float), osc_hz (float or None), duration_s (float)
    Any missing field is None. Returns None if neither file is readable.
    """
    verdict = None
    diagnose = None

    try:
        with open(verdict_path, encoding='utf-8') as f:
            verdict = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print(f"WARNING: {run_id}: cannot read verdict.json: {e}", file=sys.stderr)

    try:
        with open(diagnose_path, encoding='utf-8') as f:
            diagnose = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print(f"WARNING: {run_id}: cannot read diagnose.json: {e}", file=sys.stderr)

    if verdict is None and diagnose is None:
        return None

    kpis = {
        'reached':    _safe_get(verdict,  'reached'),
        't_to_s':     _safe_get(verdict,  'time_to_sp_s'),
        'holdmae':    _safe_get(verdict,  'hold_mae_deg'),
        't_at_s':     _safe_get(verdict,  'hold_duration_s'),
        'osc_hz':     _safe_get(diagnose, 'hold_tracking', 'fft_freq_hz'),
        'duration_s': _safe_get(diagnose, 'sample_rate', 'duration_s'),
    }

    if not any(kpis.get(k) is not None for k in ('reached', 't_to_s', 'holdmae')):
        print(f"WARNING: {run_id}: analysis JSON present but no KPIs extracted", file=sys.stderr)
        return None

    return kpis


# ---------------------------------------------------------------------------
# postmortem.md parsing
# ---------------------------------------------------------------------------

def parse_postmortem(path, run_id):
    """
    Extract the one-line proximate cause from postmortem.md.
    Returns the cause string, or None if the header is absent.
    """
    try:
        text = path.read_text(encoding='utf-8')
    except OSError as e:
        print(f"WARNING: {run_id}: cannot read postmortem.md: {e}", file=sys.stderr)
        return None

    # First non-blank line after the ## Proximate Cause header
    lines = text.splitlines()
    in_section = False
    for line in lines:
        if re.match(r'^\s*##\s+Proximate Cause', line):
            in_section = True
            continue
        if in_section:
            stripped = line.strip()
            if stripped:
                return stripped

    print(f"WARNING: {run_id}: postmortem.md: '## Proximate Cause' not found", file=sys.stderr)
    return None


# ---------------------------------------------------------------------------
# Run loading
# ---------------------------------------------------------------------------

def load_run(run_dir):
    """
    Load one run folder. Returns a dict:
      id, date, status ('analysed'|'failed'|'unanalysed'), config, kpis, cause
    Returns None if config.json is missing or unreadable (logged to stderr).
    """
    run_id = run_dir.name
    cfg_path = run_dir / 'config.json'

    if not cfg_path.exists():
        print(f"WARNING: {run_id}: config.json missing — skipping", file=sys.stderr)
        return None

    try:
        with open(cfg_path) as f:
            cfg_raw = json.load(f)
        config = extract_config(cfg_raw)
    except (json.JSONDecodeError, OSError) as e:
        print(f"WARNING: {run_id}: config.json unreadable ({e}) — skipping", file=sys.stderr)
        return None

    verdict_path  = run_dir / 'analysis' / 'verdict.json'
    diagnose_path = run_dir / 'analysis' / 'diagnose.json'
    postmortem_path = run_dir / 'postmortem.md'

    if verdict_path.exists():
        kpis = parse_analysis(verdict_path, diagnose_path, run_id)
        status = 'analysed' if kpis is not None else 'unanalysed'
        cause = None
    elif postmortem_path.exists():
        kpis = None
        cause = parse_postmortem(postmortem_path, run_id)
        status = 'failed'
    else:
        kpis = None
        cause = None
        status = 'unanalysed'

    return {
        'id':     run_id,
        'date':   run_id[:10],
        'status': status,
        'config': config,
        'kpis':   kpis,
        'cause':  cause,
    }


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def _mean(values):
    return sum(values) / len(values)


def _stdev(values):
    """Sample standard deviation (N-1 denominator). None if len < 2."""
    if len(values) < 2:
        return None
    m = _mean(values)
    return math.sqrt(sum((x - m) ** 2 for x in values) / (len(values) - 1))


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

DASH = '—'


def _fv(v, fmt):
    """Format numeric value v with fmt, or return DASH if None."""
    return f"{v:{fmt}}" if v is not None else DASH


def _fc(v):
    """Format a config value as a string; 'absent' if None."""
    if v is None:
        return 'absent'
    return str(v)


def _kpi_cell(kpis, key, fmt):
    """Retrieve kpis[key] and format it, or DASH."""
    if kpis is None:
        return DASH
    return _fv(kpis.get(key), fmt)


def _reached_cell(kpis):
    if kpis is None:
        return DASH
    v = kpis.get('reached')
    if v is None:
        return DASH
    return 'yes' if v else 'no'


def _stat_row(label, values, fmt_val, fmt_range, low_n_note=True):
    """
    Build a stats table row from a list of numeric values.
    Applies sample-size notes per spec: N=1 → dash for std, N=2 → low-confidence note.
    """
    n = len(values)
    if n == 0:
        return f"| {label} | {DASH} | {DASH} | {DASH} | 0 |"
    m  = _mean(values)
    s  = _stdev(values)
    lo = min(values)
    hi = max(values)
    m_s = f"{m:{fmt_val}}"
    if s is None:
        s_s = f"{DASH} *(single sample, no variance estimate)*"
    elif n == 2 and low_n_note:
        s_s = f"{s:{fmt_val}} *(low-confidence variance estimate, N=2)*"
    else:
        s_s = f"{s:{fmt_val}}"
    r_s = f"{lo:{fmt_range}}–{hi:{fmt_range}}"
    return f"| {label} | {m_s} | {s_s} | {r_s} | {n} |"


# ---------------------------------------------------------------------------
# Output generation
# ---------------------------------------------------------------------------

def generate_output(runs, n_requested, flights_dir):
    lines = []
    out = lines.append

    actual = len(runs)
    out("# History Reader Output")
    out("")
    if actual < n_requested:
        out(f"Run list: all {actual} runs from {flights_dir} "
            f"(fewer than requested {n_requested}), sorted oldest-first.")
    else:
        out(f"Run list: last {actual} runs from {flights_dir}, sorted oldest-first.")
    out("")

    # ------------------------------------------------------------------ #
    # Recent runs table                                                    #
    # ------------------------------------------------------------------ #
    out("## Recent runs")
    out("")
    headers = [
        'Run ID', 'Date', 'Status',
        'angle_kp', 'angle_ki', 'angle_kd', 'angle_iterm',
        'rate_kp', 'rate_ki', 'rate_kd', 'rate_iterm',
        'base',
        'Reached', 'T→0', 'HoldMAE', 'T@0', 'Osc Hz',
        'Notes',
    ]
    out('| ' + ' | '.join(headers) + ' |')
    out('|' + '|'.join(['---'] * len(headers)) + '|')

    for r in runs:
        c = r['config']
        k = r['kpis']
        if r['status'] == 'failed' and r['cause']:
            notes = f"proximate: {r['cause']}"
        elif r['status'] == 'unanalysed':
            notes = 'not yet analysed'
        else:
            notes = DASH

        row = [
            r['id'], r['date'], r['status'],
            _fc(c.get('angle_kp')), _fc(c.get('angle_ki')),
            _fc(c.get('angle_kd')), _fc(c.get('angle_iterm')),
            _fc(c.get('rate_kp')),  _fc(c.get('rate_ki')),
            _fc(c.get('rate_kd')),  _fc(c.get('rate_iterm')),
            _fc(c.get('base')),
            _reached_cell(k),
            _kpi_cell(k, 't_to_s',  '.2f'),
            _kpi_cell(k, 'holdmae', '.2f'),
            _kpi_cell(k, 't_at_s',  '.1f'),
            _kpi_cell(k, 'osc_hz',  '.3f'),
            notes,
        ]
        out('| ' + ' | '.join(row) + ' |')

    out("")

    # ------------------------------------------------------------------ #
    # Current configuration                                                #
    # ------------------------------------------------------------------ #
    out("## Current configuration")
    out("")

    if not runs:
        out("No runs — current configuration cannot be determined.")
        out("")
    else:
        cur = runs[-1]   # most recent = last in oldest-first list
        c = cur['config']
        ff_str = 'absent' if c['ff_absent'] else _fc(c.get('lead_ms'))
        out(f"- **angle_pid:** kp={_fc(c.get('angle_kp'))}, ki={_fc(c.get('angle_ki'))}, "
            f"kd={_fc(c.get('angle_kd'))}, iterm_limit={_fc(c.get('angle_iterm'))}")
        out(f"- **rate_pid:** kp={_fc(c.get('rate_kp'))}, ki={_fc(c.get('rate_ki'))}, "
            f"kd={_fc(c.get('rate_kd'))}, iterm_limit={_fc(c.get('rate_iterm'))}")
        out(f"- **motor.base:** {_fc(c.get('base'))}")
        out(f"- **feedforward.lead_ms:** {ff_str}")
        out(f"- **setpoint:** roll={_fc(c.get('sp_roll'))}, "
            f"pitch={_fc(c.get('sp_pitch'))}, yaw={_fc(c.get('sp_yaw'))}")
        out("")
        out(f"Current config is the configuration of the most recent run ({cur['id']}).")
        out("")

        # ------------------------------------------------------------------ #
        # Baseline analysis                                                    #
        # ------------------------------------------------------------------ #
        out("## Baseline analysis")
        out("")

        cur_cfg = cur['config']
        baseline_runs     = [r for r in runs if configs_match(r['config'], cur_cfg)]
        analysed_baseline = [r for r in baseline_runs
                             if r['status'] == 'analysed' and r['kpis'] is not None]
        n_baseline = len(baseline_runs)
        n_analysed = len(analysed_baseline)

        out(f"Baseline runs (runs in the recent list matching current config): {n_baseline}")
        if baseline_runs:
            out(f"Run IDs: {', '.join(r['id'] for r in baseline_runs)}")
        out("")

        if n_analysed == 0:
            out("No baseline runs at current config.")
            out("")
            out("Baseline sufficiency: **NONE** (0 analysed runs at current config).")
        else:
            out("Baseline KPIs (only analysed runs counted; failed/unanalysed runs excluded):")
            out("")

            def _kpi_vals(key):
                return [r['kpis'][key] for r in analysed_baseline
                        if r['kpis'].get(key) is not None]

            # Reached rate (fraction, not mean)
            reached_count = sum(1 for r in analysed_baseline if r['kpis'].get('reached'))
            reached_frac  = f"{reached_count}/{n_analysed} ({100 * reached_count // n_analysed}%)"

            out("| KPI | Mean | Std Dev | Range | Sample size |")
            out("|---|---|---|---|---|")
            out(f"| Reached rate | {reached_frac} | — | — | {n_analysed} |")
            out(_stat_row("T→0 (s)",        _kpi_vals('t_to_s'),  '.2f', '.2f'))
            out(_stat_row("HoldMAE (°)",    _kpi_vals('holdmae'), '.2f', '.2f'))
            out(_stat_row("T@0 (s)",        _kpi_vals('t_at_s'),  '.1f', '.1f'))
            out(_stat_row("Oscillation (Hz)", _kpi_vals('osc_hz'), '.3f', '.3f'))
            out("")

            if n_analysed >= 3:
                suff = "**OK** (≥3 analysed runs at current config)."
            elif n_analysed == 1:
                suff = "**INSUFFICIENT** (1 analysed run; variance estimates unreliable)."
            else:
                suff = f"**INSUFFICIENT** ({n_analysed} analysed runs; variance estimates unreliable)."
            out(f"Baseline sufficiency: {suff}")

        out("")

    # ------------------------------------------------------------------ #
    # Failed runs                                                          #
    # ------------------------------------------------------------------ #
    out("## Failed runs in window")
    out("")

    failed = [r for r in runs if r['status'] == 'failed']
    if not failed:
        out("No failed runs in the last window.")
    else:
        out(f"{len(failed)} failed run(s) in the last {actual}:")
        out("")
        for r in failed:
            cause = r['cause'] or 'no proximate cause extracted'
            out(f"- {r['id']} — {cause}")
    out("")

    # ------------------------------------------------------------------ #
    # Summary flags                                                        #
    # ------------------------------------------------------------------ #
    out("## Summary flags")
    out("")

    if not runs:
        bsuff_str = "N/A"
    else:
        cur_cfg = runs[-1]['config']
        ab = [r for r in runs
              if configs_match(r['config'], cur_cfg)
              and r['status'] == 'analysed'
              and r['kpis'] is not None]
        n = len(ab)
        if n >= 3:
            bsuff_str = "yes"
        elif n == 0:
            bsuff_str = "no (NONE — 0 analysed runs at current config)"
        else:
            bsuff_str = f"no (INSUFFICIENT — {n} analysed run(s) at current config)"

    out(f"- Baseline sufficient: {bsuff_str}")

    unanalysed = [r for r in runs if r['status'] == 'unanalysed']
    if not failed:
        out("- Failed runs in window: no")
    else:
        ids = ', '.join(r['id'] for r in failed)
        out(f"- Failed runs in window: yes ({len(failed)}) — {ids}")

    if not unanalysed:
        out("- Unanalysed runs in window: no")
    else:
        ids = ', '.join(r['id'] for r in unanalysed)
        out(f"- Unanalysed runs in window: yes ({len(unanalysed)}) — {ids}")

    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Emit a structured Markdown summary of recent Flight Benchy runs."
    )
    parser.add_argument('--n', type=int, default=10, metavar='N',
                        help="Number of recent runs to include (default: 10)")
    parser.add_argument('--flights-dir', default='test_runs/flights',
                        help="Root flights directory (default: test_runs/flights)")
    args = parser.parse_args()

    flights_dir = Path(args.flights_dir)

    if not flights_dir.exists():
        print("# History Reader Output\n")
        print(f"No runs in `{flights_dir}/` — directory does not exist.")
        return

    all_dirs = sorted(
        [d for d in flights_dir.iterdir() if d.is_dir()],
        key=lambda d: d.name,
    )

    if not all_dirs:
        print("# History Reader Output\n")
        print(f"No runs in `{flights_dir}/`.")
        return

    selected = all_dirs[-args.n:]   # last N, oldest-first

    runs = []
    seen = set()
    for d in selected:
        if d.name in seen:
            print(f"WARNING: duplicate folder name {d.name} — using first occurrence",
                  file=sys.stderr)
            continue
        seen.add(d.name)
        r = load_run(d)
        if r is not None:
            runs.append(r)

    if not runs:
        print("# History Reader Output\n")
        print(f"No valid runs in `{flights_dir}/` — all folders missing config.json.")
        return

    print(generate_output(runs, args.n, flights_dir))


if __name__ == '__main__':
    main()