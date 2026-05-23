"""
Specification loader -- reads specification.json from a run folder.

Functions
---------
load_specification(run_dir) -> Specification
    Parse specification.json. Exits immediately if absent or malformed.
"""

import json
import sys
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path


class KpiDirection(StrEnum):
    LOWER_IS_BETTER  = "lower_is_better"
    HIGHER_IS_BETTER = "higher_is_better"


@dataclass
class KpiSpec:
    direction:           KpiDirection
    threshold_pass:      float
    threshold_good:      float
    threshold_excellent: float


@dataclass
class Specification:
    tolerance_deg:   float
    hold_mae_deg:    KpiSpec
    time_to_sp_s:    KpiSpec
    settling_time_s: KpiSpec
    hold_duration_s: KpiSpec
    overshoot_pct:   KpiSpec


def load_specification(run_dir) -> Specification:
    """Load and parse specification.json from run_dir. Exits immediately if absent or malformed."""
    path = Path(run_dir) / "specification.json"
    if not path.exists():
        sys.exit(f"specification.json not found in {run_dir}")
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)

    def _kpi(key):
        try:
            entry = raw["kpis"][key]
            return KpiSpec(
                direction=KpiDirection(entry["direction"]),
                threshold_pass=float(entry["thresholds"]["pass"]),
                threshold_good=float(entry["thresholds"]["good"]),
                threshold_excellent=float(entry["thresholds"]["excellent"]),
            )
        except (KeyError, TypeError, ValueError) as err:
            sys.exit(f"specification.json malformed at kpis.{key}: {err}")

    try:
        tolerance_deg = float(raw["tolerance_deg"])
    except (KeyError, TypeError) as e:
        sys.exit(f"specification.json missing tolerance_deg: {e}")

    return Specification(
        tolerance_deg=tolerance_deg,
        hold_mae_deg=_kpi("hold_mae_deg"),
        time_to_sp_s=_kpi("time_to_sp_s"),
        settling_time_s=_kpi("settling_time_s"),
        hold_duration_s=_kpi("hold_duration_s"),
        overshoot_pct=_kpi("overshoot_pct"),
    )
