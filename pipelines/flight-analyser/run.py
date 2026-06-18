import json
import subprocess
import sys
from pathlib import Path

SCRIPTS = Path(__file__).parent / "scripts"
STAGES = ["gate", "plots", "invariants", "verdict", "diagnose", "report"]


def _error_summary(r):
    combined = (r.stdout + "\n" + r.stderr).strip()
    lines = [ln for ln in combined.splitlines() if ln.strip()]
    return lines[-1] if lines else f"exit code {r.returncode}"


def run_analysis(run_id):
    run_folder = f"test_runs/flights/{run_id}"

    for stage in STAGES:
        r = subprocess.run(
            ["python", str(SCRIPTS / f"{stage}.py"), run_folder],
            capture_output=True,
            text=True,
        )
        sys.stderr.write(r.stdout)
        sys.stderr.write(r.stderr)
        if r.returncode != 0:
            summary_path = Path(run_folder) / "analysis" / "summary.md"
            return {
                "status": "failed",
                "stage": stage,
                "run_id": run_id,
                "partial_summary_path": str(summary_path) if summary_path.exists() else None,
                "error_summary": _error_summary(r),
                "rig_state": "ok",
            }

    verdict_path = Path(run_folder) / "analysis" / "verdict.json"
    if not verdict_path.exists():
        return {
            "status": "failed",
            "stage": "read_verdict",
            "run_id": run_id,
            "partial_summary_path": str(Path(run_folder) / "analysis" / "summary.md"),
            "error_summary": "verdict.json missing after report.py exited 0",
            "rig_state": "ok",
        }

    with open(verdict_path, encoding="utf-8") as f:
        headline_kpis = json.load(f)

    return {
        "status": "completed",
        "run_id": run_id,
        "summary_md_path": str(Path(run_folder) / "analysis" / "summary.md"),
        "headline_kpis": headline_kpis,
    }


def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: run.py <run_id>")
    result = run_analysis(sys.argv[1])
    print(json.dumps(result))
    sys.exit(0 if result["status"] == "completed" else 1)


if __name__ == "__main__":
    main()
