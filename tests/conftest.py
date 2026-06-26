"""
Shared pytest fixtures for all test modules in this directory.

pytest loads this file automatically before any test module — fixtures defined
here are available to every test_*.py file without an explicit import.
"""
import sys
import json
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pytest


@pytest.fixture
def cfg():
    with open(Path(__file__).parent.parent / "src" / "config.json") as f:
        return json.load(f)
