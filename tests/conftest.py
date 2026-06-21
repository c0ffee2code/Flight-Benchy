import sys
import json
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pytest


@pytest.fixture
def cfg():
    with open(Path(__file__).parent.parent / "src" / "config.json") as f:
        return json.load(f)
