from ament_mypy.main import main

import pytest


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy() -> None:
    rc = main(["--config", "../../mypy.ini"])
    assert rc == 0, 'Found code style errors / warnings'