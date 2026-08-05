"""Lint the Python that CI can actually import.

Only checks style. The Isaac-dependent modules are still linted -- flake8 does
not import them -- but nothing here proves they run.
"""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, '\n'.join(errors)
