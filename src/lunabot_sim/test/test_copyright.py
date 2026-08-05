# Copyright 2027 Lunabot. Licensed under the MIT License.

"""Check every source file carries a copyright header.

This package is ament_python, so its linters run as pytest files rather than
through ament_lint_auto. ament_copyright was already declared as a test
dependency with no test file invoking it -- so copyright was silently
unchecked in the package holding the most Python in the workspace.

Style linting is NOT done here. ruff owns Python, via pre-commit and the CI
lint job; a second style linter would only disagree with it. See
CONTRIBUTING.md, "One linter per language".
"""

import pytest
from ament_copyright.main import main


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    assert main(argv=[]) == 0, 'found files with a missing or malformed copyright header'
