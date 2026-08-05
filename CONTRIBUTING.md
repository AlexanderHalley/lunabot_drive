# Contributing

New here? Start with [`docs/ONBOARDING.md`](docs/ONBOARDING.md).

## Before you push

```bash
colcon test && colcon test-result --verbose
pre-commit run --all-files
```

## The rules that are not style preferences

**The contract wins.** [`docs/TOPIC_FRAME_CONTRACT.md`](docs/TOPIC_FRAME_CONTRACT.md) is
authoritative for topic and frame names. If code disagrees with it, the code is wrong. If a name
genuinely needs to change, change the document *first*, in the same commit.

This is not ceremony. The 2026 README documented `/camera/image_raw` while the launch files
published `/oak_d/rgb/image_raw`, and nobody noticed until someone tried the documented name.

**One publisher per transform.** `odom → base_link` has three possible owners. `odom_source:=`
picks exactly one. Two publishers produce a TF tree that looks correct in `view_frames` and behaves
nondeterministically — the worst kind of bug, because it is invisible until it matters.

**Sim and real run the same stack.** Adding a node that runs only in simulation, or a config that
differs between them, breaks the property the whole workspace is built on. If you find yourself
writing `if sim:`, stop and ask what the hardware plugin should be doing instead.

**Nothing goes in `lunabot_msgs` that exists upstream.** See
[`src/lunabot_msgs/README.md`](src/lunabot_msgs/README.md).

**One linter per language.** `ruff` owns Python — configured in `ruff.toml`, enforced by
`pre-commit` and the CI lint job. `clang-format` owns C++, via `.clang-format`. Nothing else may
lint the same files.

This is not a preference. The workspace briefly ran `ruff` *and* `ament_flake8` + `ament_pep257`,
plus `ament_uncrustify` alongside `clang-format`, and the two systems disagree — roughly **400
style failures out of 617 tests, and zero functional ones**, each linter undoing the other's work.

Those three are switched off in every `CMakeLists.txt` with:

```cmake
set(ament_cmake_flake8_FOUND TRUE)
set(ament_cmake_pep257_FOUND TRUE)
set(ament_cmake_uncrustify_FOUND TRUE)
```

before `ament_lint_auto_find_test_dependencies()` — ament's documented skip mechanism. **Everything
else in `ament_lint_common` still runs**: copyright, cpplint, cppcheck, lint_cmake, xmllint.

Two consequences worth knowing:

- **`ruff.toml` has to carry what those linters provided**, or coverage is lost rather than
  deduplicated. That is why it selects `I` (isort, replacing `flake8-import-order`) and `D`
  (pydocstyle, replacing `ament_pep257`) rather than running on ruff's defaults.
- **`flake8-quotes` settings must match `format.quote-style`.** Ruff's `Q` rules default to double
  quotes regardless of the formatter, so leaving them unset reproduces the exact linter-versus-
  formatter loop inside a single tool.

If you re-add `ament_flake8`, you are signing up to reconcile two style systems. Don't.

**Every source file carries a copyright header.** `ament_copyright` still runs and will tell you.

```
# Copyright 2027 Lunabot. Licensed under the MIT License.
```

**Kinematic constants live in two files and must agree.** `wheel_radius` and `wheel_separation` are
in both `properties.xacro` and `controllers.yaml`, because xacro cannot reach into a controller
YAML. Change one, change the other, same commit. If they disagree, odometry is wrong and nothing
reports it.

## Comments

Explain *why*, not *what*. `// increment counter` earns nothing; `// 400P because the Pi cannot
move 720P stereo over USB and DDS at once` saves someone an afternoon.

When you disable something, say what broke. Half the value in `oak_d_s2.yaml` is the record of
which filter caused which failure on which hardware.

## Placeholders

Mark unfinished work so it can be found:

- `PLACEHOLDER` — a value that must be measured or specified
- `VERIFY` — an API or parameter name not checked against a real install
- `TODO(2027)` — deliberately deferred work

Do not quietly leave a plausible-looking number where a measured one belongs. A confident wrong
value costs far more than an obvious gap.

## Commits

Conventional prefixes (`feat(sim):`, `fix(hardware):`, `docs:`, `chore:`). Say **why** in the body,
not just what — the diff already says what.

Each commit should build and pass tests on its own.

## Adding a package

1. `src/<name>/` with `package.xml` and `CMakeLists.txt`.
2. `ament_cmake` unless the deliverable is genuinely an importable Python module with logic in it.
   `install(DIRECTORY ...)` is one line and never drifts the way `setup.py` + `data_files` does.
   `lunabot_sim` is the only package that earns `ament_python`.
3. Wire `ament_lint_auto` into `BUILD_TESTING`.
4. Add it to the `package-name` list in `.github/workflows/ci.yml`.
5. Write at least one test that runs with no hardware and no GPU.
