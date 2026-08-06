# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Resolve every VERIFY in this package against a real Isaac install.

    src/lunabot_sim/scripts/probe_isaac_api.sh --json report.json

Run this FIRST on the simulation machine, before anything else. It answers, in
one command and about a minute, the questions compat.py's docstring currently
tells you to answer by hand in a running session: which extension ids exist,
which OmniGraph node types are registered, where the IMU sensor class lives,
and which shape the URDF importer's API has this release.

Those strings are the reason a first Isaac bring-up takes a day. They were
reconstructed from documentation for a version nobody here has run, they moved
more than once across 4.2 -> 4.5 -> 5.x, and every one of them fails at a
different point in startup with an error that names the symbol rather than the
rename. Finding them one crash at a time is the slow way; this prints the
whole list at once, with what to put in compat.py next to each miss.

It changes nothing. No scene is built, no URDF imported, no ROS traffic sent.

==================== WHAT A FAILURE HERE MEANS ====================
A miss is NOT a bug in this package. It is this package's guess about a name
in a version of Isaac Sim that nobody had run yet. Fix it in compat.py -- the
candidate tuples at the top -- and nowhere else. That is the whole reason
every version-sensitive string is resolved in one file.
===================================================================
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field

from lunabot_sim import compat

# The OmniGraph node types graphs/*.py create, split by which prefix qualifies
# them. Kept here rather than read out of the graph modules because those name
# them inline inside build(); test/test_probe.py parses the graph sources and
# fails if these two lists ever drift from what the code actually creates.
REQUIRED_BRIDGE_NODES = (
    'ROS2CameraHelper',
    'ROS2Context',
    'ROS2PublishClock',
    'ROS2PublishImu',
    'ROS2PublishJointState',
    'ROS2PublishOdometry',
    'ROS2SubscribeJointState',
)

REQUIRED_CORE_NODES = (
    'IsaacArticulationController',
    'IsaacComputeOdometry',
    'IsaacCreateRenderProduct',
    'IsaacReadIMU',
    'IsaacReadSimulationTime',
)

# ROS2CameraHelper's `type` input selects what the helper publishes.
# graphs/camera.py hardcodes these three and marks them VERIFY.
CAMERA_HELPER_TYPES = ('rgb', 'depth', 'depth_pcl')

OK = 'ok'
MISSING = 'missing'
UNKNOWN = 'unknown'
INFO = 'info'


@dataclass
class Finding:
    """One thing the probe looked for, and what it found.

    `fix` is the whole point of the dataclass: a report that says a name is
    missing without saying where to change it has moved the archaeology rather
    than ended it.
    """

    name: str
    status: str
    detail: str = ''
    fix: str = ''

    @property
    def blocking(self) -> bool:
        """True when the sim cannot start until this is resolved.

        UNKNOWN is not blocking: it means the probe could not introspect the
        answer, not that the answer is wrong. Reported, not failed on.
        """
        return self.status == MISSING


@dataclass
class Report:
    findings: list[Finding] = field(default_factory=list)

    def add(self, name, status, detail='', fix=''):
        self.findings.append(Finding(name, status, detail, fix))
        return self.findings[-1]

    @property
    def blocking(self):
        return [f for f in self.findings if f.blocking]

    def to_json(self) -> str:
        return json.dumps({'findings': [asdict(f) for f in self.findings]}, indent=2)

    def to_text(self) -> str:
        width = max((len(f.name) for f in self.findings), default=0)
        lines = ['', 'Isaac API probe', '=' * 72]
        for finding in self.findings:
            status = finding.status.upper()
            lines.append(f'[{status:>7}] {finding.name:<{width}}  {finding.detail}')
            if finding.fix:
                lines.append(f'{"":>10}  -> {finding.fix}')
        lines.append('=' * 72)

        blocking = self.blocking
        if blocking:
            lines += [
                f'{len(blocking)} name(s) did not resolve. This is not a bug in lunabot_sim: it',
                'is its guess about an Isaac version nobody here had run. Fix the candidate',
                'tuples in lunabot_sim/compat.py and nowhere else, then run this again.',
                '',
            ]
        else:
            lines += [
                'Every name resolved. compat.py matches this install; the VERIFY markers',
                'in it can be replaced with the version you just probed.',
                '',
            ]
        return '\n'.join(lines)

    def exit_code(self) -> int:
        return 1 if self.blocking else 0


# ---------------------------------------------------------------------------
# The Isaac-dependent half. Everything above this line imports nothing but the
# standard library and compat, which is what lets test_probe.py run in CI.
# ---------------------------------------------------------------------------


def run(report: Report | None = None) -> Report:
    """Probe a live Isaac session. A SimulationApp must already exist."""
    report = report or Report()

    _probe_environment(report)
    bridge_extension = _probe_extensions(report)
    _probe_node_types(report, bridge_extension)
    _probe_imu_sensor(report)
    _probe_urdf_importer(report)
    return report


def _probe_environment(report: Report) -> None:
    report.add('isaac version', INFO, compat.isaac_version())
    report.add(
        'ROS_DOMAIN_ID',
        INFO,
        os.environ.get('ROS_DOMAIN_ID', 'unset'),
        'The graphs set the domain explicitly (graphs/context.py). This is only '
        'what the bridge would fall back to.',
    )
    report.add('RMW_IMPLEMENTATION', INFO, os.environ.get('RMW_IMPLEMENTATION', 'unset'))


def _probe_extensions(report: Report) -> str | None:
    """Enable the two extensions everything else needs, report which ids worked."""
    bridge = None
    try:
        bridge = compat.enable_ros2_bridge()
        report.add('ROS 2 bridge extension', OK, bridge)
    except compat.IsaacCompatError as exc:
        report.add(
            'ROS 2 bridge extension',
            MISSING,
            str(exc).splitlines()[0],
            'compat.ROS2_BRIDGE_EXTENSIONS. Window > Extensions, search "ros2 bridge".',
        )

    try:
        importer = compat.enable_extension(compat.URDF_IMPORTER_EXTENSIONS)
        report.add('URDF importer extension', OK, importer)
    except compat.IsaacCompatError as exc:
        report.add(
            'URDF importer extension',
            MISSING,
            str(exc).splitlines()[0],
            'compat.URDF_IMPORTER_EXTENSIONS. Window > Extensions, search "urdf".',
        )

    return bridge


def _registered_node_types() -> tuple[set[str], str]:
    """Best-effort inventory of registered OmniGraph node types.

    Returns (names, how). The introspection API has moved as much as
    everything else here, so several are tried and the caller is told which
    one answered -- an empty set with a named method is a different problem
    from an empty set because nothing answered.
    """
    import omni.graph.core as og

    attempts = []

    for accessor in ('get_registered_nodes', 'get_node_types'):
        function = getattr(og, accessor, None)
        if function is None:
            attempts.append(f'og.{accessor}: not present')
            continue
        try:
            names = {str(n) for n in function()}
            if names:
                return names, f'og.{accessor}()'
            attempts.append(f'og.{accessor}: returned nothing')
        except Exception as exc:  # noqa: BLE001 - introspection is best effort
            attempts.append(f'og.{accessor}: {exc}')

    registry = getattr(og, 'GraphRegistry', None)
    if registry is not None:
        try:
            names = {str(n) for n in registry().get_node_types()}
            if names:
                return names, 'og.GraphRegistry().get_node_types()'
            attempts.append('og.GraphRegistry: returned nothing')
        except Exception as exc:  # noqa: BLE001
            attempts.append(f'og.GraphRegistry: {exc}')

    return set(), '; '.join(attempts)


def _probe_node_types(report: Report, bridge_extension: str | None) -> None:
    """Check every node type graphs/*.py creates is registered.

    Falls back to creating each node in a scratch graph when the registry
    cannot be listed. That is slower but definitive: it fails in exactly the
    way graphs/*.py would fail, which is the question being asked.
    """
    required = [(name, compat.og_node_type(name)) for name in REQUIRED_BRIDGE_NODES]
    required += [(name, compat.core_node_type(name)) for name in REQUIRED_CORE_NODES]

    if bridge_extension is None:
        report.add(
            'OmniGraph node types',
            MISSING,
            'skipped: the ROS 2 bridge extension did not load, so none of its '
            'node types can be registered',
            'Resolve the bridge extension first.',
        )
        return

    registered, how = _registered_node_types()
    if registered:
        report.add('node type registry', INFO, f'{len(registered)} types, via {how}')
        for name, qualified in required:
            if qualified in registered:
                report.add(f'node {name}', OK, qualified)
            else:
                near = sorted(n for n in registered if n.rsplit('.', 1)[-1] == name)
                report.add(
                    f'node {name}',
                    MISSING,
                    f'{qualified} is not registered',
                    f'registered under {near}'
                    if near
                    else 'compat.OG_PREFIXES / OG_CORE_PREFIXES, or the node was renamed. '
                    'Read the real type from the OmniGraph editor property panel.',
                )
        return

    report.add(
        'node type registry',
        UNKNOWN,
        f'could not list registered types ({how}); falling back to creating each node',
    )
    _probe_node_types_by_creation(report, required)


def _probe_node_types_by_creation(report: Report, required) -> None:
    import omni.graph.core as og

    graph_path = '/World/Graphs/_ProbeScratch'
    for name, qualified in required:
        try:
            og.Controller.edit(
                {'graph_path': graph_path, 'evaluator_name': 'push'},
                {og.Controller.Keys.CREATE_NODES: [(f'Probe_{name}', qualified)]},
            )
            report.add(f'node {name}', OK, f'{qualified} (created)')
        except Exception as exc:  # noqa: BLE001 - any failure is the answer
            report.add(
                f'node {name}',
                MISSING,
                f'{qualified}: {exc}',
                'compat.OG_PREFIXES / OG_CORE_PREFIXES, or the node was renamed.',
            )

    # The scratch graph is left on the stage. It is never played, never saved,
    # and the app closes seconds later -- deleting it would mean guessing at
    # another API this probe exists because nobody has verified.


def _probe_imu_sensor(report: Report) -> None:
    """run_sim.py creates an IMUSensor; the module holding it moved in 4.5."""
    candidates = ('isaacsim.sensors.physics', 'omni.isaac.sensor')
    try:
        compat.import_first(candidates, 'IMUSensor')
        report.add('IMUSensor class', OK, ' or '.join(candidates))
    except compat.IsaacCompatError as exc:
        report.add(
            'IMUSensor class',
            MISSING,
            str(exc).splitlines()[0],
            'run_sim.py::_create_imu_sensor. Without it there is no /oak_d/imu/data, '
            'so rtabmap loses its gravity constraint and the EKF loses its only real '
            'motion measurement.',
        )


def _probe_urdf_importer(report: Report) -> None:
    """Which of the two importer API shapes robot/importer.py can use.

    Both are tried there, in order. Knowing WHICH one this install has turns a
    failed import into a one-line answer instead of two stack traces.
    """
    try:
        import omni.kit.commands

        commands = set(omni.kit.commands.get_commands())
        for command in ('URDFCreateImportConfig', 'URDFParseAndImportFile'):
            if command in commands:
                report.add(f'kit command {command}', OK)
            else:
                near = sorted(c for c in commands if 'URDF' in c.upper())
                report.add(
                    f'kit command {command}',
                    MISSING,
                    'not registered',
                    f'URDF-ish commands present: {near}'
                    if near
                    else 'robot/importer.py, which falls back to the legacy interface below.',
                )
    except Exception as exc:  # noqa: BLE001
        report.add('kit commands', UNKNOWN, f'could not list: {exc}')

    try:
        compat.import_first(compat.URDF_IMPORTER_EXTENSIONS, '_urdf')
        report.add('legacy urdf interface', OK, 'acquire_urdf_interface path is available')
    except compat.IsaacCompatError:
        report.add(
            'legacy urdf interface',
            UNKNOWN,
            'not available; the kit command path above is the only one',
        )


def main(argv=None) -> int:
    """Start a headless SimulationApp, probe it, print the report."""
    import argparse
    import sys
    from pathlib import Path

    parser = argparse.ArgumentParser(
        description='Probe an Isaac install for the names lunabot_sim/compat.py guesses at.'
    )
    parser.add_argument('--json', help='Also write the findings to this path, as JSON.')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    # SimulationApp FIRST, before any other omni import, exactly as in
    # run_sim.py. Headless: the probe renders nothing, and asking for a
    # viewport on a machine still being set up is one more thing to go wrong.
    simulation_app_class = compat.get_simulation_app_class()
    simulation_app = simulation_app_class({'headless': True})

    try:
        report = run()
    finally:
        simulation_app.close()

    print(report.to_text())
    if args.json:
        Path(args.json).write_text(report.to_json())
        print(f'findings written to {args.json}')
    return report.exit_code()


if __name__ == '__main__':
    import sys

    sys.exit(main())
