# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from glob import glob

from setuptools import find_packages, setup

package_name = 'lunabot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('lunabot_sim/config/*.yaml')),
        # Installed so `ros2 pkg prefix lunabot_sim` can find it, but note the
        # script deliberately puts the SOURCE directory on PYTHONPATH: it is
        # executed by Isaac's Python, which knows nothing about the colcon
        # install tree.
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Alexander Halley',
    maintainer_email='alexhalley@outlook.fr',
    description=(
        'Isaac Sim scene builder and ROS 2 bridge graphs for the Lunabot rover. '
        'Imports no ROS packages: Isaac runs its own Python.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # No console_scripts on purpose. run_sim must be executed by Isaac's
        # python.sh, not by the ROS environment's Python, so a `ros2 run`
        # entry point would only invite people to launch it the wrong way.
        # Use scripts/run_isaac_sim.sh.
    },
)
