# Copyright 2027 Lunabot. Licensed under the MIT License.

"""Isaac Sim scene builder and ROS 2 bridge for the Lunabot rover.

Deliberately imports NOTHING at package level. Isaac modules can only be
imported after SimulationApp has been constructed, and rclpy must never be
imported here at all -- Isaac's embedded Python is not the ROS distro's.

That restraint is what lets test_scene_layout.py import lunabot_sim.scene.boulders
in ordinary CI, on a machine with no GPU and no Omniverse install.
"""
