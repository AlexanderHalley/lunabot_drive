# Copyright 2027 Lunabot. Licensed under the MIT License.

"""OmniGraph builders, one module per concern.

Separate graphs rather than one large one: a graph that fails to evaluate
takes down everything in it, and losing the camera must not stop /clock.
Without a clock, every ROS node with use_sim_time blocks at time zero.
"""
