# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""OmniGraph builders, one module per concern.

Separate graphs rather than one large one: a graph that fails to evaluate
takes down everything in it, and losing the camera must not stop /clock.
Without a clock, every ROS node with use_sim_time blocks at time zero.
"""
