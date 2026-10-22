# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Competition arena layouts: walls, zones, the berm target, posts.

Isaac-free, like boulders.py, so test_arena_layout.py checks every layout in
ordinary CI. arena.py draws them.

WORLD FRAME: the rover spawns at the origin facing +X, and that is also where
`map` starts. So every layout is placed with its start zone around the
origin and the arena running away down +X. That is ONE of the competition's
starting poses -- the real start position and heading are drawn at random
before each run. Randomising the spawn is not done yet.

Sources, and how far to trust them:

  ucf   UCF Lunabotics 2025 Guidebook, section 3.1 and Figure 1 ("UCF Arena
        Layout"): two lanes of 8.14 m x 4.57 m; this is ONE lane. Zone sizes
        are read off the figure. Berm and target area from Figure 4.
  ksc   The Artemis Arena at KSC (finals): NASA's facility page gives ~6.8 m x
        ~5.0 m between the perimeter ducts and a central support column that
        "must be avoided". It publishes NO zone positions, column size or wall
        height; those are guesses on the UCF pattern and are marked APPROX.
        Replace from the NASA guidebook's arena figure when it is in hand.
  open  The old placeholder: flat ground, no walls, rocks in x 1-6, y +-2.
        Kept so results from before the arena existed can be compared.

Craters (at least three, up to 40-50 cm, in the obstacle zone per the
guidebook) are NOT here: the terrain is flat. See terrain.py.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle on the ground, metres, world frame."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def size(self) -> tuple[float, float]:
        return (self.x_max - self.x_min, self.y_max - self.y_min)

    @property
    def centre(self) -> tuple[float, float]:
        return ((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        """True if (x, y) lies inside, after growing the rectangle by `margin`."""
        return (
            self.x_min - margin <= x <= self.x_max + margin
            and self.y_min - margin <= y <= self.y_max + margin
        )

    def inside(self, other: Rect) -> bool:
        return (
            other.x_min <= self.x_min
            and self.x_max <= other.x_max
            and other.y_min <= self.y_min
            and self.y_max <= other.y_max
        )


@dataclass(frozen=True)
class Post:
    """A fixed vertical obstacle: a square post or a round column."""

    x: float
    y: float
    size: float
    """Side of a square post, or diameter of a round column."""

    height: float
    round: bool = False

    def footprint(self) -> Rect:
        half = self.size / 2.0
        return Rect(self.x - half, self.x + half, self.y - half, self.y + half)


@dataclass(frozen=True)
class ArenaLayout:
    name: str
    source: str

    floor: Rect | None
    """Inside faces of the walls. None means no walls (the 'open' layout)."""

    wall_height: float = 0.6
    """APPROX everywhere: no source gives it. Tall enough to fill the camera's
    view of the edge, low enough not to shade the whole floor at sun 8 deg."""

    wall_thickness: float = 0.10

    start_zone: Rect | None = None
    obstacle_zone: Rect | None = None
    excavation_zone: Rect | None = None
    construction_zone: Rect | None = None
    berm_target: Rect | None = None
    """The box only berm volume inside counts (guidebook Figure 4)."""

    fiducial_rail: Rect | None = None
    """On the start-end wall; drawn as a dark strip. Zero-depth in x."""

    posts: tuple[Post, ...] = field(default_factory=tuple)

    def keep_outs(self) -> tuple[Rect, ...]:
        """Where no boulder may be placed: start, construction, posts."""
        zones = [z for z in (self.start_zone, self.construction_zone) if z is not None]
        return tuple(zones) + tuple(p.footprint() for p in self.posts)

    def to_dict(self) -> dict:
        return asdict(self)


# ==================== UCF: one lane of Figure 1 ====================
# Lane 8.14 m long, 4.57 m wide. Obstacle half (4.07 m) holds the start box
# against the end wall, beside the fiducial rail, near the lane divider;
# excavation half (4.07 m) beyond it, with the 2.6 m x 2 m construction zone
# in its far corner on the side away from the divider.
_UCF_X0 = -0.75  # end wall behind the rover
_UCF_Y0 = -1.00  # lane divider, to the rover's right
_UCF_L, _UCF_W = 8.14, 4.57
_UCF_HALF = _UCF_L / 2.0
_UCF_X1, _UCF_Y1 = _UCF_X0 + _UCF_L, _UCF_Y0 + _UCF_W
_UCF_POST = 0.30  # hatched squares in Figure 1; size APPROX

UCF = ArenaLayout(
    name='ucf',
    source='UCF Lunabotics 2025 Guidebook, sec. 3.1, Fig. 1 and Fig. 4 (one of two lanes)',
    floor=Rect(_UCF_X0, _UCF_X1, _UCF_Y0, _UCF_Y1),
    start_zone=Rect(_UCF_X0, _UCF_X0 + 2.0, _UCF_Y0, _UCF_Y0 + 2.0),
    obstacle_zone=Rect(_UCF_X0, _UCF_X0 + _UCF_HALF, _UCF_Y0, _UCF_Y1),
    excavation_zone=Rect(_UCF_X0 + _UCF_HALF, _UCF_X1, _UCF_Y0, _UCF_Y1),
    construction_zone=Rect(_UCF_X1 - 2.6, _UCF_X1, _UCF_Y1 - 2.0, _UCF_Y1),
    # UCF target 1.5 m x 0.9 m (berm 1.3 x 0.7), centred in the zone.
    berm_target=Rect(_UCF_X1 - 1.3 - 0.75, _UCF_X1 - 1.3 + 0.75, _UCF_Y1 - 1.45, _UCF_Y1 - 0.55),
    fiducial_rail=Rect(_UCF_X0, _UCF_X0, _UCF_Y0 + 0.5, _UCF_Y0 + 1.5),
    posts=tuple(
        Post(x, y, _UCF_POST, 1.0)
        for x in (_UCF_X0, _UCF_X0 + _UCF_HALF, _UCF_X1)
        for y in (_UCF_Y0, _UCF_Y1)
    ),
)


# ==================== KSC: Artemis Arena, APPROXIMATE ====================
# 6.8 m x 5.0 m (NASA facility page). Zones laid out on the UCF pattern,
# halved along the length; KSC berm target 2.2 m x 0.9 m (Figure 4). Column
# at the centre of the floor, 0.3 m round, both APPROX.
_KSC_X0, _KSC_Y0 = -0.75, -2.5
_KSC_L, _KSC_W = 6.8, 5.0
_KSC_HALF = _KSC_L / 2.0
_KSC_X1, _KSC_Y1 = _KSC_X0 + _KSC_L, _KSC_Y0 + _KSC_W

KSC = ArenaLayout(
    name='ksc',
    source='NASA KSC Artemis Arena page (size, column); zones APPROX on the UCF pattern',
    floor=Rect(_KSC_X0, _KSC_X1, _KSC_Y0, _KSC_Y1),
    start_zone=Rect(_KSC_X0, _KSC_X0 + 2.0, -1.0, 1.0),
    obstacle_zone=Rect(_KSC_X0, _KSC_X0 + _KSC_HALF, _KSC_Y0, _KSC_Y1),
    excavation_zone=Rect(_KSC_X0 + _KSC_HALF, _KSC_X1, _KSC_Y0, _KSC_Y1),
    construction_zone=Rect(_KSC_X1 - 2.6, _KSC_X1, _KSC_Y1 - 1.5, _KSC_Y1),
    berm_target=Rect(_KSC_X1 - 2.4, _KSC_X1 - 0.2, _KSC_Y1 - 1.2, _KSC_Y1 - 0.3),
    posts=(Post((_KSC_X0 + _KSC_X1) / 2.0, 0.0, 0.30, 2.5, round=True),),
)


# ==================== open: the pre-arena placeholder ====================
OPEN = ArenaLayout(
    name='open',
    source='placeholder: no walls, rocks in x 1-6, y +-2 (boulders.ArenaConfig defaults)',
    floor=None,
)

LAYOUTS = {layout.name: layout for layout in (UCF, KSC, OPEN)}
LAYOUTS['default'] = UCF
