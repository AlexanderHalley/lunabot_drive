# maps

Saved occupancy grids, for `localization:=true` runs.

Empty by design. Maps are build artefacts of a particular arena on a
particular day; committing one invites someone to localise against a map of
last year's practice bin.

To save one from a live rtabmap session:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/<arena>_<date>
```

That writes a `.pgm` and a `.yaml`. If you do commit a map, commit both, and
put the arena and date in the filename.

Note that rtabmap keeps its own database (`~/.ros/rtabmap.db`) which is a
richer thing than an occupancy grid — for `slam:=rtabmap localization:=true`
that database is what matters, not the files here. These are for Nav2's
`map_server` path.
