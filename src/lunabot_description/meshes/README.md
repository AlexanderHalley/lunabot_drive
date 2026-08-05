# meshes

Empty on purpose.

The robot is built from primitives — boxes and cylinders. That is a deliberate choice, not a
placeholder waiting for CAD:

- **Collision geometry should stay primitive regardless.** Isaac's contact solver is where the
  simulation's cost lives, and a wheel with real tread geometry costs an enormous amount of solver
  time to model something that is a friction-coefficient question, not a geometry one. Traction on
  regolith gets tuned in `lunabot_sim/scene/terrain.py`, not here.
- **Visual meshes are the only thing worth adding**, and only once the 2027 chassis exists.

## If you add one

- Keep it under ~2 MB. Meshes live in git forever and RViz reloads them on every start.
- Decimate hard — a visual mesh needs to read correctly at 1 m, not machine correctly.
- Reference it as `package://lunabot_description/meshes/<file>` so it resolves after install.
- Add it to `<visual>` only. Leave `<collision>` as the primitive.
- Export with Z up and the origin at the link frame, or spend an afternoon on offset transforms.
