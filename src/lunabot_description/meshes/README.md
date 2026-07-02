# meshes

Export chassis / wheel / excavation-arm CAD here (`.dae` or `.stl`) and reference
them from the xacro `<visual>`/`<collision>` geometry. Until then the URDF uses
box/cylinder primitives so it still imports into Isaac Sim and shows TF.

Keep collision meshes simple (convex/primitive) — PhysX and Nav2 costmaps do not
need the detailed visual mesh.
