# YMPME

A small C99 translational 3D physics library for PC/MCU games, organized like YMGRE.

Includes sphere/AABB collision, static/dynamic/kinematic bodies, gravity, forces, impulses, friction, fixed stepping, external memory, and an optional YMGRE mesh adapter. Adds optional closed convex-mesh SAT after AABB rejection, 32 collision groups and same-assembly exclusion. No concave triangle-soup collisions, angular dynamics, joints or CCD yet.

Build: `cmake -S . -B build && cmake --build build && ctest --test-dir build`.

See [README.md](README.md) for examples and [API.md](API.md) for ownership and stepping rules.
