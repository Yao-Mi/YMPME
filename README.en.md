# YMPME

Source version **0.4.0** · [Changelog](CHANGELOG.md) · [Regression and stability checks](tests/README.md) · [SDK release](releases/README.md)

A modular C99 physics library for games, mechanism experiments and control simulation on PC/MCU. The core is independent of rendering; optional demos use YMGRE for windows and visualization.

Features include rigid bodies, joints, contact feedback, reusable mechanism models, optional sensors, and independently selectable finite-element statics modules. See the [Chinese README](README.md) for the full demo gallery and current limitations, and [API.md](API.md) for ownership and stepping rules.

## Finite-element static analysis

Three optional modules accept materials, geometry, nodal loads and displacement constraints without requiring GRE, DYNAMICS or sensors:

| Module | Elements and results | CMake option / link target |
| :--- | :--- | :--- |
| [2D statics](YMPME/STATICS/README.md) | Axial springs, trusses and Euler–Bernoulli beams/frames; displacement, rotation, reactions and element forces. | `YMPME_ENABLE_STATICS` / `YMPME::statics` |
| [3D statics](YMPME/STATICS/README3D.md) | Spatial springs/trusses and beams with biaxial bending and torsion; up to six DOFs per node. | `YMPME_ENABLE_STATICS3D` / `YMPME::statics3d` |
| [Solid statics](YMPME/STATICS/README-solid.md) | Four-node tetrahedra (TET4); displacement, reactions, stress, strain, von Mises stress and strain-energy density. | `YMPME_ENABLE_SOLID_STATICS` / `YMPME::solid_statics` |

### Actual GRE demo frames

**2D structures: all 7 cases.** Press `1`–`7` to switch in the [2D viewer](Demo/statics_viewer/README.md); `Tab` selects a node to inspect displacement, rotation and reactions.

| 1 · Cantilever | 2 · Triangular truss |
| :--- | :--- |
| [![1 · Cantilever](docs/screenshots/statics2d.png)](docs/screenshots/statics2d.png) | [![2 · Triangular truss](docs/screenshots/statics2d_truss.png)](docs/screenshots/statics2d_truss.png) |
| Fixed left end; 10 N downward tip load on a 2 m beam. Bending-moment colors. | Pinned left support, right roller; 10 N downward apex load. Bottom chord in tension, diagonals in compression. |

| 3 · Series springs | 4 · Simply supported beam |
| :--- | :--- |
| [![3 · Series springs](docs/screenshots/statics2d_springs.png)](docs/screenshots/statics2d_springs.png) | [![4 · Simply supported beam](docs/screenshots/statics2d_simple_beam.png)](docs/screenshots/statics2d_simple_beam.png) |
| Two 1000 N/m axial springs, 10 N right-end tension; Y motion constrained. | A 2 m beam with pin/roller supports and a 10 N downward midspan load. |

| 5 · Two-span continuous beam | 6 · Portal frame |
| :--- | :--- |
| [![5 · Two-span continuous beam](docs/screenshots/statics2d_continuous_beam.png)](docs/screenshots/statics2d_continuous_beam.png) | [![6 · Portal frame](docs/screenshots/statics2d_frame.png)](docs/screenshots/statics2d_frame.png) |
| Total length 4 m; 10 N downward at the left-span midpoint. Bilateral supports can provide hold-down reactions. | Fixed column bases; Fx=10 N and Fy=-10 N at the upper-right joint. Sway and bending in a rigid frame. |

| 7 · Warren bridge truss |
| :--- |
| [![7 · Warren bridge truss](docs/screenshots/statics2d_bridge.png)](docs/screenshots/statics2d_bridge.png) |
| 9 nodes, 15 members and a 4 m span; 10 N downward at the central bottom joint. Red denotes tension, blue compression. |

**3D beams and trusses: all 3 cases.** Press `1`–`3` in the [3D viewer](Demo/statics3d_viewer/README.md). These are beam/truss elements, with up to six DOFs per node.

| 1 · Biaxial cantilever | 2 · Spatial four-bar truss | 3 · Torsion cantilever |
| :--- | :--- | :--- |
| [![1 · Biaxial cantilever](docs/screenshots/statics3d_bending.png)](docs/screenshots/statics3d_bending.png) | [![2 · Spatial four-bar truss](docs/screenshots/statics3d_truss.png)](docs/screenshots/statics3d_truss.png) | [![3 · Torsion cantilever](docs/screenshots/statics3d.png)](docs/screenshots/statics3d.png) |
| All six root DOFs fixed; tip Fy=-10 N and Fz=5 N produce bending in two directions. | Four base nodes fixed in translation; 10 N downward at the apex. Member axial-force colors. | Tip Mx=3 N·m; section markers show accumulated twist. The centerline stays straight under pure torsion. |

**3D solids: both cases.** Press `1` / `2` in the [solid viewer](Demo/solid_statics_viewer/README.md). Both images show von Mises stress.

| 1 · Cube compression | 2 · Block bending |
| :--- | :--- |
| [![1 · Cube compression](docs/screenshots/solid_statics_cube.png)](docs/screenshots/solid_statics_cube.png) | [![2 · Block bending](docs/screenshots/solid_statics.png)](docs/screenshots/solid_statics.png) |
| 200×200×200 mm; 729 nodes and 3072 TET4 elements. X− face fixed; 1000 N distributed over the X+ face toward −X. | 400×120×120 mm; 325 nodes and 1152 TET4 elements. One end fixed; 1000 N distributed downward over the opposite face. |

The solid viewer also supports internal slices (`Z`), shading modes (`H`), stress/displacement/strain-energy-density fields (`C`) and mesh-density changes (`M`). Slicing changes visibility only; smooth coloring uses averaged nodal values for display and does not improve solver accuracy. Each selectable case appears once above. Full-frame screenshots and reproduction arguments are in the [capture log](docs/screenshots/README.md).

All three modules solve **small-deformation, linear-elastic static equilibrium**. They do not include plasticity, buckling, fracture, nonlinear contact or transient dynamics. Display amplification does not change numerical results. Local solid stress peaks require mesh-refinement checks. These FEM modules are separate from the approximate SOFTBODY pad/ball models.

## Build and run

Core only, with default options in a fresh build directory:

```sh
cmake -S . -B build/core
cmake --build build/core -j
ctest --test-dir build/core --output-on-failure
```

Build all three FEM windows, assuming YMGRE is in the adjacent `0502_YMGRE` directory:

```sh
cmake -S . -B build \
  -DYMPME_GRE_ROOT=../0502_YMGRE \
  -DYMPME_ENABLE_STATICS=ON -DYMPME_BUILD_STATICS_VIEWER=ON \
  -DYMPME_ENABLE_STATICS3D=ON -DYMPME_BUILD_STATICS3D_VIEWER=ON \
  -DYMPME_ENABLE_SOLID_STATICS=ON -DYMPME_BUILD_SOLID_STATICS_VIEWER=ON
cmake --build build --target demo_statics_viewer demo_statics3d_viewer demo_solid_statics_viewer -j
./build/Demo/statics_viewer/demo_statics_viewer
./build/Demo/statics3d_viewer/demo_statics3d_viewer
./build/Demo/solid_statics_viewer/demo_solid_statics_viewer
```

Each executable can also be opened directly, without a launcher script, external model assets or a particular working directory. Enable only the module/viewer pairs you need. For numerical use alone, omit the viewer options; the solver libraries do not require GRE.
