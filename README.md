# dactyl-cc, ported to Truck

A Rust port of the original C++/OpenSCAD generator (`../src/dactyl.cc`) that
builds the dactyl-cc case directly as B-rep solids with
[Truck](https://github.com/ricosjp/truck), and writes the STL files without
OpenSCAD.

```
truck/
├── Cargo.toml
├── src/
│   ├── geom.rs      # Shape (union of solids), primitives, booleans, 2D shadow
│   ├── key.rs       # switches, caps, connectors, `Key`   (port of util/key.*)
│   ├── key_data.rs  # key placement / bowl geometry      (port of key_data.*)
│   └── main.rs      # case assembly + output             (port of dactyl.cc)
└── build.sh
```

## Build and run

```
cd truck
./build.sh                 # cargo build --release && generate ./out/*.stl
# or
cargo run --release -- --out out
```

The toolchain is pinned to stable in `rust-toolchain.toml` (one of Truck's
dependencies, `branches`, does not build on nightly).

Outputs (same file names as the OpenSCAD flow):

| file | description |
| --- | --- |
| `left.stl` / `right.stl` | case halves |
| `bottom_left.stl` / `bottom_right.stl` | bottom plates |

Other flags:

* `--out DIR` – output directory (default: current directory)
* `--sanity` – run self-checks of the geometry primitives
* `DACTYL_VERBOSE=1` – print details about handled boolean failures

A full generation takes ~3 seconds.

## How the model is represented

The original program produces OpenSCAD source that is a tree of `union`,
`hull`, `difference`, `projection` and `linear_extrude` nodes. The port maps
that tree onto Truck:

* **`Shape`** (`geom.rs`) is a list of convex solids. Unions are therefore free
  (no boolean is needed to combine shapes); the list is only collapsed when a
  real solid is required.
* **`hull()`** is a convex hull of the input points, computed with
  [`convexhull3d`](https://crates.io/crates/convexhull3d) and turned into planar
  triangle faces. Hull inputs are deduplicated because the same vertex often
  comes from several overlapping solids. (`quickhull` silently dropped hull
  vertices for inputs with a tiny axis extent, e.g. the 0.01 mm wall
  connectors, which produced wrong hulls.)
* **2D hulls** (`projection()`) use Andrew's monotone chain. `convexhull3d`
  rejects degenerate coplanar inputs, and `quickhull`'s 2D hull has the same
  vertex-dropping bug as its 3D version.
* **`cube` / `cylinder` / the cap frustums** are built from prism sweeps or
  explicit planar faces. Cubes are triangulated: `truck-shapeops` mis-handles
  quadrilateral prism faces when cutting triangular hull solids (e.g. the
  thumb-plate cut box left the thumb wall hulls uncut).
* **`cylinder_smooth`** (screw holes in the case) is a revolved cylinder. Its
  B-rep only stores the three circle vertices from Truck's `whole_rsweep`, so
  its 2D hull is a triangle. The bottom-plate screw holes are therefore cut
  with the faceted `cylinder(height, radius, 30)` of the reference instead of
  a 2D hull of the smooth cylinder.
* **`projection() + linear_extrude()`** (only used for the bottom plate) is an
  exact 2D operation: every convex solid's shadow is its projected convex hull,
  the hulls are rasterized and their union outline is traced with marching
  squares, then extruded.
* **`difference()`** is done per convex solid (`and` with the inverted negated
  solid), which is far more reliable than subtracting from the already-unioned
  non-convex case.

## Truck boolean robustness

`truck-shapeops` fails (usually returning `None`, sometimes panicking with
"shell is not oriented and closed") when solids share axis-aligned faces, which
happens constantly in this model (boxes on the same z plane, keys on the same
x/y plane). `subtract_with_nudge` retries with sub-0.05 mm translations and tiny
rotations about the negative solid's own centre. Every boolean is wrapped in
`catch_unwind`; failed unions keep both solids side by side, failed subtractions
keep the solid uncut and are reported in the summary line.

## Validation

Compared against the STL files produced by the OpenSCAD flow (see `../things`
and `../src`):

* bounding boxes of all four files match the OpenSCAD output exactly (0.00 mm
  on the case, <0.1 mm on the plate);
* top-down silhouettes have an IoU of 0.9997 for the case halves and 0.9988
  for the bottom plates (rasterised at 0.2 mm); the five plate screw holes are
  round (30-gon) and match the reference holes pixel for pixel;
* horizontal cross-sections (z = 1 … 52 mm) have an IoU of 0.88 … 1.00;
* every hull point set was checked against the hull inputs parsed from the
  reference `left.scad` (419 hulls, all identical up to rounding);
* screw holes, the thumb-plate cut and all wall geometry are reproduced.

Known small differences:

* the bottom plate outline is traced on a 0.2 mm grid and simplified, so its
  outline can differ from the OpenSCAD plate by ~0.1 mm;
* STL triangle counts are higher than OpenSCAD's (hulls and the plate outline
  are triangulated by Truck), which is fine for slicing;
* a handful of disjoint solid pairs are still reported as failed subtractions
  when `DACTYL_VERBOSE=1`; they are detected (empty intersection) and skipped.
