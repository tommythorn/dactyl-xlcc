//! Geometry helpers on top of the Truck CAD kernel.
//!
//! The whole model is expressed as a union of *convex* solids (boxes, cylinders,
//! extruded convex polygons and convex hulls of point sets).  Keeping the union
//! as a list of solids makes unions virtually free and lets us compute 2D
//! "shadows" (`projection()`) exactly, because the shadow of a convex solid is
//! the convex hull of its projected vertices.

use std::collections::HashMap;
use truck_meshalgo::prelude::*;
use truck_modeling::*;
use truck_shapeops::{and, or};

/// Tolerance used by the boolean operations and by tessellation.
pub const BOOL_TOL: f64 = 0.05;

/// Set `DACTYL_VERBOSE=1` to print the details of handled boolean failures.
pub fn verbose() -> bool {
  std::env::var("DACTYL_VERBOSE").is_ok()
}
pub const MESH_TOL: f64 = 0.05;

/// A shape is the union of a list of solids.
///
/// All solids stored here are expected to be convex (or at least a convex
/// union of convex solids); see the module docs for why that matters.
#[derive(Clone, Debug, Default)]
pub struct Shape {
  pub solids: Vec<Solid>,
}

impl Shape {
  pub fn new(solid: Solid) -> Self {
    Shape { solids: vec![solid] }
  }

  pub fn union_all(shapes: Vec<Shape>) -> Self {
    let mut solids = Vec::new();
    for shape in shapes {
      solids.extend(shape.solids);
    }
    Shape { solids }
  }

  pub fn union(self, other: &Shape) -> Self {
    let mut solids = self.solids.clone();
    solids.extend(other.solids.clone());
    Shape { solids }
  }

  /// Applies one affine transform to every solid.
  pub fn transform(&self, mat: Matrix4) -> Self {
    Shape {
      solids: self.solids.iter().map(|s| builder::transformed(s, mat)).collect(),
    }
  }

  pub fn translate(&self, x: f64, y: f64, z: f64) -> Self {
    self.transform(Matrix4::from_translation(Vector3::new(x, y, z)))
  }

  pub fn translate_z(&self, z: f64) -> Self {
    self.translate(0.0, 0.0, z)
  }

  pub fn rotate_x(&self, deg: f64) -> Self {
    self.transform(Matrix4::from_angle_x(Rad(deg.to_radians())))
  }

  #[allow(dead_code)]
  pub fn rotate_y(&self, deg: f64) -> Self {
    self.transform(Matrix4::from_angle_y(Rad(deg.to_radians())))
  }

  pub fn rotate_z(&self, deg: f64) -> Self {
    self.transform(Matrix4::from_angle_z(Rad(deg.to_radians())))
  }

  #[allow(dead_code)]
  pub fn mirror_x(&self) -> Self {
    let mat = Matrix4::from_nonuniform_scale(-1.0, 1.0, 1.0);
    Shape {
      solids: self
        .solids
        .iter()
        .map(|s| {
          let mut mirrored = builder::transformed(s, mat);
          mirrored.not();
          mirrored
        })
        .collect(),
    }
  }

  /// Convex hull of the vertices of all the shapes.
  pub fn hull(shapes: &[Shape]) -> Self {
    let mut points = Vec::new();
    for shape in shapes {
      for solid in &shape.solids {
        points.extend(solid_points(solid));
      }
    }
    match hull_solid(&points) {
      Some(solid) => {
        if std::env::var("DACTYL_CHECK_HULLS").is_ok() {
          check_hull(&points, &solid);
        }
        Shape::new(solid)
      }
      None => {
        HULL_FAILURES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        Shape::default()
      }
    }
  }

  /// Union of the 2D hulls of every solid, extruded along z.
  ///
  /// This is the OpenSCAD `projection() + linear_extrude()` idiom, valid
  /// because every solid in a `Shape` is convex.
  pub fn project_extrude(&self, height: f64, center: bool) -> Self {
    let mut solids = Vec::new();
    for solid in &self.solids {
      let poly = hull_2d(solid_points(solid));
      if poly.len() < 3 {
        continue;
      }
      if let Some(prism) = prism(&poly, height) {
        solids.push(if center {
          builder::translated(&prism, Vector3::new(0.0, 0.0, -height / 2.0))
        } else {
          prism
        });
      }
    }
    Shape { solids }
  }

  /// Boolean union of every solid in this shape.
  pub fn collapse(&self) -> Option<Solid> {
    union_solids(&self.solids)
  }

  /// Difference of this shape with `other`.
  ///
  /// Every solid of `self` is cut by every negative solid that overlaps its
  /// bounding box. Doing the subtraction solid by solid (instead of first
  /// unioning everything) is much more robust than a boolean on the whole
  /// non-convex case.
  pub fn subtract(&self, other: &Shape) -> Self {
    if other.solids.is_empty() {
      return self.clone();
    }
    let mut solids: Vec<Solid> = Vec::new();
    for solid in &self.solids {
      let bb = bounding_box(solid);
      let mut current = Some(solid.clone());
      for (neg_index, neg) in other.solids.iter().enumerate() {
        let Some(base_solid) = current.as_ref() else {
          break;
        };
        let nbb = bounding_box(neg);
        if !boxes_overlap(&bb, &nbb) {
          continue;
        }
        // A base solid completely inside the negative disappears. `truck-shapeops`
        // cannot handle nested solids, so handle it directly.
        let base_points = solid_points(base_solid);
        if convex_contains(neg, &base_points) {
          SUBTRACT_DROPPED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
          current = None;
          continue;
        }
        // The bounding box is just a cheap filter. A disjoint pair is handled
        // by `guarded_and` returning an empty solid (which is treated as a
        // failed subtraction and keeps the base).
        let mut n = neg.clone();
        n.not();
        match subtract_with_nudge(base_solid, &n) {
          Some(result) => current = Some(result),
          None => {
            // Disjoint solids (overlapping bounding boxes only) are not errors:
            // their intersection boolean is empty, so the base is kept as is.
            let disjoint = match guarded_and(base_solid, neg) {
              Some(intersection) => {
                intersection.boundaries().iter().all(|sh| sh.is_empty())
              }
              None => true,
            };
            if disjoint {
              continue;
            }
            SUBTRACT_FAILURES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if !verbose() {
              continue;
            }
            let b = bounding_box(base_solid);
            let sc = base_solid
              .boundaries()
              .iter()
              .map(|sh| sh.shell_condition())
              .collect::<Vec<_>>();
            eprintln!(
              "warning: subtraction of negative {} failed (shells {:?}, faces {}, verts {}, size {:.2}x{:.2}x{:.2} at {:.1},{:.1},{:.1})",
              neg_index,
              sc,
              base_solid.boundaries().iter().map(|sh| sh.len()).sum::<usize>(),
              solid_points(base_solid).len() / 2,
              b.1.x - b.0.x,
              b.1.y - b.0.y,
              b.1.z - b.0.z,
              b.0.x,
              b.0.y,
              b.0.z
            );
          }
        }
      }
      if let Some(current) = current {
        solids.push(current);
      }
    }
    Shape { solids }
  }
}

// ---------------------------------------------------------------------------
// Solid construction
// ---------------------------------------------------------------------------

/// Vertices of a solid (with duplicates).
pub fn solid_points(solid: &Solid) -> Vec<Point3> {
  let mut points = Vec::new();
  for shell in solid.boundaries() {
    for face in shell.face_iter() {
      for wire in face.boundaries() {
        for edge in wire.edge_iter() {
          points.push(edge.front().point());
          points.push(edge.back().point());
        }
      }
    }
  }
  points
}

/// Builds a solid out of a vertex list and a list of planar polygon faces.
///
/// The face vertex order is the polygon winding, it must be consistent across
/// all faces (i.e. all faces wound the same rotational direction).
pub fn solid_from_faces(vertices: &[Point3], faces: &[Vec<usize>]) -> Option<Solid> {
  let verts: Vec<Vertex> = vertices.iter().map(|p| Vertex::new(*p)).collect();
  let mut edges: HashMap<(usize, usize), Edge> = HashMap::new();
  let mut face_list: Vec<Face> = Vec::new();

  for indices in faces {
    if indices.len() < 3 {
      continue;
    }
    let mut wire = Wire::new();
    for k in 0..indices.len() {
      let a = indices[k];
      let b = indices[(k + 1) % indices.len()];
      if a == b {
        return None;
      }
      let key = (a.min(b), a.max(b));
      let edge = match edges.get(&key) {
        Some(edge) => {
          if a < b {
            edge.clone()
          } else {
            edge.inverse()
          }
        }
        None => {
          let new_edge = Edge::new(
            &verts[key.0],
            &verts[key.1],
            Curve::Line(Line(vertices[key.0], vertices[key.1])),
          );
          edges.insert(key, new_edge.clone());
          if a < b {
            new_edge
          } else {
            new_edge.inverse()
          }
        }
      };
      wire.push_back(edge);
    }
    let face = builder::try_attach_plane(&[wire]).ok()?;
    face_list.push(face);
  }

  if face_list.is_empty() {
    return None;
  }
  let mut shell = Shell::new();
  for face in face_list {
    shell.push(face);
  }
  Some(Solid::new_unchecked(vec![shell]))
}

/// Convex hull of a point set, as a Truck solid.
///
/// Uses `convexhull3d` (a Rust port of convhull_3d) which returns the exact
/// convex hull with outward-oriented triangles. (The `quickhull` crate 0.1
/// silently drops hull vertices when one axis extent is tiny, which broke every
/// hull built from the 0.01 mm post connectors.)
pub fn hull_solid(points: &[Point3]) -> Option<Solid> {
  let unique = dedupe_points(points);
  if unique.len() < 4 {
    return None;
  }
  let verts: Vec<convexhull3d::Vertex> = unique
    .iter()
    .map(|p| convexhull3d::Vertex::new(p.x, p.y, p.z))
    .collect();
  let hull = convexhull3d::ConvexHull3D::build(&verts).ok()?;
  if hull.num_faces() < 4 {
    return None;
  }
  let hull_vertices: Vec<Point3> = hull
    .vertices()
    .iter()
    .map(|v| Point3::new(v.x, v.y, v.z))
    .collect();
  let triangles: Vec<[usize; 3]> = hull
    .faces()
    .iter()
    .map(|f| {
      let i = f.indices();
      [i[0], i[1], i[2]]
    })
    .collect();
  HULL_POLY.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
  let solid = build_solid_from_triangles(&hull_vertices, &triangles)?;
  if let Ok(mut plog) = HULL_POINTS_LOG.lock() {
    plog.push(unique.iter().map(|p| (p.x, p.y, p.z)).collect());
  }
  if let Ok(mut log) = HULL_BBOX_LOG.lock() {
    let (mn, mx) = bounding_box(&solid);
    log.push([mn.x, mn.y, mn.z, mx.x, mx.y, mx.z, unique.len() as f64]);
  }
  Some(solid)
}

/// Same hull as `hull_solid` (kept for the hull regression check).
pub fn hull_solid_triangles(points: &[Point3]) -> Option<Solid> {
  hull_solid(points)
}

pub fn dedupe_points(points: &[Point3]) -> Vec<Point3> {
  let mut seen: std::collections::HashSet<(i64, i64, i64)> = std::collections::HashSet::new();
  let mut unique: Vec<Point3> = Vec::new();
  for p in points {
    let key = (
      (p.x * 1e5).round() as i64,
      (p.y * 1e5).round() as i64,
      (p.z * 1e5).round() as i64,
    );
    if seen.insert(key) {
      unique.push(*p);
    }
  }
  unique
}

/// Builds a solid shell out of a triangle list (shares edges between faces).
pub fn build_solid_from_triangles(vertices: &[Point3], triangles: &[[usize; 3]]) -> Option<Solid> {
  if triangles.len() < 4 {
    return None;
  }
  let verts: Vec<Vertex> = vertices.iter().map(|p| Vertex::new(*p)).collect();
  let mut edges: HashMap<(usize, usize), Edge> = HashMap::new();
  let mut face_list: Vec<Face> = Vec::new();
  for idx in triangles {
    if idx[0] == idx[1] || idx[1] == idx[2] || idx[0] == idx[2] {
      continue;
    }
    let a = vertices[idx[0]];
    let b = vertices[idx[1]];
    let c = vertices[idx[2]];
    let mut wire = Wire::new();
    for k in 0..3 {
      let x0 = idx[k];
      let x1 = idx[(k + 1) % 3];
      let key = (x0.min(x1), x0.max(x1));
      let edge = match edges.get(&key) {
        Some(e) => e.clone(),
        None => {
          let e = Edge::new(
            &verts[key.0],
            &verts[key.1],
            Curve::Line(Line(vertices[key.0], vertices[key.1])),
          );
          edges.insert(key, e.clone());
          e
        }
      };
      wire.push_back(if x0 < x1 { edge } else { edge.inverse() });
    }
    let plane = Plane::new(a, b, c);
    match Face::try_new(vec![wire], Surface::Plane(plane)) {
      Ok(face) => face_list.push(face),
      Err(_) => return None,
    }
  }
  if face_list.len() < 4 {
    return None;
  }
  let mut shell = Shell::new();
  for face in face_list {
    shell.push(face);
  }
  Some(Solid::new_unchecked(vec![shell]))
}

/// Monte Carlo volume estimate of a convex solid from its face planes.
pub fn hull_volume_mc(solid: &Solid) -> f64 {
  let (mn, mx) = bounding_box(solid);
  let hull_points = solid_points(solid);
  let centroid = hull_points
    .iter()
    .fold(Vector3::new(0.0, 0.0, 0.0), |acc, p| acc + p.to_vec())
    / hull_points.len().max(1) as f64;
  let centroid = Point3::new(centroid.x, centroid.y, centroid.z);
  let mut planes: Vec<(Vector3, Point3)> = Vec::new();
  for shell in solid.boundaries() {
    for face in shell.face_iter() {
      if let Surface::Plane(plane) = face.surface() {
        let origin = plane.origin();
        let mut n = plane.normal();
        if n.dot(centroid - origin) > 0.0 {
          n = -1.0 * n;
        }
        planes.push((n, origin));
      }
    }
  }
  if planes.is_empty() {
    return 0.0;
  }
  let mut seed = 0x12345678u64;
  let mut rand = || {
    seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (seed >> 11) as f64 / (1u64 << 53) as f64
  };
  let n = 40000;
  let mut inside = 0;
  for _ in 0..n {
    let p = Point3::new(
      mn.x + (mx.x - mn.x) * rand(),
      mn.y + (mx.y - mn.y) * rand(),
      mn.z + (mx.z - mn.z) * rand(),
    );
    if planes.iter().all(|(normal, origin)| normal.dot(p - origin) <= 1e-6) {
      inside += 1;
    }
  }
  inside as f64 / n as f64 * (mx.x - mn.x) * (mx.y - mn.y) * (mx.z - mn.z)
}

/// Regression check (enabled with `DACTYL_CHECK_HULLS=1`): the hull must contain
/// all its input points and be a closed shell.
fn check_hull(points: &[Point3], solid: &Solid) {
  let unique = dedupe_points(points);
  if !convex_contains(solid, &unique) {
    HULL_MISSES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let missing = unique
      .iter()
      .filter(|p| !convex_contains(solid, &[**p]))
      .count();
    eprintln!(
      "hull check: hull does not contain {} of {} input points (faces {})",
      missing,
      unique.len(),
      solid.boundaries().iter().map(|sh| sh.len()).sum::<usize>()
    );
  }
  for shell in solid.boundaries() {
    if shell.shell_condition() != ShellCondition::Closed {
      HULL_NOT_CLOSED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
      eprintln!("hull check: shell is not closed ({:?})", shell.shell_condition());
      break;
    }
  }
}

/// 2D convex hull of the projected (x, y) coordinates of a point set.
///
/// Andrew's monotone chain: always produces a convex polygon in CCW order and
/// handles degenerate/collinear inputs (which `convexhull3d` rejects).
pub fn hull_2d(points: Vec<Point3>) -> Vec<(f64, f64)> {
  let mut pts: Vec<(f64, f64)> = Vec::new();
  for p in &points {
    let key = ((p.x * 1e5).round(), (p.y * 1e5).round());
    pts.push((p.x, p.y));
    let _ = key;
  }
  pts.sort_by(|a, b| {
    let ka = ((a.0 * 1e5).round() as i64, (a.1 * 1e5).round() as i64);
    let kb = ((b.0 * 1e5).round() as i64, (b.1 * 1e5).round() as i64);
    ka.cmp(&kb)
  });
  pts.dedup_by(|a, b| {
    ((a.0 * 1e5).round() as i64, (a.1 * 1e5).round() as i64)
      == ((b.0 * 1e5).round() as i64, (b.1 * 1e5).round() as i64)
  });
  if pts.len() < 3 {
    return Vec::new();
  }
  let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| -> f64 {
    (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
  };
  let mut lower: Vec<(f64, f64)> = Vec::new();
  for &p in &pts {
    while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 1e-9 {
      lower.pop();
    }
    lower.push(p);
  }
  let mut upper: Vec<(f64, f64)> = Vec::new();
  for &p in pts.iter().rev() {
    while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 1e-9 {
      upper.pop();
    }
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  lower.extend(upper);
  if lower.len() < 3 {
    return Vec::new();
  }
  lower
}

/// Extrudes a closed 2D polygon from z = 0 to z = height.
pub fn prism(poly: &[(f64, f64)], height: f64) -> Option<Solid> {
  let verts: Vec<Vertex> = poly
    .iter()
    .map(|(x, y)| Vertex::new(Point3::new(*x, *y, 0.0)))
    .collect();
  let mut wire = Wire::new();
  for i in 0..verts.len() {
    wire.push_back(builder::line(&verts[i], &verts[(i + 1) % verts.len()]));
  }
  let face = builder::try_attach_plane(&[wire]).ok()?;
  Some(builder::tsweep(&face, Vector3::new(0.0, 0.0, height)))
}

/// Axis aligned box. `center` matches OpenSCAD's `center` argument.
pub fn cube(x: f64, y: f64, z: f64, center: bool) -> Shape {
  // Triangulated faces: `truck-shapeops` mis-handles quadrilateral prism faces
  // when cutting triangular hull solids, so every cube is built from triangles.
  let x0 = if center { -x / 2.0 } else { 0.0 };
  let y0 = if center { -y / 2.0 } else { 0.0 };
  let z0 = if center { -z / 2.0 } else { 0.0 };
  let (x1, y1, z1) = (x0 + x, y0 + y, z0 + z);
  let vertices = vec![
    Point3::new(x0, y0, z0),
    Point3::new(x1, y0, z0),
    Point3::new(x1, y1, z0),
    Point3::new(x0, y1, z0),
    Point3::new(x0, y0, z1),
    Point3::new(x1, y0, z1),
    Point3::new(x1, y1, z1),
    Point3::new(x0, y1, z1),
  ];
  let faces = vec![
    vec![0, 3, 2, 1],
    vec![4, 5, 6, 7],
    vec![0, 1, 5, 4],
    vec![1, 2, 6, 5],
    vec![2, 3, 7, 6],
    vec![3, 0, 4, 7],
  ];
  let solid = solid_from_faces(&vertices, &faces).expect("cube solid");
  Shape::new(solid)
}

/// Cylinder tessellated with `fn_` segments. `center` matches OpenSCAD.
pub fn cylinder(height: f64, radius: f64, fn_: usize, center: bool) -> Shape {
  let poly: Vec<(f64, f64)> = (0..fn_)
    .map(|i| {
      let angle = 2.0 * std::f64::consts::PI * (i as f64) / (fn_ as f64);
      (radius * angle.cos(), radius * angle.sin())
    })
    .collect();
  let solid = prism(&poly, height).expect("cylinder prism");
  if center {
    Shape::new(builder::translated(
      &solid,
      Vector3::new(0.0, 0.0, -height / 2.0),
    ))
  } else {
    Shape::new(solid)
  }
}

/// Tube along z from z = 0 to `height`, tessellated with `fn_` segments.
///
/// One solid whose end faces are rings (a polygon with a polygonal hole).
pub fn tube(height: f64, outer_radius: f64, inner_radius: f64, fn_: usize) -> Shape {
  let ring = |radius: f64, reverse: bool| {
    let mut verts: Vec<Vertex> = (0..fn_)
      .map(|i| {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (fn_ as f64);
        Vertex::new(Point3::new(radius * angle.cos(), radius * angle.sin(), 0.0))
      })
      .collect();
    if reverse {
      verts.reverse();
    }
    let mut wire = Wire::new();
    for i in 0..verts.len() {
      wire.push_back(builder::line(&verts[i], &verts[(i + 1) % verts.len()]));
    }
    wire
  };
  let face = builder::try_attach_plane(&[ring(outer_radius, false), ring(inner_radius, true)])
    .expect("tube face");
  Shape::new(builder::tsweep(&face, Vector3::new(0.0, 0.0, height)))
}

/// Smooth cylinder (revolved circle), z axis, `center` like OpenSCAD's cylinder.
///
/// Smooth (curved) surfaces are the most reliable input for truck booleans.
pub fn cylinder_smooth(height: f64, radius: f64, center: bool) -> Shape {
  let vertex = builder::vertex(Point3::new(0.0, -height / 2.0, radius));
  let circle = builder::rsweep(&vertex, Point3::origin(), Vector3::unit_y(), Rad(7.0));
  let disk = builder::try_attach_plane(&[circle]).expect("cylinder disk");
  let solid = builder::tsweep(&disk, Vector3::new(0.0, height, 0.0));
  // The sweep spans y = -height/2 .. height/2; rotating x by 90 degrees makes
  // that the z axis, so the cylinder is already centered at the origin.
  let mut shape = Shape::new(solid).rotate_x(90.0);
  if !center {
    shape = shape.translate_z(height / 2.0);
  }
  shape
}

/// A box that linearly tapers: square of `bottom` size at `z0`, `top` size at `z1`.
pub fn frustum(bottom: f64, z0: f64, top: f64, z1: f64) -> Option<Solid> {
  let b = bottom / 2.0;
  let t = top / 2.0;
  let vertices = vec![
    Point3::new(-b, -b, z0),
    Point3::new(b, -b, z0),
    Point3::new(b, b, z0),
    Point3::new(-b, b, z0),
    Point3::new(-t, -t, z1),
    Point3::new(t, -t, z1),
    Point3::new(t, t, z1),
    Point3::new(-t, t, z1),
  ];
  // Wound consistently: bottom reversed, top forward, sides outward.
  let faces = vec![
    vec![0, 3, 2, 1],
    vec![4, 5, 6, 7],
    vec![0, 1, 5, 4],
    vec![1, 2, 6, 5],
    vec![2, 3, 7, 6],
    vec![3, 0, 4, 7],
  ];
  solid_from_faces(&vertices, &faces)
}

// ---------------------------------------------------------------------------
// Boolean operations
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

use std::sync::atomic::{AtomicU64, AtomicUsize};
use std::sync::Mutex;

pub static UNION_COUNTER: AtomicUsize = AtomicUsize::new(0);
pub static HULL_FAILURES: AtomicUsize = AtomicUsize::new(0);
pub static HULL_POLY: AtomicUsize = AtomicUsize::new(0);
pub static HULL_TRI_FALLBACK: AtomicUsize = AtomicUsize::new(0);
pub static HULL_MISSES: AtomicUsize = AtomicUsize::new(0);
pub static HULL_NOT_CLOSED: AtomicUsize = AtomicUsize::new(0);
pub static SUB_PLAIN: AtomicUsize = AtomicUsize::new(0);
pub static SUB_TRANSLATE: AtomicUsize = AtomicUsize::new(0);
pub static SUB_ROTATE: AtomicUsize = AtomicUsize::new(0);
pub static SUB_ROTATE_MAX: AtomicU64 = AtomicU64::new(0);
pub static SUBTRACT_DROPPED: AtomicUsize = AtomicUsize::new(0);
pub static SUBTRACT_FAILURES: AtomicUsize = AtomicUsize::new(0);
pub static HULL_BBOX_LOG: Mutex<Vec<[f64; 7]>> = Mutex::new(Vec::new());
pub static HULL_POINTS_LOG: Mutex<Vec<Vec<(f64, f64, f64)>>> = Mutex::new(Vec::new());

/// Runs a boolean operation, catching panics inside `truck-shapeops` (which
/// panics when it produces a non-closed shell).
fn guarded_or(a: &Solid, b: &Solid) -> Option<Solid> {
  std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| or(a, b, BOOL_TOL)))
    .unwrap_or(None)
}

fn guarded_and(a: &Solid, b: &Solid) -> Option<Solid> {
  let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| and(a, b, BOOL_TOL)))
    .unwrap_or(None);
  // An empty result is not a successful subtraction: it means the boolean could
  // not cut the solid (or the shapes do not actually intersect).
  match result {
    Some(solid) if solid.boundaries().iter().any(|shell| !shell.is_empty()) => Some(solid),
    _ => None,
  }
}

/// Subtract `neg` (already inverted with `.not()`) from `base`.
///
/// Axis-aligned faces are the classic degeneracy for `truck-shapeops`; when the
/// plain boolean fails we retry with a tiny translation and, failing that, a
/// tiny rotation around the negative's own centre. The cut stays visually
/// identical at these epsilon magnitudes.
fn subtract_with_nudge(base: &Solid, neg: &Solid) -> Option<Solid> {
  if let Some(result) = guarded_and(base, neg) {
    SUB_PLAIN.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    return Some(result);
  }
  for eps in [1e-3, 5e-3, 2e-2, 5e-2, 1e-1, 2e-1] {
    let shifted = builder::translated(neg, Vector3::new(eps, -eps * 0.7, eps * 1.3));
    if let Some(result) = guarded_and(base, &shifted) {
      SUB_TRANSLATE.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
      return Some(result);
    }
  }
  let (mn, mx) = bounding_box(neg);
  let center = Point3::new((mn.x + mx.x) / 2.0, (mn.y + mx.y) / 2.0, (mn.z + mx.z) / 2.0);
  for deg in [0.01, 0.02, 0.05, 0.1, 0.2, 0.5] {
    let rotated = builder::rotated(neg, center, Vector3::unit_z(), Rad(deg));
    if let Some(result) = guarded_and(base, &rotated) {
      SUB_ROTATE.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
      let milli = (deg * 1000.0).round() as u64;
      let prev = SUB_ROTATE_MAX.load(std::sync::atomic::Ordering::Relaxed);
      if milli > prev {
        SUB_ROTATE_MAX.store(milli, std::sync::atomic::Ordering::Relaxed);
      }
      return Some(result);
    }
  }
  None
}

fn union_two(a: &Solid, b: &Solid) -> Solid {
  let n = UNION_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
  for (i, eps) in [1e-4, 1e-3, 5e-3, 2e-2].iter().enumerate() {
    let jx = eps * (1.0 + (n % 3) as f64);
    let jy = eps * (1.0 + ((n / 3) % 3) as f64);
    let jz = eps * (1.0 + ((n / 9 + i) % 3) as f64);
    let shifted = builder::translated(b, Vector3::new(jx, jy, jz));
    if let Some(result) = guarded_or(a, &shifted) {
      return result;
    }
  }
  if verbose() {
    eprintln!("warning: union failed, appending boundary shells");
  }
  let mut boundaries = a.boundaries().clone();
  boundaries.extend(b.boundaries().clone());
  Solid::new_unchecked(boundaries)
}

/// Axis aligned bounding box of a solid.
pub fn bounding_box(solid: &Solid) -> (Vector3, Vector3) {
  let points = solid_points(solid);
  let mut min = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
  let mut max = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
  for p in points {
    min.x = min.x.min(p.x);
    min.y = min.y.min(p.y);
    min.z = min.z.min(p.z);
    max.x = max.x.max(p.x);
    max.y = max.y.max(p.y);
    max.z = max.z.max(p.z);
  }
  (min, max)
}

/// Whether a convex solid (built of planar faces) contains every given point.
///
/// All solids produced by this module are convex hulls of their vertices, so
/// "all vertices inside the negative" means the whole solid is inside it.
pub fn convex_contains(solid: &Solid, points: &[Point3]) -> bool {
  if points.is_empty() {
    return true;
  }
  let bb = bounding_box(solid);
  let solid_points_all = solid_points(solid);
  let centroid = solid_points_all
    .iter()
    .fold(Vector3::new(0.0, 0.0, 0.0), |acc, p| acc + p.to_vec())
    / solid_points_all.len().max(1) as f64;
  let centroid = Point3::new(centroid.x, centroid.y, centroid.z);
  for p in points {
    if p.x < bb.0.x - 1e-6
      || p.x > bb.1.x + 1e-6
      || p.y < bb.0.y - 1e-6
      || p.y > bb.1.y + 1e-6
      || p.z < bb.0.z - 1e-6
      || p.z > bb.1.z + 1e-6
    {
      return false;
    }
  }
  for shell in solid.boundaries() {
    for face in shell.face_iter() {
      let surface = face.surface();
      let plane = match surface {
        Surface::Plane(plane) => plane,
        _ => return false,
      };
      let origin = plane.origin();
      let mut normal = plane.normal();
      // Outward normals point away from the solid's centre. The centroid of the
      // vertices is always inside a convex hull; the bounding-box centre is not.
      let to_center = centroid - origin;
      if normal.dot(to_center) > 0.0 {
        normal = -1.0 * normal;
      }
      for p in points {
        let d = *p - origin;
        if normal.dot(d) > 1e-5 {
          return false;
        }
      }
    }
  }
  true
}

pub fn boxes_overlap(a: &(Vector3, Vector3), b: &(Vector3, Vector3)) -> bool {
  a.0.x <= b.1.x && a.1.x >= b.0.x
    && a.0.y <= b.1.y && a.1.y >= b.0.y
    && a.0.z <= b.1.z && a.1.z >= b.0.z
}

/// Balanced-tree union of solids, to avoid the quadratic cost of a sequential fold.
pub fn union_solids(solids: &[Solid]) -> Option<Solid> {
  match solids.len() {
    0 => None,
    1 => Some(solids[0].clone()),
    _ => {
      let mut level: Vec<Solid> = solids.to_vec();
      while level.len() > 1 {
        let mut next = Vec::with_capacity(level.len() / 2 + 1);
        let mut i = 0;
        while i + 1 < level.len() {
          next.push(union_two(&level[i], &level[i + 1]));
          i += 2;
        }
        if i < level.len() {
          next.push(level[i].clone());
        }
        level = next;
      }
      Some(level.remove(0))
    }
  }
}

/// Tessellates every solid of the shape into a single polygon mesh.
///
/// Solids that overlap stay as separate shells in the mesh; slicers treat
/// overlapping closed volumes as their union.
pub fn shape_mesh(shape: &Shape) -> PolygonMesh {
  let mut mesh = PolygonMesh::default();
  for solid in &shape.solids {
    for shell in solid.boundaries() {
      // Tessellate shell by shell: tessellating a `Solid` would validate that
      // every shell is closed, which is not guaranteed after a failed union.
      let mut part = shell.triangulation(MESH_TOL).to_polygon();
      part.put_together_same_attrs(TOLERANCE);
      mesh.merge(part);
    }
  }
  mesh
}

/// Tessellates a solid into a triangulated polygon mesh and writes an STL file.
pub fn write_stl(solid: &Solid, path: &str) -> std::io::Result<()> {
  write_mesh_stl(&Shape::new(solid.clone()), path, false)
}

/// Vertices closer together than this are merged when writing an STL file.
/// Must stay well below the 0.01 mm width of the post connectors.
const WELD_TOL: f64 = 2e-3;

/// Triangles of one shell, with nearly coincident vertices merged and the
/// triangles that collapse as a result removed.
///
/// `truck-shapeops` leaves sliver faces along its cuts whose corners are a
/// micron or so apart. Left alone they tessellate into zero-area triangles and
/// into edges that do not pair up once the coordinates are rounded to `f32`,
/// which slicers report as non-manifold edges and reversed faces.
fn welded_triangles(shell: &Shell) -> Vec<[[f32; 3]; 3]> {
  let mesh = shell.triangulation(MESH_TOL).to_polygon();
  let positions = mesh.positions();
  let cell = |v: f64| (v / WELD_TOL).floor() as i64;
  let mut grid: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
  let mut welded: Vec<Point3> = Vec::new();
  let mut remap: Vec<usize> = Vec::with_capacity(positions.len());
  for p in positions {
    let (cx, cy, cz) = (cell(p.x), cell(p.y), cell(p.z));
    let mut found = None;
    'search: for dx in -1..=1 {
      for dy in -1..=1 {
        for dz in -1..=1 {
          for &i in grid.get(&(cx + dx, cy + dy, cz + dz)).into_iter().flatten() {
            if (welded[i] - *p).magnitude() <= WELD_TOL {
              found = Some(i);
              break 'search;
            }
          }
        }
      }
    }
    let index = found.unwrap_or_else(|| {
      welded.push(*p);
      grid.entry((cx, cy, cz)).or_default().push(welded.len() - 1);
      welded.len() - 1
    });
    remap.push(index);
  }

  let as_f32 = |i: usize| [welded[i].x as f32, welded[i].y as f32, welded[i].z as f32];
  let mut triangles = Vec::new();
  for face in mesh.face_iter() {
    let corners: Vec<usize> = face.iter().map(|v| remap[v.pos]).collect();
    for k in 1..corners.len().saturating_sub(1) {
      let (a, b, c) = (corners[0], corners[k], corners[k + 1]);
      let (pa, pb, pc) = (as_f32(a), as_f32(b), as_f32(c));
      if pa == pb || pb == pc || pa == pc {
        continue;
      }
      triangles.push([pa, pb, pc]);
    }
  }
  triangles
}

/// Tessellates a shape (union of solids) and writes a binary STL file.
///
/// Solids that overlap stay as separate closed shells; slicers treat
/// overlapping closed volumes as their union.
///
/// With `mirror` the mesh is mirrored in x. Mirroring the finished triangles
/// (rather than tessellating a mirrored shape) keeps both halves identical.
pub fn write_mesh_stl(shape: &Shape, path: &str, mirror: bool) -> std::io::Result<()> {
  use std::io::Write;
  let mut triangles = Vec::new();
  for solid in &shape.solids {
    for shell in solid.boundaries() {
      triangles.extend(welded_triangles(shell));
    }
  }
  if mirror {
    for [a, b, c] in triangles.iter_mut() {
      a[0] = -a[0];
      b[0] = -b[0];
      c[0] = -c[0];
      std::mem::swap(b, c);
    }
  }
  let mut file = std::io::BufWriter::new(std::fs::File::create(path)?);
  file.write_all(&[0u8; 80])?;
  file.write_all(&(triangles.len() as u32).to_le_bytes())?;
  for [a, b, c] in &triangles {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
      u[1] * v[2] - u[2] * v[1],
      u[2] * v[0] - u[0] * v[2],
      u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    let n = if len > 0.0 { [n[0] / len, n[1] / len, n[2] / len] } else { [0.0; 3] };
    for value in n.iter().chain(a).chain(b).chain(c) {
      file.write_all(&value.to_le_bytes())?;
    }
    file.write_all(&[0u8; 2])?;
  }
  file.flush()
}

// ---------------------------------------------------------------------------
// Transforms (OpenSCAD-like transform lists)
// ---------------------------------------------------------------------------

/// A rotation and translation. Rotations are applied in z, x, y order before
/// the translation, matching the original OpenSCAD code.
#[derive(Clone, Copy, Debug, Default)]
pub struct Transform {
  pub x: f64,
  pub y: f64,
  pub z: f64,
  pub rx: f64,
  pub ry: f64,
  pub rz: f64,
}

impl Transform {
  pub fn translation(x: f64, y: f64, z: f64) -> Self {
    Transform {
      x,
      y,
      z,
      ..Default::default()
    }
  }

  pub fn rotation(rx: f64, ry: f64, rz: f64) -> Self {
    Transform {
      rx,
      ry,
      rz,
      ..Default::default()
    }
  }

  pub fn matrix(&self) -> Matrix4 {
    let mut mat = Matrix4::from_translation(Vector3::new(self.x, self.y, self.z));
    mat = mat * Matrix4::from_angle_y(Rad(self.ry.to_radians()));
    mat = mat * Matrix4::from_angle_x(Rad(self.rx.to_radians()));
    mat = mat * Matrix4::from_angle_z(Rad(self.rz.to_radians()));
    mat
  }

  #[allow(dead_code)]
  pub fn apply_point(&self, p: Point3) -> Point3 {
    transform_point(&self.matrix(), p)
  }
}

/// Applies a matrix to a point (cgmath `Transform::transform_point` without the trait import).
pub fn transform_point(mat: &Matrix4, p: Point3) -> Point3 {
  let v = *mat * Vector4::new(p.x, p.y, p.z, 1.0);
  Point3::new(v.x, v.y, v.z)
}

#[derive(Clone, Debug, Default)]
pub struct TransformList {
  transforms: Vec<Transform>,
}

impl TransformList {
  pub fn empty() -> Self {
    TransformList::default()
  }

  #[allow(dead_code)]
  pub fn is_empty(&self) -> bool {
    self.transforms.is_empty()
  }

  pub fn add_transform(&mut self, t: Transform) -> &mut Transform {
    self.transforms.push(t);
    let len = self.transforms.len();
    &mut self.transforms[len - 1]
  }

  pub fn add_transform_front(&mut self, t: Transform) -> &mut Transform {
    self.transforms.insert(0, t);
    &mut self.transforms[0]
  }

  pub fn mutable_front(&mut self) -> &mut Transform {
    if self.transforms.is_empty() {
      self.transforms.push(Transform::default());
    }
    &mut self.transforms[0]
  }

  pub fn rotate_x(&mut self, deg: f64) -> &mut Self {
    let t = self.add_transform(Transform::default());
    t.rx = deg;
    self
  }

  pub fn rotate_y(&mut self, deg: f64) -> &mut Self {
    let t = self.add_transform(Transform::default());
    t.ry = deg;
    self
  }

  #[allow(dead_code)]
  pub fn rotate_z(&mut self, deg: f64) -> &mut Self {
    let t = self.add_transform(Transform::default());
    t.rz = deg;
    self
  }

  pub fn rotate_front(&mut self, rx: f64, ry: f64, rz: f64) -> &mut Self {
    self.add_transform_front(Transform::rotation(rx, ry, rz));
    self
  }

  pub fn translate(&mut self, x: f64, y: f64, z: f64) -> &mut Self {
    self.add_transform(Transform::translation(x, y, z));
    self
  }

  pub fn translate_front(&mut self, x: f64, y: f64, z: f64) -> &mut Self {
    self.add_transform_front(Transform::translation(x, y, z));
    self
  }

  pub fn translate_z(&mut self, z: f64) -> &mut Self {
    self.translate(0.0, 0.0, z)
  }

  pub fn append(&mut self, other: &TransformList) -> &mut Self {
    self.transforms.extend(other.transforms.iter().copied());
    self
  }

  pub fn append_front(&mut self, other: &TransformList) -> &mut Self {
    let mut merged = other.transforms.clone();
    merged.extend(self.transforms.iter().copied());
    self.transforms = merged;
    self
  }

  /// The composed matrix (transforms applied in order).
  pub fn matrix(&self) -> Matrix4 {
    let mut mat = Matrix4::from_scale(1.0);
    for t in &self.transforms {
      mat = t.matrix() * mat;
    }
    mat
  }

  pub fn apply_shape(&self, shape: &Shape) -> Shape {
    shape.transform(self.matrix())
  }

  pub fn apply_point(&self, p: Point3) -> Point3 {
    transform_point(&self.matrix(), p)
  }
}

// ---------------------------------------------------------------------------
// 2D union of convex polygons (used for the bottom plate shadow)
// ---------------------------------------------------------------------------

fn point_in_polygon(poly: &[(f64, f64)], x: f64, y: f64) -> bool {
  let mut inside = false;
  let n = poly.len();
  for i in 0..n {
    let (x0, y0) = poly[i];
    let (x1, y1) = poly[(i + 1) % n];
    if (y0 > y) != (y1 > y) {
      let t = (y - y0) / (y1 - y0);
      if x < x0 + t * (x1 - x0) {
        inside = !inside;
      }
    }
  }
  inside
}

fn polygon_area(poly: &[(f64, f64)]) -> f64 {
  let n = poly.len();
  let mut area = 0.0;
  for i in 0..n {
    let (x0, y0) = poly[i];
    let (x1, y1) = poly[(i + 1) % n];
    area += x0 * y1 - x1 * y0;
  }
  area * 0.5
}

/// Douglas-Peucker polygon simplification (the polygon is treated as closed).
fn simplify_closed_polygon(poly: &[(f64, f64)], tol: f64) -> Vec<(f64, f64)> {
  if poly.len() < 4 {
    return poly.to_vec();
  }
  // Split the cycle at the farthest vertex from the start to get two paths.
  let (x0, y0) = poly[0];
  let mut far = 0;
  let mut far_d = -1.0;
  for (i, (x, y)) in poly.iter().enumerate() {
    let d = (x - x0) * (x - x0) + (y - y0) * (y - y0);
    if d > far_d {
      far_d = d;
      far = i;
    }
  }
  let path1: Vec<(f64, f64)> = poly[..=far].to_vec();
  let mut path2: Vec<(f64, f64)> = poly[far..].to_vec();
  path2.push(poly[0]);

  fn dp(points: &[(f64, f64)], tol: f64, keep: &mut Vec<bool>) {
    let mut stack = vec![(0usize, points.len() - 1)];
    while let Some((a, b)) = stack.pop() {
      if b <= a + 1 {
        continue;
      }
      let (ax, ay) = points[a];
      let (bx, by) = points[b];
      let dx = bx - ax;
      let dy = by - ay;
      let len2 = dx * dx + dy * dy;
      let mut best = 0.0;
      let mut idx = a;
      for i in a + 1..b {
        let (px, py) = points[i];
        let d = if len2 == 0.0 {
          (px - ax) * (px - ax) + (py - ay) * (py - ay)
        } else {
          let t = ((px - ax) * dx + (py - ay) * dy) / len2;
          let t = t.clamp(0.0, 1.0);
          let qx = ax + t * dx;
          let qy = ay + t * dy;
          (px - qx) * (px - qx) + (py - qy) * (py - qy)
        };
        if d > best {
          best = d;
          idx = i;
        }
      }
      if best.sqrt() > tol {
        keep[idx] = true;
        stack.push((a, idx));
        stack.push((idx, b));
      }
    }
  }

  let mut keep1 = vec![false; path1.len()];
  let mut keep2 = vec![false; path2.len()];
  keep1[0] = true;
  keep1[path1.len() - 1] = true;
  keep2[0] = true;
  keep2[path2.len() - 1] = true;
  dp(&path1, tol, &mut keep1);
  dp(&path2, tol, &mut keep2);

  let mut out: Vec<(f64, f64)> = Vec::new();
  for (i, p) in path1.iter().enumerate() {
    if keep1[i] {
      out.push(*p);
    }
  }
  for (i, p) in path2.iter().enumerate().skip(1) {
    if keep2[i] && i != path2.len() - 1 {
      out.push(*p);
    }
  }
  out
}

/// Exact union boundary of a set of convex 2D polygons.
///
/// The region is rasterized and its boundary is traced with marching squares;
/// closed loops are simplified with Douglas-Peucker. Only the outer boundaries
/// are returned (holes are cut later, e.g. by the screw holes).
pub fn union_polygons_2d(polys: &[Vec<(f64, f64)>], res: f64) -> Vec<Vec<(f64, f64)>> {
  if polys.is_empty() {
    return Vec::new();
  }
  let mut min_x = f64::INFINITY;
  let mut min_y = f64::INFINITY;
  let mut max_x = f64::NEG_INFINITY;
  let mut max_y = f64::NEG_INFINITY;
  for poly in polys {
    for (x, y) in poly {
      min_x = min_x.min(*x);
      min_y = min_y.min(*y);
      max_x = max_x.max(*x);
      max_y = max_y.max(*y);
    }
  }
  let w = ((max_x - min_x) / res).ceil() as usize + 2;
  let h = ((max_y - min_y) / res).ceil() as usize + 2;
  let origin_x = min_x - res;
  let origin_y = min_y - res;

  let mut mask = vec![0u8; w * h];
  for poly in polys {
    if poly.len() < 3 {
      continue;
    }
    let mut pmin_x = f64::INFINITY;
    let mut pmin_y = f64::INFINITY;
    let mut pmax_x = f64::NEG_INFINITY;
    let mut pmax_y = f64::NEG_INFINITY;
    for (x, y) in poly {
      pmin_x = pmin_x.min(*x);
      pmin_y = pmin_y.min(*y);
      pmax_x = pmax_x.max(*x);
      pmax_y = pmax_y.max(*y);
    }
    let x0 = (((pmin_x - origin_x) / res).floor() as isize).max(0) as usize;
    let y0 = (((pmin_y - origin_y) / res).floor() as isize).max(0) as usize;
    let x1 = (((pmax_x - origin_x) / res).ceil() as usize + 1).min(w - 1);
    let y1 = (((pmax_y - origin_y) / res).ceil() as usize + 1).min(h - 1);
    for gy in y0..=y1 {
      for gx in x0..=x1 {
        let idx = gy * w + gx;
        if mask[idx] == 1 {
          continue;
        }
        let x = origin_x + (gx as f64 + 0.5) * res;
        let y = origin_y + (gy as f64 + 0.5) * res;
        if point_in_polygon(poly, x, y) {
          mask[idx] = 1;
        }
      }
    }
  }


  // Marching squares: collect oriented boundary edges as grid-corner points.
  let mut outgoing: HashMap<(i32, i32), Vec<(i32, i32)>> = HashMap::new();
  let add_edge = |outgoing: &mut HashMap<(i32, i32), Vec<(i32, i32)>>,
                  a: (i32, i32),
                  b: (i32, i32)| {
    outgoing.entry(a).or_default().push(b);
  };
  for gy in 0..h {
    for gx in 0..w {
      if mask[gy * w + gx] == 0 {
        continue;
      }
      let x = gx as i32;
      let y = gy as i32;
      let filled = |cx: isize, cy: isize| -> bool {
        if cx < 0 || cy < 0 || cx >= w as isize || cy >= h as isize {
          return false;
        }
        mask[cy as usize * w + cx as usize] == 1
      };
      if !filled(x as isize - 1, y as isize) {
        add_edge(&mut outgoing, (x, y), (x, y + 1));
      }
      if !filled(x as isize + 1, y as isize) {
        add_edge(&mut outgoing, (x + 1, y + 1), (x + 1, y));
      }
      if !filled(x as isize, y as isize - 1) {
        add_edge(&mut outgoing, (x + 1, y), (x, y));
      }
      if !filled(x as isize, y as isize + 1) {
        add_edge(&mut outgoing, (x, y + 1), (x + 1, y + 1));
      }
    }
  }

  // Stitch edges into loops.
  let mut loops: Vec<Vec<(f64, f64)>> = Vec::new();
  while !outgoing.is_empty() {
    let start = *outgoing.keys().next().unwrap();
    let mut current = start;
    let mut grid_loop: Vec<(i32, i32)> = Vec::new();
    loop {
      let Some(nexts) = outgoing.get_mut(&current) else {
        break;
      };
      if nexts.is_empty() {
        outgoing.remove(&current);
        break;
      }
      let next = nexts.remove(0);
      if nexts.is_empty() {
        outgoing.remove(&current);
      }
      grid_loop.push(current);
      current = next;
      if current == start {
        break;
      }
    }
    if grid_loop.len() < 4 {
      continue;
    }
    let poly: Vec<(f64, f64)> = grid_loop
      .iter()
      .map(|(ix, iy)| {
        (
          origin_x + *ix as f64 * res,
          origin_y + *iy as f64 * res,
        )
      })
      .collect();
    // Marching squares yields clockwise loops for outer boundaries and
    // counter-clockwise loops for holes. Holes are cut later (by the screw
    // holes and the thumb plate cut), so only keep the outer boundaries.
    let area = polygon_area(&poly);
    if area >= 0.0 {
      continue;
    }
    let mut poly = poly;
    poly.reverse();
    let simplified = simplify_closed_polygon(&poly, res * 0.7);
    if simplified.len() >= 3 {
      loops.push(simplified);
    }
  }
  loops
}
