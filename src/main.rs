//! dactyl-cc case generator, ported from src/dactyl.cc to the Truck CAD kernel.
//!
//! Instead of writing OpenSCAD source, this program builds the model as solids
//! and writes STL files directly:
//!
//! * `left.stl` / `right.stl` – the two halves of the case
//! * `bottom_left.stl` / `bottom_right.stl` – the bottom plates
//!
//! Run with `cargo run --release`.

mod geom;
mod key;
mod key_data;

use geom::*;
use key::*;
use key_data::{KeyData, KeyId};
use truck_modeling::{Point3, Solid};

/// Add the caps into the output for testing.
const ADD_CAPS: bool = false;
const WRITE_TEST_KEYS: bool = false;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Direction {
  Up,
  Down,
  Left,
  Right,
}

struct WallPoint {
  transforms: TransformList,
  out_direction: Direction,
  extra_distance: f64,
  extra_width: f64,
}

fn wall_point(
  transforms: TransformList,
  out_direction: Direction,
  extra_distance: f64,
  extra_width: f64,
) -> WallPoint {
  WallPoint {
    transforms,
    out_direction,
    extra_distance,
    extra_width,
  }
}

fn main() {
  install_panic_hook();
  let args: Vec<String> = std::env::args().collect();
  if args.iter().any(|a| a == "--sanity") {
    sanity();
    return;
  }
  let dump_hulls = args.iter().any(|a| a == "--dump-hulls");
  let dump_solids = args.iter().any(|a| a == "--dump-solids");
  // `--out DIR` writes the STL files into DIR (default: current directory).
  let out_dir = match args.iter().position(|a| a == "--out") {
    Some(i) if i + 1 < args.len() => args[i + 1].clone(),
    _ => ".".to_string(),
  };
  std::fs::create_dir_all(&out_dir).ok();
  println!("generating.. (output: {})", out_dir);

  let mut key_origin = TransformList::empty();
  key_origin.translate(-20.0, -40.0, 3.0);

  // This is where all of the logic to position the keys is done. Everything
  // below is cosmetic, trying to build the case.
  let mut d = KeyData::new(&key_origin);

  if WRITE_TEST_KEYS {
    let test_ids = [KeyId::Three, KeyId::E, KeyId::Four, KeyId::Five, KeyId::D];
    let mut test_shapes = Vec::new();
    for id in test_ids {
      let key = d.key_mut(id);
      key.add_side_nub = false;
      key.extra_z = 4.0;
      test_shapes.push(key.get_switch());
      if ADD_CAPS {
        test_shapes.push(key.get_cap(false));
      }
    }
    Shape::union_all(test_shapes)
      .collapse()
      .map(|solid| write_stl(&solid, "test_keys.stl"))
      .transpose()
      .unwrap();
    return;
  }

  // Set all of the widths here. This must be done before calling any of
  // GetTopLeft etc.

  d.key_mut(KeyId::Backspace).extra_width_bottom = 11.0;
  d.key_mut(KeyId::Backspace).extra_width_left = 3.0;
  d.key_mut(KeyId::Delete).extra_width_bottom = 11.0;
  d.key_mut(KeyId::End).extra_width_bottom = 3.0;
  d.key_mut(KeyId::Ctrl).extra_width_top = 3.0;
  d.key_mut(KeyId::Alt).extra_width_top = 3.0;
  d.key_mut(KeyId::Alt).extra_width_right = 3.0;
  d.key_mut(KeyId::Alt).extra_width_left = 3.0;
  d.key_mut(KeyId::Home).extra_width_right = 3.0;
  d.key_mut(KeyId::Home).extra_width_left = 3.0;
  d.key_mut(KeyId::Home).extra_width_top = 3.0;
  d.key_mut(KeyId::End).extra_width_top = 3.0;
  d.key_mut(KeyId::End).extra_width_right = 3.0;
  d.key_mut(KeyId::End).extra_width_left = 3.0;

  // left wall
  for id in d.grid_column_ids(0).into_iter().flatten() {
    d.key_mut(id).extra_width_left = 4.0;
  }

  d.key_mut(KeyId::Five).extra_width_right = 4.0;
  d.key_mut(KeyId::T).extra_width_right = 4.0;
  d.key_mut(KeyId::G).extra_width_right = 4.0;

  // top row
  for id in d.grid_row_ids(0).into_iter().flatten() {
    d.key_mut(id).extra_width_top = 2.0;
  }
  d.key_mut(KeyId::B).extra_width_bottom = 3.0;

  let mut shapes: Vec<Shape> = Vec::new();

  //
  // Thumb plate
  //

  let connector = get_post_connector();
  {
    let ctrl = d.key(KeyId::Ctrl);
    let alt = d.key(KeyId::Alt);
    let backspace = d.key(KeyId::Backspace);
    let delete = d.key(KeyId::Delete);
    let end = d.key(KeyId::End);

    shapes.push(Shape::union_all(vec![
      connect_horizontal(ctrl, alt, &connector, 0.0),
      connect_horizontal(backspace, delete, &connector, 0.0),
      connect_vertical(ctrl, delete, &connector, 0.0),
      tri(
        &end.get_bottom_left(0.0),
        &delete.get_bottom_right(0.0),
        &backspace.get_bottom_left(0.0),
        &connector,
      ),
    ]));
  }

  shapes.push(connect_main_keys(&d));

  shapes.push(tri_fan(
    &d.key(KeyId::Ctrl).get_top_left(0.0),
    &[
      d.key(KeyId::B).get_bottom_right(0.0),
      d.key(KeyId::B).get_top_right(0.0),
      d.key(KeyId::G).get_bottom_right(0.0),
    ],
    &connector,
  ));

  // These transforms with translate_front are moving the connectors down in
  // the z direction to reduce the vertical jumps.
  let mut slash_bottom_right = d.key(KeyId::Slash).get_bottom_right(0.0);
  slash_bottom_right.translate_front(0.0, -5.0, -3.0);

  let mut left_arrow_br_1 = d.key(KeyId::LeftArrow).get_bottom_right(0.0);
  left_arrow_br_1.translate_front(0.0, 0.0, -1.0);

  let mut slash_br_1 = d.key(KeyId::Slash).get_bottom_right(0.0);
  slash_br_1.translate_front(0.0, 0.0, -1.0);

  let mut right_arrow_bl_1 = d.key(KeyId::RightArrow).get_bottom_left(0.0);
  right_arrow_bl_1.translate_front(0.0, 0.0, -1.0);

  shapes.push(tri_fan(
    &slash_bottom_right,
    &[
      left_arrow_br_1.clone(),
      d.key(KeyId::LeftArrow).get_bottom_left(0.0),
      slash_br_1.clone(),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &d.key(KeyId::Backspace).get_bottom_left(0.0),
    &[
      slash_bottom_right.clone(),
      left_arrow_br_1.clone(),
      right_arrow_bl_1.clone(),
      d.key(KeyId::RightArrow).get_bottom_right(0.0),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &d.key(KeyId::Tilde).get_bottom_right(0.0),
    &[
      d.key(KeyId::Slash).get_bottom_left(0.0),
      slash_br_1.clone(),
      slash_bottom_right.clone(),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &d.key(KeyId::Delete).get_top_left(0.0),
    &[
      d.key(KeyId::Ctrl).get_top_left(0.0),
      d.key(KeyId::B).get_bottom_right(0.0),
      d.key(KeyId::Backspace).get_top_left(0.0),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &d.key(KeyId::B).get_bottom_left(0.0),
    &[
      d.key(KeyId::B).get_bottom_right(0.0),
      d.key(KeyId::Backspace).get_top_left(0.0),
      d.key(KeyId::Backspace).get_top_left(0.0),
      d.key(KeyId::RightArrow).get_bottom_right(0.0),
      d.key(KeyId::RightArrow).get_top_right(0.0),
      d.key(KeyId::V).get_bottom_right(0.0),
    ],
    &connector,
  ));

  // Bottom right corner.
  shapes.push(tri_fan(
    &d.key(KeyId::Shift).get_bottom_right(0.0),
    &[
      d.key(KeyId::Z).get_bottom_left(0.0),
      d.key(KeyId::Tilde).get_top_left(0.0),
      d.key(KeyId::Tilde).get_bottom_left(0.0),
      d.key(KeyId::Shift).get_bottom_left(0.0),
    ],
    &connector,
  ));

  // Connecting top wall to keys
  let mut key_plus_top_right_wall = d.key(KeyId::Plus).get_top_right(0.0);
  key_plus_top_right_wall.translate_front(0.0, 3.0, -3.0);
  let mut key_2_top_left_wall = d.key(KeyId::Two).get_top_left(0.0);
  key_2_top_left_wall.translate_front(0.0, 3.75, 0.0);
  let mut key_2_top_right_wall = d.key(KeyId::Two).get_top_right(0.0);
  key_2_top_right_wall.translate_front(0.0, 4.0, -1.0);
  let mut key_3_top_right_wall = d.key(KeyId::Three).get_top_right(0.0);
  key_3_top_right_wall.translate_front(0.0, 3.5, 0.0);
  let mut key_4_top_right_wall = d.key(KeyId::Four).get_top_right(0.0);
  key_4_top_right_wall.translate_front(0.0, 2.2, 0.0);

  shapes.push(tri_fan(
    &key_4_top_right_wall,
    &[
      d.key(KeyId::Five).get_top_right(0.0),
      d.key(KeyId::Five).get_top_left(0.0),
      d.key(KeyId::Four).get_top_right(0.0),
      d.key(KeyId::Four).get_top_left(0.0),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &key_3_top_right_wall,
    &[
      key_4_top_right_wall.clone(),
      d.key(KeyId::Four).get_top_left(0.0),
      d.key(KeyId::Three).get_top_right(0.0),
      d.key(KeyId::Three).get_top_left(0.0),
      key_2_top_right_wall.clone(),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &key_2_top_right_wall,
    &[
      key_2_top_left_wall.clone(),
      d.key(KeyId::Two).get_top_right(0.0),
      d.key(KeyId::Three).get_top_left(0.0),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &key_2_top_left_wall,
    &[
      d.key(KeyId::One).get_top_right(0.0),
      d.key(KeyId::Two).get_top_left(0.0),
      d.key(KeyId::Two).get_top_right(0.0),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &d.key(KeyId::Plus).get_top_right(0.0),
    &[
      d.key(KeyId::One).get_top_left(0.0),
      d.key(KeyId::One).get_top_right(0.0),
      key_2_top_left_wall.clone(),
    ],
    &connector,
  ));
  shapes.push(tri_fan(
    &key_plus_top_right_wall,
    &[
      key_2_top_left_wall.clone(),
      d.key(KeyId::Plus).get_top_right(0.0),
      d.key(KeyId::Plus).get_top_left(0.0),
    ],
    &connector,
  ));

  //
  // Make the wall
  //
  {
    let mut ctrl_tl_rotated = d.key(KeyId::Ctrl).get_top_left(0.0);
    ctrl_tl_rotated.rotate_front(0.0, 0.0, -15.0);

    let wall_points: Vec<WallPoint> = vec![
      // Start top left and go clockwise
      wall_point(d.key(KeyId::Plus).get_top_left(0.0), Direction::Up, 0.0, 0.0),
      wall_point(key_plus_top_right_wall.clone(), Direction::Up, 0.0, 0.3),
      wall_point(key_2_top_left_wall.clone(), Direction::Up, 0.0, 0.3),
      wall_point(key_2_top_right_wall.clone(), Direction::Up, 0.0, 0.0),
      wall_point(key_3_top_right_wall.clone(), Direction::Up, 0.0, 0.0),
      wall_point(key_4_top_right_wall.clone(), Direction::Up, 0.0, 0.0),
      wall_point(d.key(KeyId::Five).get_top_right(0.0), Direction::Up, 0.0, 0.0),
      wall_point(d.key(KeyId::Five).get_top_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::Five).get_bottom_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::T).get_top_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::T).get_bottom_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::G).get_top_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::G).get_bottom_right(0.0), Direction::Right, 1.0, 0.5),
      wall_point(ctrl_tl_rotated.clone(), Direction::Up, 1.0, 0.5),
      wall_point(d.key(KeyId::Ctrl).get_top_right(0.0), Direction::Up, 0.0, 0.0),
      wall_point(d.key(KeyId::Alt).get_top_left(0.0), Direction::Up, 0.0, 0.0),
      wall_point(d.key(KeyId::Alt).get_top_right(0.0), Direction::Up, 0.0, 0.5),
      wall_point(d.key(KeyId::Alt).get_top_right(0.0), Direction::Right, 0.0, 0.5),
      wall_point(d.key(KeyId::Alt).get_bottom_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::Home).get_top_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::Home).get_bottom_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::End).get_top_right(0.0), Direction::Right, 0.0, 0.0),
      wall_point(d.key(KeyId::End).get_bottom_right(0.0), Direction::Right, 0.0, 0.5),
      wall_point(d.key(KeyId::End).get_bottom_right(0.0), Direction::Down, 0.0, 0.5),
      wall_point(d.key(KeyId::End).get_bottom_left(0.0), Direction::Down, 0.0, 0.0),
      wall_point(d.key(KeyId::Backspace).get_bottom_left(0.0), Direction::Down, 0.0, 0.0),
      wall_point(slash_bottom_right.clone(), Direction::Down, 0.0, 0.0),
      wall_point(d.key(KeyId::Tilde).get_bottom_right(0.0), Direction::Down, 0.0, 0.0),
      wall_point(d.key(KeyId::Tilde).get_bottom_left(0.0), Direction::Down, 0.0, 0.0),
      wall_point(d.key(KeyId::Shift).get_bottom_left(0.0), Direction::Down, 0.0, 0.75),
      wall_point(d.key(KeyId::Shift).get_bottom_left(0.0), Direction::Left, 0.0, 0.5),
      wall_point(d.key(KeyId::Shift).get_top_left(0.0), Direction::Left, 0.0, 0.5),
      wall_point(d.key(KeyId::Caps).get_bottom_left(0.0), Direction::Left, 0.0, 0.0),
      wall_point(d.key(KeyId::Caps).get_top_left(0.0), Direction::Left, 0.0, 0.0),
      wall_point(d.key(KeyId::Tab).get_bottom_left(0.0), Direction::Left, 0.0, 0.0),
      wall_point(d.key(KeyId::Tab).get_top_left(0.0), Direction::Left, 0.0, 0.0),
      wall_point(d.key(KeyId::Plus).get_bottom_left(0.0), Direction::Left, 0.0, 0.0),
      wall_point(d.key(KeyId::Plus).get_top_left(0.0), Direction::Left, 0.0, 0.0),
    ];

    let mut wall_slices: Vec<Vec<Shape>> = Vec::new();
    for point in wall_points {
      let s1 = point.transforms.apply_shape(&connector);

      let mut t = point.transforms.clone();
      let distance = 4.8 + point.extra_distance;
      match point.out_direction {
        Direction::Up => {
          let mut extra = TransformList::empty();
          extra.translate(0.0, distance, 0.0).rotate_x(-20.0);
          t.append_front(&extra);
        }
        Direction::Down => {
          let mut extra = TransformList::empty();
          extra.translate(0.0, -1.0 * distance, 0.0).rotate_x(20.0);
          t.append_front(&extra);
        }
        Direction::Left => {
          let mut extra = TransformList::empty();
          extra.translate(-1.0 * distance, 0.0, 0.0).rotate_y(-20.0);
          t.append_front(&extra);
        }
        Direction::Right => {
          let mut extra = TransformList::empty();
          extra.translate(distance, 0.0, 0.0).rotate_y(20.0);
          t.append_front(&extra);
        }
      }

      // Make sure the section extruded to the bottom is thick enough. With
      // certain angles the projection is very small if you just use the post
      // connector from the transform. Compute an explicit shape.
      let post_offset = Point3::new(0.0, 0.0, -4.0);
      let p = point.transforms.apply_point(post_offset);
      let p2 = t.apply_point(post_offset);

      // Direction back towards the key, projected onto the xy plane.
      let out_x = p2.x - p.x;
      let out_y = p2.y - p.y;
      let len = (out_x * out_x + out_y * out_y).sqrt();
      let (in_x, in_y) = if len > 1e-9 {
        (-out_x / len, -out_y / len)
      } else {
        (0.0, 0.0)
      };

      let width = 3.3 + point.extra_width;
      let s2 = Shape::hull(&[
        cube(0.1, 0.1, 0.1, true).translate(p2.x, p2.y, p2.z),
        cube(0.1, 0.1, 0.1, true).translate(p2.x + width * in_x, p2.y + width * in_y, p2.z),
      ]);

      let mut slice = Vec::new();
      slice.push(Shape::hull(&[s1.clone(), s2.clone()]));
      slice.push(Shape::hull(&[
        s2.clone(),
        s2.project_extrude(0.1, true).translate_z(0.05),
      ]));

      wall_slices.push(slice);
    }

    for i in 0..wall_slices.len() {
      let slice = &wall_slices[i];
      let next_slice = &wall_slices[(i + 1) % wall_slices.len()];
      for j in 0..slice.len() {
        shapes.push(Shape::hull(&[slice[j].clone(), next_slice[j].clone()]));
      }
    }
  }

  for key in d.all_keys() {
    shapes.push(key.get_switch());
    if ADD_CAPS {
      shapes.push(key.get_cap(false));
    }
  }

  // Add all the screw inserts.
  let screw_height = 10.0;
  let screw_radius = 4.4 / 2.0;
  let screw_holes: Vec<Shape>;
  let screw_positions: Vec<Point3>;
  {
    let screw_hole = cylinder_smooth(screw_height + 5.0, screw_radius, true);
    let screw_insert =
      cylinder(screw_height, screw_radius + 1.65, 30, true).translate_z(screw_height / 2.0);

    let origin = Point3::new(0.0, 0.0, 0.0);

    let mut screw_left_bottom = d.key(KeyId::Shift).get_bottom_left(0.0).apply_point(origin);
    screw_left_bottom.z = 0.0;
    screw_left_bottom.x += 3.2;

    let mut screw_left_top = d.key(KeyId::Plus).get_top_left(0.0).apply_point(origin);
    screw_left_top.z = 0.0;
    screw_left_top.x += 2.8;
    screw_left_top.y += -0.5;

    let mut screw_right_top = d.key(KeyId::Five).get_top_right(0.0).apply_point(origin);
    screw_right_top.z = 0.0;
    screw_right_top.x += 4.0;
    screw_right_top.y += -15.5;

    let mut screw_right_bottom = d.key(KeyId::End).get_bottom_left(0.0).apply_point(origin);
    screw_right_bottom.z = 0.0;
    screw_right_bottom.y += 3.5;
    screw_right_bottom.x += 1.5;

    let mut screw_right_mid = d.key(KeyId::Ctrl).get_top_left(0.0).apply_point(origin);
    screw_right_mid.z = 0.0;
    screw_right_mid.y += -0.9;

    let positions = [
      screw_left_top,
      screw_right_top,
      screw_right_mid,
      screw_right_bottom,
      screw_left_bottom,
    ];
    if geom::verbose() {
      for (i, p) in positions.iter().enumerate() {
        println!("screw {} at {:.2}, {:.2}, {:.2}", i, p.x, p.y, p.z);
      }
      for key in d.all_keys() {
        let m = key.get_middle().apply_point(Point3::new(0.0, 0.0, 0.0));
        println!("key {} center {:.2}, {:.2}, {:.2}", key.name, m.x, m.y, m.z);
      }
    }

    shapes.push(Shape::union_all(
      positions
        .iter()
        .map(|p| screw_insert.translate(p.x, p.y, p.z))
        .collect(),
    ));
    screw_positions = positions.to_vec();
    screw_holes = positions
      .iter()
      .map(|p| screw_hole.translate(p.x, p.y, p.z))
      .collect();
  }

  let mut negative_shapes: Vec<Shape> = Vec::new();
  negative_shapes.extend(screw_holes.iter().cloned());
  // Cut off the parts sticking up into the thumb plate.
  negative_shapes.push(
    d.key(KeyId::Backspace)
      .get_top_left(0.0)
      .apply_shape(&cube(50.0, 50.0, 6.0, true).translate_z(3.0)),
  );

  println!("unioning {} shapes...", shapes.len());
  let case_shapes = Shape::union_all(shapes);

  if geom::verbose() {
    let p = Point3::new(64.04, -13.81, 5.0);
    let mut hits = 0;
    for solid in &case_shapes.solids {
      if convex_contains(solid, &[p]) {
        hits += 1;
        let (mn, mx) = bounding_box(solid);
        println!(
          "pre-subtract probe inside solid bbox {:.1},{:.1},{:.1}..{:.1},{:.1},{:.1} faces {}",
          mn.x, mn.y, mn.z, mx.x, mx.y, mx.z,
          solid.boundaries().iter().map(|sh| sh.len()).sum::<usize>()
        );
      }
    }
    println!("pre-subtract probe solids: {}", hits);
    for (i, neg) in negative_shapes.iter().enumerate() {
      for solid in &neg.solids {
        let (mn, mx) = bounding_box(solid);
        println!("negative {} bbox {:.1},{:.1},{:.1}..{:.1},{:.1},{:.1}", i, mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
      }
    }
  }
  println!("subtracting {} shapes...", negative_shapes.len());
  let result = case_shapes.subtract(&Shape::union_all(negative_shapes.clone()));
  println!(
    "subtractions: {} plain, {} translated, {} rotated (max angle {:.2} deg)",
    geom::SUB_PLAIN.load(std::sync::atomic::Ordering::Relaxed),
    geom::SUB_TRANSLATE.load(std::sync::atomic::Ordering::Relaxed),
    geom::SUB_ROTATE.load(std::sync::atomic::Ordering::Relaxed),
    geom::SUB_ROTATE_MAX.load(std::sync::atomic::Ordering::Relaxed) as f64 / 1000.0
  );
  println!(
    "hulls: {} polygonal, {} triangle fallback",
    geom::HULL_POLY.load(std::sync::atomic::Ordering::Relaxed),
    geom::HULL_TRI_FALLBACK.load(std::sync::atomic::Ordering::Relaxed)
  );
  println!(
    "hulls missing input points: {}, solids dropped by containment: {}",
    geom::HULL_MISSES.load(std::sync::atomic::Ordering::Relaxed),
    geom::SUBTRACT_DROPPED.load(std::sync::atomic::Ordering::Relaxed)
  );
  let hull_failures = geom::HULL_FAILURES.load(std::sync::atomic::Ordering::Relaxed);
  let sub_failures = geom::SUBTRACT_FAILURES.load(std::sync::atomic::Ordering::Relaxed);
  if hull_failures > 0 || sub_failures > 0 {
    println!(
      "warning: {} hull(s) failed, {} subtraction(s) failed (run with DACTYL_VERBOSE=1 for details)",
      hull_failures,
      sub_failures
    );
  }

  if dump_hulls {
    if let Ok(points_log) = geom::HULL_POINTS_LOG.lock() {
      for (i, pts) in points_log.iter().enumerate() {
        for p in pts {
          println!("HULLPT {} {:.4} {:.4} {:.4}", i, p.0, p.1, p.2);
        }
      }
    }
    let log = geom::HULL_BBOX_LOG.lock().unwrap();
    for b in log.iter() {
      println!(
        "HULLBBOX {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {}",
        b[0], b[1], b[2], b[3], b[4], b[5], b[6]
      );
    }
  }
  if geom::verbose() {
    // Probe the screw insert positions against the resulting solids.
    let probes = [
      (46.0, -46.0, 36.0),
      (-68.80, 56.46, 5.0),
      (55.10, 41.64, 5.0),
      (64.04, -13.81, 5.0),
      (59.18, -72.00, 5.0),
      (-67.29, -21.50, 5.0),
    ];
    for (x, y, z) in probes {
      let p = Point3::new(x, y, z);
      let mut hits = 0;
      for solid in &result.solids {
        if convex_contains(solid, &[p]) {
          hits += 1;
          let (mn, mx) = bounding_box(solid);
          println!(
            "probe {:.1},{:.1},{:.1} inside solid bbox {:.1},{:.1},{:.1}..{:.1},{:.1},{:.1}",
            x, y, z, mn.x, mn.y, mn.z, mx.x, mx.y, mx.z
          );
        }
      }
      println!("probe {:.1},{:.1}: {} solids", x, y, hits);
    }
  }
  if dump_solids {
    use truck_meshalgo::prelude::*;
    let solids = &result.solids;
    println!("SOLIDCOUNT {}", solids.len());
    let mut tri_total = 0usize;
    for solid in solids {
      println!("SOLID");
      for shell in solid.boundaries() {
        println!("SHELL");
        let mesh = shell.triangulation(MESH_TOL).to_polygon();
        let positions = mesh.positions().clone();
        for face in mesh.face_iter() {
          if face.len() != 3 {
            continue;
          }
          let p = face.iter().map(|v| positions[v.pos]).collect::<Vec<_>>();
          println!(
            "TRI {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}",
            p[0].x, p[0].y, p[0].z, p[1].x, p[1].y, p[1].z, p[2].x, p[2].y, p[2].z
          );
          tri_total += 1;
        }
      }
    }
    eprintln!("dumped {} triangles", tri_total);
  }
  println!("writing left.stl");
  write_mesh_stl(&result, &out_path(&out_dir, "left.stl")).unwrap();
  println!("writing right.stl");
  write_mesh_stl(&result.mirror_x(), &out_path(&out_dir, "right.stl")).unwrap();

  // Bottom plate
  {
    println!("building bottom plate...");
    let mut shadow_polygons: Vec<Vec<(f64, f64)>> = Vec::new();
    // The original projects the un-subtracted case: cutting holes out of a
    // convex solid does not change its shadow.
    for solid in &case_shapes.solids {
      let poly = hull_2d(solid_points(solid));
      if poly.len() >= 3 {
        shadow_polygons.push(poly);
      }
    }
    // The switches are hulled together per key in the original.
    for key in d.all_keys() {
      let switch = key.get_switch();
      let mut points = Vec::new();
      for solid in switch.solids {
        points.extend(solid_points(&solid));
      }
      let poly = hull_2d(points);
      if poly.len() >= 3 {
        shadow_polygons.push(poly);
      }
    }

    if let Ok(path) = std::env::var("DACTYL_DUMP_SHADOWS") {
      let mut text = String::new();
      for (i, poly) in shadow_polygons.iter().enumerate() {
        for p in poly {
          text.push_str(&format!("SHADOW {} {:.4} {:.4}\n", i, p.0, p.1));
        }
      }
      let _ = std::fs::write(&path, text);
    }
    let loops = union_polygons_2d(&shadow_polygons, 0.2);
    if let Ok(path) = std::env::var("DACTYL_DUMP_LOOPS") {
      let mut text = String::new();
      for (i, poly) in loops.iter().enumerate() {
        for p in poly {
          text.push_str(&format!("LOOP {} {:.4} {:.4}\n", i, p.0, p.1));
        }
      }
      let _ = std::fs::write(&path, text);
    }
    println!("plate outline: {} loops", loops.len());
    let mut plate_prisms: Vec<Solid> = Vec::new();
    for poly in &loops {
      if let Some(p) = prism(poly, 1.5) {
        plate_prisms.push(builder_translate_z(p, -0.75));
      }
    }
    let plate = Shape { solids: plate_prisms };

    // The original bottom plate only cuts the screw holes. The holes are the
    // faceted cylinders from the reference (`Cylinder(screw_height + 5,
    // screw_radius, 30)`); taking the 2D hull of the smooth cylinder solids is
    // wrong because their B-rep only stores three circle vertices.
    let plate_hole = cylinder(screw_height + 5.0, screw_radius, 30, true);
    let holes: Vec<Solid> = screw_positions
      .iter()
      .flat_map(|p| plate_hole.translate(p.x, p.y, p.z).solids)
      .collect();
    let plate = plate.subtract(&Shape { solids: holes });

    println!("writing bottom_left.stl");
    write_mesh_stl(&plate, &out_path(&out_dir, "bottom_left.stl")).unwrap();
    println!("writing bottom_right.stl");
    write_mesh_stl(&plate.mirror_x(), &out_path(&out_dir, "bottom_right.stl")).unwrap();
  }
}

/// `truck-shapeops` panics when it produces a non-closed shell (a documented
/// robustness gap for solids sharing axis aligned faces). Those panics are
/// handled with `catch_unwind`, so keep their noise out of the output.
fn install_panic_hook() {
  let default = std::panic::take_hook();
  std::panic::set_hook(Box::new(move |info| {
    let msg = info.to_string();
    // `truck-shapeops` panics instead of returning `None` in a few degenerate
    // cases; those are caught with `catch_unwind`.
    let in_shapeops = info
      .location()
      .map(|loc| loc.file().contains("truck-shapeops"))
      .unwrap_or(false);
    if msg.contains("This shell is not oriented and closed") || (in_shapeops && !verbose()) {
      return;
    }
    default(info);
  }));
}

fn out_path(dir: &str, name: &str) -> String {
  if dir == "." {
    name.to_string()
  } else {
    format!("{}/{}", dir.trim_end_matches('/'), name)
  }
}

fn builder_translate_z(solid: Solid, z: f64) -> Solid {
  truck_modeling::builder::translated(&solid, truck_modeling::Vector3::new(0.0, 0.0, z))
}

fn connect_main_keys(d: &KeyData) -> Shape {
  let connector = get_post_connector();
  let mut shapes = Vec::new();
  for r in 0..d.grid_num_rows() {
    for c in 0..d.grid_num_columns() {
      let Some(key) = d.grid_get(r as isize, c as isize) else {
        continue;
      };
      let left = d.grid_get(r as isize, c as isize - 1);
      let top_left = d.grid_get(r as isize - 1, c as isize - 1);
      let top = d.grid_get(r as isize - 1, c as isize);

      if let Some(left) = left {
        shapes.push(connect_horizontal(left, key, &connector, 0.0));
      }
      if let Some(top) = top {
        shapes.push(connect_vertical(top, key, &connector, 0.0));
        if let (Some(left), Some(top_left)) = (left, top_left) {
          shapes.push(connect_diagonal(
            top_left,
            top,
            key,
            left,
            &connector,
            0.0,
          ));
        }
      }
    }
  }
  Shape::union_all(shapes)
}

// ---------------------------------------------------------------------------
// Sanity checks of the geometry primitives (`--sanity`)
// ---------------------------------------------------------------------------

fn mesh_volume(mesh: &truck_meshalgo::rexport_polymesh::PolygonMesh) -> f64 {
  let positions = mesh.positions();
  let mut volume = 0.0;
  for face in mesh.face_iter() {
    let p = face
      .iter()
      .map(|v| positions[v.pos])
      .collect::<Vec<_>>();
    for k in 1..p.len().saturating_sub(1) {
      let a = p[0];
      let b = p[k];
      let c = p[k + 1];
      volume += (a.x * (b.y * c.z - b.z * c.y)
        + a.y * (b.z * c.x - b.x * c.z)
        + a.z * (b.x * c.y - b.y * c.x))
        / 6.0;
    }
  }
  volume
}

fn sanity() {
  use truck_modeling::ShellCondition;

  // Convex hull of a box's vertices must be a closed shell and tessellate.
  let cube_shape = cube(10.0, 10.0, 10.0, true);
  let cube_solid = cube_shape.collapse().expect("cube solid");
  assert!(
    cube_solid.boundaries()[0].shell_condition() == ShellCondition::Closed,
    "cube shell is not closed"
  );
  let hull = hull_solid(&solid_points(&cube_solid)).expect("cube hull");
  assert!(
    hull.boundaries()[0].shell_condition() == ShellCondition::Closed,
    "cube hull shell is not closed"
  );
  println!(
    "cube hull: tris {} vol {:.3} mc {:.3}",
    shape_mesh(&Shape::new(hull.clone())).face_iter().count(),
    mesh_volume(&shape_mesh(&Shape::new(hull.clone()))),
    hull_volume_mc(&hull)
  );

  // A cylinder hole through a box.
  let cylinder_solid = cylinder(15.0, 2.2, 30, true)
    .collapse()
    .expect("cylinder solid");
  let mut hole = cylinder_solid.clone();
  hole.not();
  let cut = truck_shapeops::and(&cube_solid, &hole, BOOL_TOL);
  assert!(cut.is_some(), "cylinder subtraction failed");

  // Hull of the switch side nub (cube hulled with a cylinder). quickhull's
  // polygonal hull must not lose the hull's faces here.
  let nub_cube = cube(K_WALL_WIDTH, 2.75, K_SWITCH_THICKNESS, true).translate(
    K_WALL_WIDTH / 2.0 + K_SWITCH_WIDTH / 2.0,
    0.0,
    K_SWITCH_THICKNESS / 2.0,
  );
  let nub_cyl = cylinder(2.75, 1.0, 30, true)
    .rotate_x(90.0)
    .translate(K_SWITCH_WIDTH / 2.0, 0.0, 1.0);
  let nub_points: Vec<Point3> = nub_cube
    .solids
    .iter()
    .chain(nub_cyl.solids.iter())
    .flat_map(|s| solid_points(s))
    .collect();
  let nub_poly = hull_solid(&nub_points).expect("nub hull");
  let nub_tri = hull_solid_triangles(&nub_points).expect("nub triangle hull");
  let desc = |s: &Solid| {
    let (mn, mx) = bounding_box(s);
    format!(
      "faces {} bbox {:.3},{:.3},{:.3} .. {:.3},{:.3},{:.3}",
      s.boundaries().iter().map(|sh| sh.len()).sum::<usize>(),
      mn.x,
      mn.y,
      mn.z,
      mx.x,
      mx.y,
      mx.z
    )
  };
  println!("nub cube bbox: {:?}", nub_cube.solids.iter().map(|s| bounding_box(s)).collect::<Vec<_>>());
  println!("nub cyl bbox: {:?}", nub_cyl.solids.iter().map(|s| bounding_box(s)).collect::<Vec<_>>());
  let pmin = nub_points.iter().fold(Point3::new(1e9,1e9,1e9), |a,p| Point3::new(a.x.min(p.x), a.y.min(p.y), a.z.min(p.z)));
  let pmax = nub_points.iter().fold(Point3::new(-1e9,-1e9,-1e9), |a,p| Point3::new(a.x.max(p.x), a.y.max(p.y), a.z.max(p.z)));
  println!("nub points: {} bbox {:.2},{:.2},{:.2} .. {:.2},{:.2},{:.2}", nub_points.len(), pmin.x,pmin.y,pmin.z,pmax.x,pmax.y,pmax.z);
  println!("cube hull: vol {:.3} mc {:.3}", mesh_volume(&shape_mesh(&Shape::new(hull.clone()))), hull_volume_mc(&hull));
  println!("nub hull poly: {} vol {:.4}", desc(&nub_poly), mesh_volume(&shape_mesh(&Shape::new(nub_poly.clone()))));
  println!("nub hull tri : {} vol {:.4}", desc(&nub_tri), mesh_volume(&shape_mesh(&Shape::new(nub_tri.clone()))));

  // hull of two thin boxes far apart (the wall connector case)
  let c1 = cube(0.01, 0.01, 3.5, true).translate(0.0, 0.0, 0.0);
  let c2 = cube(0.01, 0.01, 3.5, true).translate(19.0, 0.0, 0.0);
  let thin_points: Vec<Point3> = c1
    .solids
    .iter()
    .chain(c2.solids.iter())
    .flat_map(|s| solid_points(s))
    .collect();
  let thin_poly = hull_solid(&thin_points).expect("thin hull");
  let thin_tri = hull_solid_triangles(&thin_points).expect("thin tri hull");
  println!("thin poly: {} vol {:.4}", desc(&thin_poly), mesh_volume(&shape_mesh(&Shape::new(thin_poly.clone()))));
  println!("thin tri : {} vol {:.4}", desc(&thin_tri), mesh_volume(&shape_mesh(&Shape::new(thin_tri.clone()))));

  // Union of two overlapping boxes.
  let b = cube(10.0, 10.0, 10.0, true).translate(5.0, 0.0, 0.0);
  let union = Shape::new(cube_solid).union(&b).collapse();
  assert!(union.is_some(), "box union failed");

  // 2D union of two overlapping squares.
  let sq = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
  let sq2 = vec![(5.0, 5.0), (15.0, 5.0), (15.0, 15.0), (5.0, 15.0)];
  let loops = union_polygons_2d(&[sq, sq2], 0.5);
  assert_eq!(loops.len(), 1, "2D union should produce one outline");
  let xs: Vec<f64> = loops[0].iter().map(|p| p.0).collect();
  let ys: Vec<f64> = loops[0].iter().map(|p| p.1).collect();
  let min_x = xs.iter().cloned().fold(f64::INFINITY, f64::min);
  let max_x = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
  let min_y = ys.iter().cloned().fold(f64::INFINITY, f64::min);
  let max_y = ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
  assert!(min_x <= 0.01 && max_x >= 14.99 && min_y <= 0.01 && max_y >= 14.99);

  println!("sanity checks passed");
}
