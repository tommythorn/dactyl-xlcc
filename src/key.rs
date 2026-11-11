//! Switches, caps, connectors, and the `Key` abstraction (port of util/key.h/.cc).

use crate::geom::*;

// All sizes in mm (mirrors util/key.h).
pub const K_SWITCH_WIDTH: f64 = 14.4;
pub const K_SWITCH_THICKNESS: f64 = 4.0;
pub const K_WALL_WIDTH: f64 = 2.0;

pub const K_DSA_HEIGHT: f64 = 8.0;
pub const K_SA_HEIGHT: f64 = 12.5;
/// This is the height of the taller side on the key. The short side has K_SA_HEIGHT.
pub const K_SA_EDGE_HEIGHT: f64 = 13.7;
pub const K_DSA_TOP_SIZE: f64 = 13.2;
pub const K_DSA_BOTTOM_SIZE: f64 = 18.4;
pub const K_DSA_HALF_SIZE: f64 = 16.2;
pub const K_SA_HALF_SIZE: f64 = 17.2;

pub const K_SA_TALL_HEIGHT: f64 = 14.0;
pub const K_SA_TALL_EDGE_HEIGHT: f64 = 16.5;

pub const K_SWITCH_HORIZONTAL_OFFSET: f64 = K_SWITCH_WIDTH / 2.0 + K_WALL_WIDTH;
/// Distance between the top of the switch plate and the tip of the switch stem.
#[allow(dead_code)]
pub const K_SWITCH_TIP_OFFSET: f64 = 10.0;

pub const K_DSA_SWITCH_Z_OFFSET: f64 = K_DSA_HEIGHT + 6.4;
pub const K_SA_SWITCH_Z_OFFSET: f64 = K_SA_HEIGHT + 6.4;

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyType {
  Dsa,
  Sa,
  SaEdge,
  SaTallEdge,
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaEdgeType {
  Left,
  Right,
  Top,
  Bottom,
}

// ---------------------------------------------------------------------------
// Switches and caps
// ---------------------------------------------------------------------------

pub fn make_switch(add_side_nub: bool) -> Shape {
  let top_wall = cube(
    K_SWITCH_WIDTH + K_WALL_WIDTH * 2.0,
    K_WALL_WIDTH,
    K_SWITCH_THICKNESS,
    true,
  )
  .translate(0.0, K_WALL_WIDTH / 2.0 + K_SWITCH_WIDTH / 2.0, K_SWITCH_THICKNESS / 2.0);

  let mut walls = vec![
    top_wall.clone(),
    top_wall.rotate_z(90.0),
    top_wall.rotate_z(180.0),
    top_wall.rotate_z(270.0),
  ];

  if add_side_nub {
    let side_nub = Shape::hull(&[
      cube(K_WALL_WIDTH, 2.75, K_SWITCH_THICKNESS, true).translate(
        K_WALL_WIDTH / 2.0 + K_SWITCH_WIDTH / 2.0,
        0.0,
        K_SWITCH_THICKNESS / 2.0,
      ),
      cylinder(2.75, 1.0, 30, true)
        .rotate_x(90.0)
        .translate(K_SWITCH_WIDTH / 2.0, 0.0, 1.0),
    ]);
    walls.push(side_nub.clone());
    walls.push(side_nub.rotate_z(180.0));
  }

  Shape::union_all(walls).translate_z(-K_SWITCH_THICKNESS)
}

struct CapSegment {
  height: f64,
  width: f64,
}

/// Segments from bottom to top.
fn make_cap(segments: &[CapSegment]) -> Shape {
  let total_height: f64 = segments.iter().map(|s| s.height).sum();
  let mut solids = Vec::new();
  let mut height_so_far = 0.0;
  for i in 0..segments.len() - 1 {
    let bottom_width = segments[i].width;
    let top_width = segments[i + 1].width;
    let h = segments[i].height;
    height_so_far += h;
    let z1 = height_so_far;
    let z0 = height_so_far - h;
    if let Some(solid) = frustum(bottom_width, z0, top_width, z1) {
      solids.push(solid);
    }
  }
  Shape { solids }.translate_z(-total_height)
}

fn rotate_cap_edge(shape: Shape, edge_type: SaEdgeType) -> Shape {
  match edge_type {
    SaEdgeType::Left => shape.rotate_z(-90.0),
    SaEdgeType::Right => shape.rotate_z(90.0),
    SaEdgeType::Top => shape.rotate_z(180.0),
    SaEdgeType::Bottom => shape,
  }
}

pub fn make_dsa_cap() -> Shape {
  make_cap(&[
    CapSegment { height: K_DSA_HEIGHT / 2.0, width: K_DSA_BOTTOM_SIZE },
    CapSegment { height: K_DSA_HEIGHT / 2.0, width: K_DSA_HALF_SIZE },
    CapSegment { height: 0.0, width: K_DSA_TOP_SIZE },
  ])
}

pub fn make_sa_cap() -> Shape {
  make_cap(&[
    CapSegment { height: K_SA_HEIGHT / 2.0, width: K_DSA_BOTTOM_SIZE },
    CapSegment { height: K_SA_HEIGHT / 2.0, width: K_SA_HALF_SIZE },
    CapSegment { height: 0.0, width: K_DSA_TOP_SIZE },
  ])
}

fn make_sa_tall_cap() -> Shape {
  make_cap(&[
    CapSegment { height: K_SA_TALL_HEIGHT / 2.0, width: K_DSA_BOTTOM_SIZE },
    CapSegment { height: K_SA_TALL_HEIGHT / 2.0, width: K_DSA_HALF_SIZE },
    CapSegment { height: 0.0, width: K_DSA_TOP_SIZE },
  ])
}

fn make_sa_edge_cap(edge_type: SaEdgeType, tall: bool) -> Shape {
  let sa_cap = if tall { make_sa_tall_cap() } else { make_sa_cap() };
  let (base_height, edge_height) = if tall {
    (K_SA_TALL_HEIGHT, K_SA_TALL_EDGE_HEIGHT - K_SA_TALL_HEIGHT)
  } else {
    (K_SA_HEIGHT, K_SA_EDGE_HEIGHT - K_SA_HEIGHT)
  };
  let _ = base_height;

  let bar = cube(K_DSA_TOP_SIZE, 0.01, 0.01, true);
  let half_top = K_DSA_TOP_SIZE * 0.5;

  let bottom_edge = Shape::union_all(vec![
    sa_cap.clone(),
    Shape::hull(&[
      sa_cap.clone(),
      bar.clone().translate(0.0, -half_top, 0.0),
      bar.clone().translate(0.0, -half_top, edge_height),
      bar.clone().translate(0.0, half_top, 0.0),
    ]),
  ]);
  rotate_cap_edge(bottom_edge, edge_type)
}

pub fn make_sa_edge_cap_for(edge_type: SaEdgeType) -> Shape {
  make_sa_edge_cap(edge_type, false)
}

pub fn make_sa_tall_edge_cap(edge_type: SaEdgeType) -> Shape {
  make_sa_edge_cap(edge_type, true)
}

// ---------------------------------------------------------------------------
// Key
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct Key {
  pub name: String,
  pub parent_transforms: TransformList,
  pub local_transforms: TransformList,

  pub extra_width_top: f64,
  pub extra_width_bottom: f64,
  pub extra_width_left: f64,
  pub extra_width_right: f64,

  pub extra_z: f64,

  pub add_side_nub: bool,
  pub disable_switch_z_offset: bool,

  pub key_type: KeyType,
  pub sa_edge_type: SaEdgeType,
}

impl Default for Key {
  fn default() -> Self {
    Key {
      name: String::new(),
      parent_transforms: TransformList::empty(),
      local_transforms: TransformList::empty(),
      extra_width_top: 0.0,
      extra_width_bottom: 0.0,
      extra_width_left: 0.0,
      extra_width_right: 0.0,
      extra_z: 0.0,
      add_side_nub: true,
      disable_switch_z_offset: false,
      key_type: KeyType::Dsa,
      sa_edge_type: SaEdgeType::Bottom,
    }
  }
}

impl Key {
  #[allow(dead_code)]
  pub fn at(x: f64, y: f64, z: f64) -> Self {
    let mut key = Key::default();
    key.set_position(x, y, z);
    key
  }

  pub fn set_position(&mut self, x: f64, y: f64, z: f64) -> &mut Self {
    let t = self.t();
    t.x = x;
    t.y = y;
    t.z = z;
    self
  }

  pub fn t(&mut self) -> &mut Transform {
    self.local_transforms.mutable_front()
  }

  /// Prepends a transform (C++ `Key::AddTransform` uses `AddTransformFront`):
  /// the key z adjustment added by `key_data` must be applied *before* the key
  /// position and rotations.
  pub fn add_transform(&mut self) -> &mut Transform {
    self.local_transforms.add_transform_front(Transform::default())
  }

  #[allow(dead_code)]
  pub fn set_parent(&mut self, key: &Key) -> &mut Self {
    self.parent_transforms = key.get_transforms();
    self
  }

  pub fn set_parent_transforms(&mut self, transforms: &TransformList) -> &mut Self {
    self.parent_transforms = transforms.clone();
    self
  }

  pub fn get_transforms(&self) -> TransformList {
    let mut transforms = self.local_transforms.clone();
    transforms.append(&self.parent_transforms);
    transforms
  }

  pub fn get_switch_transforms(&self) -> TransformList {
    let mut switch_z_offset = if self.key_type == KeyType::Dsa {
      K_DSA_SWITCH_Z_OFFSET
    } else {
      K_SA_SWITCH_Z_OFFSET
    };
    if self.disable_switch_z_offset {
      switch_z_offset = 0.0;
    }
    let mut transforms = TransformList::empty();
    let t = transforms.add_transform(Transform::default());
    t.z = -switch_z_offset - self.extra_z;
    transforms.append(&self.get_transforms());
    transforms
  }

  #[allow(dead_code)]
  pub fn get_inverse_switch(&self) -> Shape {
    let switch = self.get_switch();
    Shape::hull(&[switch.clone()]).subtract(&switch)
  }

  #[allow(dead_code)]
  pub fn get_inverse_cap(&self, custom_vertical_length: f64) -> Shape {
    let width = K_DSA_BOTTOM_SIZE + 0.1;
    let height = if custom_vertical_length > 0.0 {
      custom_vertical_length
    } else {
      width
    };
    let mut transforms = TransformList::empty();
    let t = transforms.add_transform(Transform::default());
    t.z = self.extra_z;
    transforms.append(&self.get_switch_transforms());
    transforms.apply_shape(&cube(width, height, 30.0, true).translate_z(15.0))
  }

  pub fn get_switch(&self) -> Shape {
    let mut shapes = Vec::new();
    if self.extra_z > 0.0 {
      let mut s = make_switch(false)
        .union(&make_switch(self.add_side_nub).translate_z(self.extra_z));
      if self.extra_z > 4.0 {
        s = s.union(&make_switch(false).translate_z(4.0));
      }
      shapes.push(self.get_switch_transforms().apply_shape(&s));
    } else {
      shapes.push(
        self
          .get_switch_transforms()
          .apply_shape(&make_switch(self.add_side_nub)),
      );
    }

    let post = get_post_connector();
    if self.extra_width_top > 0.0 {
      shapes.push(Shape::hull(&[
        self.get_top_right(0.0).apply_shape(&post),
        self.get_top_right_internal().apply_shape(&post),
        self.get_top_left_internal().apply_shape(&post),
        self.get_top_left(0.0).apply_shape(&post),
      ]));
    }
    if self.extra_width_bottom > 0.0 {
      shapes.push(Shape::hull(&[
        self.get_bottom_left(0.0).apply_shape(&post),
        self.get_bottom_left_internal().apply_shape(&post),
        self.get_bottom_right_internal().apply_shape(&post),
        self.get_bottom_right(0.0).apply_shape(&post),
      ]));
    }
    if self.extra_width_left > 0.0 {
      shapes.push(Shape::hull(&[
        self.get_bottom_left(0.0).apply_shape(&post),
        self.get_bottom_left_internal().apply_shape(&post),
        self.get_top_left_internal().apply_shape(&post),
        self.get_top_left(0.0).apply_shape(&post),
      ]));
    }
    if self.extra_width_right > 0.0 {
      shapes.push(Shape::hull(&[
        self.get_bottom_right(0.0).apply_shape(&post),
        self.get_bottom_right_internal().apply_shape(&post),
        self.get_top_right_internal().apply_shape(&post),
        self.get_top_right(0.0).apply_shape(&post),
      ]));
    }
    Shape::union_all(shapes)
  }

  pub fn get_cap(&self, fill_in_cap_path: bool) -> Shape {
    let (cap, cap_height) = match self.key_type {
      KeyType::Dsa => (make_dsa_cap(), K_DSA_HEIGHT),
      KeyType::Sa => (make_sa_cap(), K_SA_HEIGHT),
      KeyType::SaEdge => (make_sa_edge_cap_for(self.sa_edge_type), K_SA_HEIGHT),
      KeyType::SaTallEdge => (make_sa_tall_edge_cap(self.sa_edge_type), K_SA_TALL_HEIGHT),
    };

    let mut cap = cap;
    if fill_in_cap_path {
      let bottom = cap.project_extrude(6.0, true).translate_z(-3.0 - cap_height);
      cap = cap.union(&bottom);
    }

    if self.disable_switch_z_offset {
      let switch_z_offset = if self.key_type == KeyType::Dsa {
        K_DSA_SWITCH_Z_OFFSET
      } else {
        K_SA_SWITCH_Z_OFFSET
      };
      let mut transforms = TransformList::empty();
      let t = transforms.add_transform(Transform::default());
      t.z = switch_z_offset;
      transforms.append(&self.get_transforms());
      return transforms.apply_shape(&cap);
    }
    self.get_transforms().apply_shape(&cap)
  }

  pub fn get_top_right(&self, offset: f64) -> TransformList {
    let mut transforms = TransformList::empty();
    transforms.add_transform(Transform::translation(
      self.extra_width_right + offset,
      self.extra_width_top + offset,
      0.0,
    ));
    transforms.append(&self.get_top_right_internal());
    transforms
  }

  fn get_top_right_internal(&self) -> TransformList {
    let mut transforms = TransformList::empty();
    transforms.add_transform(Transform::translation(
      K_SWITCH_HORIZONTAL_OFFSET,
      K_SWITCH_HORIZONTAL_OFFSET,
      0.0,
    ));
    transforms.append(&self.get_switch_transforms());
    transforms
  }

  pub fn get_top_left(&self, offset: f64) -> TransformList {
    let mut transforms = TransformList::empty();
    transforms.add_transform(Transform::translation(
      -1.0 * (self.extra_width_left + offset),
      self.extra_width_top + offset,
      0.0,
    ));
    transforms.append(&self.get_top_left_internal());
    transforms
  }

  fn get_top_left_internal(&self) -> TransformList {
    let mut transforms = TransformList::empty();
    transforms.add_transform(Transform::translation(
      -K_SWITCH_HORIZONTAL_OFFSET,
      K_SWITCH_HORIZONTAL_OFFSET,
      0.0,
    ));
    transforms.append(&self.get_switch_transforms());
    transforms
  }

  pub fn get_bottom_right(&self, offset: f64) -> TransformList {
    let mut transforms = TransformList::empty();
    transforms.add_transform(Transform::translation(
      self.extra_width_right + offset,
      -1.0 * (self.extra_width_bottom + offset),
      0.0,
    ));
    transforms.append(&self.get_bottom_right_internal());
    transforms
  }

  fn get_bottom_right_internal(&self) -> TransformList {
    let mut transforms = TransformList::empty();
    transforms.add_transform(Transform::translation(
      K_SWITCH_HORIZONTAL_OFFSET,
      -K_SWITCH_HORIZONTAL_OFFSET,
      0.0,
    ));
    transforms.append(&self.get_switch_transforms());
    transforms
  }

  pub fn get_bottom_left(&self, offset: f64) -> TransformList {
    let mut transforms = TransformList::empty();
    transforms.add_transform(Transform::translation(
      -1.0 * (self.extra_width_left + offset),
      -1.0 * (self.extra_width_bottom + offset),
      0.0,
    ));
    transforms.append(&self.get_bottom_left_internal());
    transforms
  }

  fn get_bottom_left_internal(&self) -> TransformList {
    let mut transforms = TransformList::empty();
    transforms.add_transform(Transform::translation(
      -K_SWITCH_HORIZONTAL_OFFSET,
      -K_SWITCH_HORIZONTAL_OFFSET,
      0.0,
    ));
    transforms.append(&self.get_switch_transforms());
    transforms
  }

  #[allow(dead_code)]
  pub fn get_middle(&self) -> TransformList {
    self.get_switch_transforms()
  }
}

// ---------------------------------------------------------------------------
// Connectors
// ---------------------------------------------------------------------------

pub fn get_post_connector() -> Shape {
  cube(0.01, 0.01, 3.5, true).translate_z(-1.75)
}

pub fn connect_vertical(top: &Key, bottom: &Key, connector: &Shape, offset: f64) -> Shape {
  let a = top.get_bottom_right(offset).apply_shape(connector);
  let b = top.get_bottom_left(offset).apply_shape(connector);
  let c = bottom.get_top_left(offset).apply_shape(connector);
  let d = bottom.get_top_right(offset).apply_shape(connector);
  Shape::hull(&[a.clone(), b.clone(), c.clone()])
    .union(&Shape::hull(&[c, d, a.clone()]))
}

pub fn connect_horizontal(left: &Key, right: &Key, connector: &Shape, offset: f64) -> Shape {
  let a = left.get_top_right(offset).apply_shape(connector);
  let b = left.get_bottom_right(offset).apply_shape(connector);
  let c = right.get_bottom_left(offset).apply_shape(connector);
  let d = right.get_top_left(offset).apply_shape(connector);
  Shape::hull(&[a.clone(), b, c.clone()])
    .union(&Shape::hull(&[c, d, a]))
}

pub fn connect_diagonal(
  top_left: &Key,
  top_right: &Key,
  bottom_right: &Key,
  bottom_left: &Key,
  connector: &Shape,
  offset: f64,
) -> Shape {
  let a = top_left.get_bottom_right(offset).apply_shape(connector);
  let b = top_right.get_bottom_left(offset).apply_shape(connector);
  let c = bottom_right.get_top_left(offset).apply_shape(connector);
  let d = bottom_left.get_top_right(offset).apply_shape(connector);
  Shape::hull(&[a.clone(), b, c.clone()])
    .union(&Shape::hull(&[c, d, a]))
}

pub fn tri(t1: &TransformList, t2: &TransformList, t3: &TransformList, connector: &Shape) -> Shape {
  Shape::hull(&[
    t1.apply_shape(connector),
    t2.apply_shape(connector),
    t3.apply_shape(connector),
  ])
}

pub fn tri_fan(
  center: &TransformList,
  transforms: &[TransformList],
  connector: &Shape,
) -> Shape {
  let center_shape = center.apply_shape(connector);
  let shapes: Vec<Shape> = transforms.iter().map(|t| t.apply_shape(connector)).collect();
  tri_fan_shapes(center_shape, &shapes)
}

pub fn tri_fan_shapes(center: Shape, shapes: &[Shape]) -> Shape {
  let mut result = Vec::new();
  for i in 0..shapes.len().saturating_sub(1) {
    result.push(Shape::hull(&[center.clone(), shapes[i].clone(), shapes[i + 1].clone()]));
  }
  Shape::union_all(result)
}

#[allow(dead_code)]
pub fn tri_mesh(transforms: &[TransformList], connector: &Shape) -> Shape {
  let shapes: Vec<Shape> = transforms.iter().map(|t| t.apply_shape(connector)).collect();
  tri_mesh_shapes(&shapes)
}

pub fn tri_mesh_shapes(shapes: &[Shape]) -> Shape {
  let mut result = Vec::new();
  for i in 0..shapes.len().saturating_sub(2) {
    result.push(Shape::hull(&[
      shapes[i].clone(),
      shapes[i + 1].clone(),
      shapes[i + 2].clone(),
    ]));
  }
  Shape::union_all(result)
}
