//! Key layout data (port of key_data.cc / key_data.h).

use crate::geom::*;
use crate::key::*;
use truck_modeling::Point3;

const K_DEFAULT_KEY_SPACING: f64 = 19.0;
/// The direct distance between switch tops in the bowl.
const K_BOWL_KEY_SPACING: f64 = 18.0;

const K_D_COLUMN_RADIUS: f64 = 55.0;
const K_A_COLUMN_RADIUS: f64 = 70.0;
const K_S_COLUMN_RADIUS: f64 = 65.0;
const K_G_COLUMN_RADIUS: f64 = 65.0;
const K_F_COLUMN_RADIUS: f64 = 70.0;
const K_CAPS_COLUMN_RADIUS: f64 = 60.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyId {
  Plus,
  One,
  Two,
  Three,
  Four,
  Five,
  Tab,
  Q,
  W,
  E,
  R,
  T,
  Caps,
  A,
  S,
  D,
  F,
  G,
  Shift,
  Z,
  X,
  C,
  V,
  B,
  Tilde,
  Slash,
  LeftArrow,
  RightArrow,
  Backspace,
  Delete,
  End,
  Home,
  Ctrl,
  Alt,
}

const NUM_KEYS: usize = 34;

impl KeyId {
  fn index(self) -> usize {
    self as usize
  }
}

pub struct KeyData {
  keys: Vec<Key>,
  grid: Vec<Vec<Option<KeyId>>>,
  thumb: Vec<KeyId>,
}

/// Rotates a key about the x axis until it has traveled the direct distance (not on the arc).
fn get_rotated_key(radius: f64, up: bool) -> Key {
  let distance = K_BOWL_KEY_SPACING;
  let rotation_direction = if up { 1.0 } else { -1.0 };
  let mut degrees = 1.0;

  // Use precomputed numbers for known radiuses to speed up execution.
  if radius == 50.0 {
    degrees = 20.740;
  }
  if radius == 55.0 {
    degrees = 18.840;
  }
  if radius == 60.0 {
    degrees = 17.26;
  }
  if radius == 65.0 {
    degrees = 15.920;
  }
  if radius == 70.0 {
    degrees = 14.780;
  }

  let mut is_first = true;
  loop {
    let mut k = Key::default();
    k.local_transforms
      .translate_z(-1.0 * radius)
      .rotate_x(rotation_direction * degrees)
      .translate_z(radius);
    let point = k.get_transforms().apply_point(Point3::new(0.0, 0.0, 0.0));
    let d = point - Point3::new(0.0, 0.0, 0.0);
    let current_distance = (d.x * d.x + d.y * d.y + d.z * d.z).sqrt();
    if current_distance > distance {
      if !is_first {
        println!("Computed degrees {:.3} for radius {:.3}", degrees, radius);
      }
      return k;
    }
    degrees += 0.01;
    is_first = false;
  }
}

impl KeyData {
  pub fn new(key_origin: &TransformList) -> Self {
    let mut keys: Vec<Key> = (0..NUM_KEYS).map(|_| Key::default()).collect();

    // -------------------------------------------------------------
    // Thumb keys
    // -------------------------------------------------------------

    let parent = key_origin.clone();
    let k = &mut keys[KeyId::Backspace.index()];
    k.name = "key_backspace".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(60.0, -9.18, 42.83);
    let t = k.t();
    t.rz = -21.0;
    t.rx = 12.0;
    t.ry = -4.5;

    // Second thumb key.
    let parent = keys[KeyId::Backspace.index()].get_transforms();
    let k = &mut keys[KeyId::Delete.index()];
    k.name = "key_delete".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(K_DEFAULT_KEY_SPACING, 0.0, 0.0);

    // Bottom side key.
    let parent = keys[KeyId::Delete.index()].get_transforms();
    let k = &mut keys[KeyId::End.index()];
    k.name = "key_end".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(K_DEFAULT_KEY_SPACING, -9.0, 0.0);

    // Middle side key.
    let parent = keys[KeyId::Delete.index()].get_transforms();
    let k = &mut keys[KeyId::Home.index()];
    k.name = "key_home".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(K_DEFAULT_KEY_SPACING, 10.0, 0.0);

    // Top side key.
    let parent = keys[KeyId::Delete.index()].get_transforms();
    let k = &mut keys[KeyId::Alt.index()];
    k.name = "key_alt".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(K_DEFAULT_KEY_SPACING, 10.0 + K_DEFAULT_KEY_SPACING, 0.0);

    // Top left key.
    let parent = keys[KeyId::Delete.index()].get_transforms();
    let k = &mut keys[KeyId::Ctrl.index()];
    k.name = "key_ctrl".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(0.0, 10.0 + K_DEFAULT_KEY_SPACING, 0.0);

    // -------------------------------------------------------------
    // Main bowl keys
    // -------------------------------------------------------------

    // All keys in the dish are relative to d and then based off of their
    // associated key in the home row.
    let parent = key_origin.clone();
    let k = &mut keys[KeyId::D.index()];
    k.name = "d".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(26.40, 50.32, 17.87);
    k.t().ry = -15.0;

    let parent = keys[KeyId::D.index()].get_transforms();
    let k = &mut keys[KeyId::F.index()];
    k.name = "f".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(19.938, -0.950, 5.249);
    k.t().ry = -5.0;

    let parent = keys[KeyId::F.index()].get_transforms();
    let k = &mut keys[KeyId::G.index()];
    k.name = "g".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(20.0, -1.310, 3.305);
    k.t().ry = -4.0;

    let parent = keys[KeyId::D.index()].get_transforms();
    let k = &mut keys[KeyId::S.index()];
    k.name = "s".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(-19.571, -0.090, 5.430);
    k.t().ry = 5.0;

    let parent = keys[KeyId::S.index()].get_transforms();
    let k = &mut keys[KeyId::A.index()];
    k.name = "a".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(-20.887, -6.170, 5.358);

    let parent = keys[KeyId::A.index()].get_transforms();
    let k = &mut keys[KeyId::Caps.index()];
    k.name = "caps".to_string();
    k.set_parent_transforms(&parent);
    k.set_position(-22.597, 4.000, 0.207);
    k.t().ry = 5.0;

    // D column
    keys[KeyId::E.index()] = get_rotated_key(K_D_COLUMN_RADIUS, true);
    let parent = keys[KeyId::D.index()].get_transforms();
    let k = &mut keys[KeyId::E.index()];
    k.name = "e".to_string();
    k.set_parent_transforms(&parent);

    // This key is different from the others in the column. It should be less
    // angled due to the larger radius.
    keys[KeyId::Three.index()] = get_rotated_key(K_D_COLUMN_RADIUS + 15.0, true);
    let parent = keys[KeyId::E.index()].get_transforms();
    let k = &mut keys[KeyId::Three.index()];
    k.name = "3".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::C.index()] = get_rotated_key(K_D_COLUMN_RADIUS, false);
    let parent = keys[KeyId::D.index()].get_transforms();
    let k = &mut keys[KeyId::C.index()];
    k.name = "c".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::LeftArrow.index()] = get_rotated_key(K_D_COLUMN_RADIUS, false);
    let parent = keys[KeyId::C.index()].get_transforms();
    let k = &mut keys[KeyId::LeftArrow.index()];
    k.name = "left_arrow".to_string();
    k.set_parent_transforms(&parent);

    // S column
    keys[KeyId::W.index()] = get_rotated_key(K_S_COLUMN_RADIUS, true);
    let parent = keys[KeyId::S.index()].get_transforms();
    let k = &mut keys[KeyId::W.index()];
    k.name = "w".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::Two.index()] = get_rotated_key(K_S_COLUMN_RADIUS, true);
    let parent = keys[KeyId::W.index()].get_transforms();
    let k = &mut keys[KeyId::Two.index()];
    k.name = "2".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::X.index()] = get_rotated_key(K_S_COLUMN_RADIUS, false);
    let parent = keys[KeyId::S.index()].get_transforms();
    let k = &mut keys[KeyId::X.index()];
    k.name = "x".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::Slash.index()] = get_rotated_key(K_S_COLUMN_RADIUS, false);
    let parent = keys[KeyId::X.index()].get_transforms();
    let k = &mut keys[KeyId::Slash.index()];
    k.name = "slash".to_string();
    k.set_parent_transforms(&parent);

    // F column
    keys[KeyId::R.index()] = get_rotated_key(K_F_COLUMN_RADIUS, true);
    let parent = keys[KeyId::F.index()].get_transforms();
    let k = &mut keys[KeyId::R.index()];
    k.name = "r".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::Four.index()] = get_rotated_key(K_F_COLUMN_RADIUS, true);
    let parent = keys[KeyId::R.index()].get_transforms();
    let k = &mut keys[KeyId::Four.index()];
    k.name = "4".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::V.index()] = get_rotated_key(K_F_COLUMN_RADIUS, false);
    let parent = keys[KeyId::F.index()].get_transforms();
    let k = &mut keys[KeyId::V.index()];
    k.name = "v".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::RightArrow.index()] = get_rotated_key(K_F_COLUMN_RADIUS, false);
    let parent = keys[KeyId::V.index()].get_transforms();
    let k = &mut keys[KeyId::RightArrow.index()];
    k.name = "right_arrow".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::T.index()] = get_rotated_key(K_G_COLUMN_RADIUS, true);
    let parent = keys[KeyId::G.index()].get_transforms();
    let k = &mut keys[KeyId::T.index()];
    k.name = "t".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::Five.index()] = get_rotated_key(K_G_COLUMN_RADIUS, true);
    let parent = keys[KeyId::T.index()].get_transforms();
    let k = &mut keys[KeyId::Five.index()];
    k.name = "5".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::B.index()] = get_rotated_key(K_G_COLUMN_RADIUS, false);
    let parent = keys[KeyId::G.index()].get_transforms();
    let k = &mut keys[KeyId::B.index()];
    k.name = "b".to_string();
    k.set_parent_transforms(&parent);

    // A column
    keys[KeyId::Q.index()] = get_rotated_key(K_A_COLUMN_RADIUS, true);
    let parent = keys[KeyId::A.index()].get_transforms();
    let k = &mut keys[KeyId::Q.index()];
    k.name = "q".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::One.index()] = get_rotated_key(K_A_COLUMN_RADIUS, true);
    let parent = keys[KeyId::Q.index()].get_transforms();
    let k = &mut keys[KeyId::One.index()];
    k.name = "1".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::Z.index()] = get_rotated_key(K_A_COLUMN_RADIUS, false);
    let parent = keys[KeyId::A.index()].get_transforms();
    let k = &mut keys[KeyId::Z.index()];
    k.name = "z".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::Tilde.index()] = get_rotated_key(K_A_COLUMN_RADIUS, false);
    let parent = keys[KeyId::Z.index()].get_transforms();
    let k = &mut keys[KeyId::Tilde.index()];
    k.name = "tilde".to_string();
    k.set_parent_transforms(&parent);

    // Caps column
    keys[KeyId::Tab.index()] = get_rotated_key(K_CAPS_COLUMN_RADIUS, true);
    let parent = keys[KeyId::Caps.index()].get_transforms();
    let k = &mut keys[KeyId::Tab.index()];
    k.name = "tab".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::Plus.index()] = get_rotated_key(K_CAPS_COLUMN_RADIUS, true);
    let parent = keys[KeyId::Tab.index()].get_transforms();
    let k = &mut keys[KeyId::Plus.index()];
    k.name = "plus".to_string();
    k.set_parent_transforms(&parent);

    keys[KeyId::Shift.index()] = get_rotated_key(K_CAPS_COLUMN_RADIUS, false);
    let parent = keys[KeyId::Caps.index()].get_transforms();
    let k = &mut keys[KeyId::Shift.index()];
    k.name = "shift".to_string();
    k.set_parent_transforms(&parent);

    // Keys are measured from the tip of the switch and by default keys are
    // measured from the tip of the cap. Adjust the keys position so that the
    // origin is at the switch top.
    let thumb = vec![
      KeyId::Delete,
      KeyId::Backspace,
      KeyId::Ctrl,
      KeyId::Alt,
      KeyId::Home,
      KeyId::End,
    ];
    let grid_ids: Vec<Vec<Option<KeyId>>> = vec![
      vec![Some(KeyId::Plus), Some(KeyId::One), Some(KeyId::Two), Some(KeyId::Three), Some(KeyId::Four), Some(KeyId::Five)],
      vec![Some(KeyId::Tab), Some(KeyId::Q), Some(KeyId::W), Some(KeyId::E), Some(KeyId::R), Some(KeyId::T)],
      vec![Some(KeyId::Caps), Some(KeyId::A), Some(KeyId::S), Some(KeyId::D), Some(KeyId::F), Some(KeyId::G)],
      vec![Some(KeyId::Shift), Some(KeyId::Z), Some(KeyId::X), Some(KeyId::C), Some(KeyId::V), Some(KeyId::B)],
      vec![None, Some(KeyId::Tilde), Some(KeyId::Slash), Some(KeyId::LeftArrow), Some(KeyId::RightArrow), None],
    ];

    // Collect every key in the same order as the original: thumb keys first,
    // then the grid row-major.
    let mut all_ids: Vec<KeyId> = thumb.clone();
    for row in &grid_ids {
      for id in row {
        if let Some(id) = id {
          all_ids.push(*id);
        }
      }
    }

    for id in all_ids {
      let key = &mut keys[id.index()];
      key.add_transform();
      key.disable_switch_z_offset = true;
      // The 10mm switch tip offset from the original code.
      key.t().z -= 10.0;
    }

    KeyData {
      keys,
      grid: grid_ids,
      thumb,
    }
  }

  pub fn key(&self, id: KeyId) -> &Key {
    &self.keys[id.index()]
  }

  pub fn key_mut(&mut self, id: KeyId) -> &mut Key {
    &mut self.keys[id.index()]
  }

  #[allow(dead_code)]
  pub fn thumb_keys(&self) -> Vec<&Key> {
    self.thumb.iter().map(|id| self.key(*id)).collect()
  }

  #[allow(dead_code)]
  pub fn grid_column(&self, c: usize) -> Vec<Option<&Key>> {
    self.grid
      .iter()
      .map(|row| {
        row
          .get(c)
          .and_then(|id| id.as_ref())
          .map(|id| self.key(*id))
      })
      .collect()
  }

  #[allow(dead_code)]
  pub fn grid_row(&self, r: usize) -> Vec<Option<&Key>> {
    self.grid[r]
      .iter()
      .map(|id| id.as_ref().map(|id| self.key(*id)))
      .collect()
  }

  pub fn grid_get(&self, r: isize, c: isize) -> Option<&Key> {
    if r < 0 || c < 0 {
      return None;
    }
    let row = self.grid.get(r as usize)?;
    let id = row.get(c as usize)?;
    id.as_ref().map(|id| self.key(*id))
  }

  pub fn grid_num_rows(&self) -> usize {
    self.grid.len()
  }

  pub fn grid_num_columns(&self) -> usize {
    self.grid[0].len()
  }

  /// All keys in the original order (thumb keys, then grid keys).
  pub fn all_keys(&self) -> Vec<&Key> {
    self.thumb
      .iter()
      .map(|id| self.key(*id))
      .chain(self.grid.iter().flatten().filter_map(|id| id.as_ref()).map(|id| self.key(*id)))
      .collect()
  }

  #[allow(dead_code)]
  pub fn all_key_ids(&self) -> Vec<KeyId> {
    let mut result = self.thumb.clone();
    for row in &self.grid {
      for id in row {
        if let Some(id) = id {
          result.push(*id);
        }
      }
    }
    result
  }
}

impl KeyData {
  pub fn grid_column_ids(&self, c: usize) -> Vec<Option<KeyId>> {
    self.grid.iter().map(|row| row.get(c).copied().flatten()).collect()
  }

  pub fn grid_row_ids(&self, r: usize) -> Vec<Option<KeyId>> {
    self.grid[r].clone()
  }

  #[allow(dead_code)]
  pub fn grid_id_at(&self, r: isize, c: isize) -> Option<KeyId> {
    if r < 0 || c < 0 {
      return None;
    }
    self.grid.get(r as usize)?.get(c as usize).copied().flatten()
  }
}
