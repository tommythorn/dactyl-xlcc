//! Controller tray for an Adafruit Feather nRF52840 Express and a TRRS jack,
//! and the dock in the back wall of the case that it slides into.
//!
//! The tray is a separate print. It carries the Feather (micro USB end towards
//! the back of the case) and a PJ-320A style TRRS jack next to it, behind a
//! faceplate that fills a notch in the back wall. The USB end of the board
//! sits in a slot through the faceplate, flush with the outside. With the bottom plate off,
//! the tray slides up into the notch from below: flanges on both sides of the
//! faceplate run in channels of the dock, so the tray cannot move in or out,
//! and the bottom plate keeps it from dropping out.
//!
//! The back wall is not parallel to the x axis, so the faceplate (and the dock)
//! follow the wall, skewed, while the board and the jack point straight back
//! (along y) to stay clear of the switches of the D column. All plan view
//! shapes of the faceplate and dock are therefore parallelograms: two sides
//! along the wall, two sides along y. `d` below is the depth into the case,
//! measured along y from the outer face of the faceplate.

use crate::geom::*;
use truck_modeling::{Point3, Vector3};

// --- The parts (nominal sizes, mm) -----------------------------------------

/// Feather PCB: 2.0" x 0.9", 1.6 mm thick, mounting holes 0.1" from the edges.
const FEATHER_WIDTH: f64 = 22.86;
const FEATHER_LENGTH: f64 = 50.8;
const PCB_THICKNESS: f64 = 1.6;
const FEATHER_HOLE_INSET: f64 = 2.54;
/// The pegs that go through the four mounting holes (the holes are 2.54 mm).
const PEG_DIAMETER: f64 = 2.3;
const PEG_HEIGHT: f64 = 2.4;

/// PJ-320A TRRS jack body (without the barrel) and its barrel.
const JACK_WIDTH: f64 = 6.1;
const JACK_LENGTH: f64 = 12.1;
const JACK_HEIGHT: f64 = 5.0;
const JACK_BARREL_DIAMETER: f64 = 5.0;
const JACK_BARREL_LENGTH: f64 = 2.0;

// --- The tray ---------------------------------------------------------------

/// Fit clearance around the board and the jack.
const FIT: f64 = 0.25;
const FLOOR_THICKNESS: f64 = 2.0;
/// Room under the board for solder joints and wire ends.
const STANDOFF_HEIGHT: f64 = 2.0;
/// The USB end of the board sits in a slot through the faceplate, as wide as
/// the board. This is how far the slot reaches above the top of the PCB.
const SLOT_ABOVE_PCB_TOP: f64 = 6.3;
/// How far the USB end of the board stays behind the outer face of the
/// faceplate, where it comes closest (the faceplate is skewed, the board is
/// not).
const FEATHER_SETBACK: f64 = 0.0;
/// Thickness of the faceplate in front of the jack: the length of the barrel,
/// so the barrel ends flush with the outside.
const JACK_SKIN: f64 = JACK_BARREL_LENGTH;
const JACK_WALL: f64 = 1.2;
/// Height of the pedestal the jack sits on.
const JACK_PEDESTAL: f64 = 3.0;
/// Height of the lip behind the jack. Low enough to get the jack in: barrel
/// first into its hole, at an angle, then the back end down behind the lip.
const JACK_BACK_LIP: f64 = 1.5;
/// Faceplate material left above the slot for the board.
const FACE_TOP_MARGIN: f64 = 1.5;
const FACE_SIDE_MARGIN: f64 = 1.0;

// --- The dock ---------------------------------------------------------------

/// Depth of the part of the faceplate that sits in the notch in the wall.
const PLUG_DEPTH: f64 = 4.0;
/// The flanges: how far they reach past the notch on both sides, and the
/// channel they run in (depth range behind the outer face).
const FLANGE_REACH: f64 = 2.5;
const FLANGE_FRONT: f64 = PLUG_DEPTH + 0.05;
const FLANGE_BACK: f64 = 6.02;
const CHANNEL_BACK: f64 = 6.1;
/// Clearance between the faceplate and the sides and top of the notch.
const NOTCH_CLEARANCE: f64 = 0.04;
/// Width of the dock on the left side of the notch (the width on the right
/// side, next to the corner of the case, is up to the caller), and its total
/// depth.
const DOCK_SIDE: f64 = 5.0;
const DOCK_DEPTH: f64 = 8.4;
/// Height of the dock above the notch (the lintel).
const DOCK_LINTEL: f64 = 3.0;

fn pcb_top() -> f64 {
  FLOOR_THICKNESS + STANDOFF_HEIGHT + PCB_THICKNESS
}

/// Height of the faceplate.
fn face_height() -> f64 {
  pcb_top() + SLOT_ABOVE_PCB_TOP + FACE_TOP_MARGIN
}

/// Width of the part of the faceplate that sits in the notch.
fn face_width() -> f64 {
  2.0 * FACE_SIDE_MARGIN + (JACK_WIDTH + 2.0 * FIT + 2.0 * JACK_WALL) + 0.5 + (FEATHER_WIDTH + 2.0 * FIT)
}

/// Where the tray goes: the notch in the back wall.
pub struct Dock {
  /// Left and right end of the notch on the outer face of the wall.
  left: (f64, f64),
  right: (f64, f64),
  /// Width of the dock on the right side of the notch.
  side_right: f64,
}

impl Dock {
  /// `wall` is the outer face of the back wall in plan view (sorted by x), and
  /// `right_x` where the right side of the notch should be. `side_right` is
  /// how far the dock reaches past the notch on that side.
  pub fn new(wall: &[(f64, f64)], right_x: f64, side_right: f64) -> Dock {
    let wall_y = |x: f64| {
      for w in wall.windows(2) {
        if x <= w[1].0 || w[1].0 == wall[wall.len() - 1].0 {
          let s = (x - w[0].0) / (w[1].0 - w[0].0);
          return w[0].1 + s * (w[1].1 - w[0].1);
        }
      }
      wall[wall.len() - 1].1
    };
    let left_x = right_x - face_width() - 2.0 * NOTCH_CLEARANCE;
    let dock = Dock {
      left: (left_x, wall_y(left_x)),
      right: (right_x, wall_y(right_x)),
      side_right: side_right.min(DOCK_SIDE),
    };
    if verbose() {
      let bulge = wall
        .iter()
        .filter(|p| p.0 > left_x && p.0 < right_x)
        .map(|p| p.1 - dock.face_y(p.0))
        .fold(0.0, f64::max);
      println!(
        "tray dock: notch x {:.2}..{:.2}, y {:.2}..{:.2}, wall stands {:.2} proud of the faceplate at most",
        left_x, right_x, dock.left.1, dock.right.1, bulge
      );
      println!(
        "tray: faceplate {:.1} x {:.1}, usb centre x {:.2}, jack centre x {:.2}, board y {:.2}..{:.2}",
        face_width(), face_height(), dock.feather_x(), dock.jack_x(),
        dock.board_front_y() - FEATHER_LENGTH, dock.board_front_y()
      );
    }
    dock
  }

  /// y of the outer face of the faceplate at `x`.
  fn face_y(&self, x: f64) -> f64 {
    let s = (x - self.left.0) / (self.right.0 - self.left.0);
    self.left.1 + s * (self.right.1 - self.left.1)
  }

  /// Parallelogram from `x0` to `x1`, between depths `d0` and `d1`.
  fn quad(&self, x0: f64, x1: f64, d0: f64, d1: f64) -> Vec<(f64, f64)> {
    vec![
      (x0, self.face_y(x0) - d1),
      (x1, self.face_y(x1) - d1),
      (x1, self.face_y(x1) - d0),
      (x0, self.face_y(x0) - d0),
    ]
  }

  fn block(&self, x0: f64, x1: f64, d0: f64, d1: f64, z0: f64, z1: f64) -> Shape {
    convex_prism(&self.quad(x0, x1, d0, d1), z0, z1)
  }

  /// A slab of the faceplate between depths `d0` and `d1`: `outline` and
  /// `holes` are polygons in (x, z), as seen from outside the case.
  fn face_layer(&self, outline: &[(f64, f64)], holes: &[Vec<(f64, f64)>], d0: f64, d1: f64) -> Shape {
    let lift = |poly: &[(f64, f64)]| -> Vec<Point3> {
      poly.iter().map(|&(x, z)| Point3::new(x, self.face_y(x) - d0, z)).collect()
    };
    let holes: Vec<Vec<Point3>> = holes.iter().map(|h| lift(h)).collect();
    sweep_face(&lift(outline), &holes, Vector3::new(0.0, -(d1 - d0), 0.0))
  }

  // Layout across the faceplate, left to right: jack, then Feather.
  fn face_left(&self) -> f64 {
    self.left.0 + NOTCH_CLEARANCE
  }
  fn face_right(&self) -> f64 {
    self.right.0 - NOTCH_CLEARANCE
  }
  fn jack_x(&self) -> f64 {
    self.face_left() + FACE_SIDE_MARGIN + JACK_WALL + FIT + JACK_WIDTH / 2.0
  }
  fn feather_x(&self) -> f64 {
    self.face_right() - FACE_SIDE_MARGIN - FIT - FEATHER_WIDTH / 2.0
  }
  /// y of the USB end of the board.
  fn board_front_y(&self) -> f64 {
    let half = FEATHER_WIDTH / 2.0;
    let x = self.feather_x();
    self.face_y(x - half).min(self.face_y(x + half)) - FEATHER_SETBACK
  }
  /// z of the underside of the jack. Kept low: the jack goes in upside down
  /// and its pins must stay well clear of the switches above.
  fn jack_z(&self) -> f64 {
    JACK_PEDESTAL
  }

  /// The notch to cut out of the back wall.
  pub fn notch(&self) -> Shape {
    self.block(
      self.left.0,
      self.right.0,
      -4.0,
      CHANNEL_BACK,
      -1.0,
      face_height() + NOTCH_CLEARANCE,
    )
  }

  /// The dock: added to the case around the notch. It replaces the wall next
  /// to the notch with straight surfaces for the faceplate to sit against, has
  /// the channels for the flanges, and a lintel over the notch.
  pub fn frame(&self) -> Shape {
    let notch_top = face_height() + NOTCH_CLEARANCE;
    let top = notch_top + DOCK_LINTEL;
    let (l, r) = (self.left.0, self.right.0);
    let web = FLANGE_REACH + 0.1;
    let side = self.side_right;
    Shape::union_all(vec![
      // In the wall, next to the notch: the flanges pull against these.
      self.block(l - DOCK_SIDE, l, 0.3, PLUG_DEPTH, 0.0, top),
      self.block(r, r + side, 0.3, PLUG_DEPTH, 0.0, top),
      // Past the ends of the flanges.
      self.block(l - DOCK_SIDE, l - web, PLUG_DEPTH - 0.5, DOCK_DEPTH, 0.0, top),
      self.block(r + web, r + side, PLUG_DEPTH - 0.5, DOCK_DEPTH, 0.0, top),
      // Behind the flanges: these stop the tray being pushed in.
      self.block(l - DOCK_SIDE, l - 0.3, CHANNEL_BACK, DOCK_DEPTH, 0.0, top),
      self.block(r + 0.3, r + side, CHANNEL_BACK, DOCK_DEPTH, 0.0, top),
      // Lintel.
      self.block(l - DOCK_SIDE, r + side, 0.3, PLUG_DEPTH, notch_top, top),
    ])
  }

  /// The tray, in place in the case. Without `jack` it carries the Feather
  /// only, in the same place: the side of the notch with the most headroom
  /// under the key plate.
  pub fn tray(&self, jack: bool) -> Shape {
    let mut shapes = Vec::new();
    let height = face_height();
    let (x0, x1) = (self.face_left(), self.face_right());
    let (xj, xf) = (self.jack_x(), self.feather_x());
    let zt = pcb_top();
    let zj = self.jack_z();

    //
    // Faceplate
    //
    let skin_outline = vec![(x0, 0.0), (x1, 0.0), (x1, height), (x0, height)];
    // Round hole for the barrel of the jack (holes are wound clockwise).
    let barrel_r = JACK_BARREL_DIAMETER / 2.0 + 0.2;
    let barrel_z = zj + JACK_HEIGHT / 2.0;
    let steps = 32;
    let barrel_hole: Vec<(f64, f64)> = (0..steps)
      .map(|k| {
        let a = -2.0 * std::f64::consts::PI * k as f64 / steps as f64;
        (xj + barrel_r * a.cos(), barrel_z + barrel_r * a.sin())
      })
      .collect();

    // Outline with the pocket for the body of the jack, open at the top.
    let jack_half = JACK_WIDTH / 2.0 + FIT;
    let body_outline = if !jack {
      skin_outline.clone()
    } else {
      vec![
        (x0, 0.0),
        (x1, 0.0),
        (x1, height),
        (xj + jack_half, height),
        (xj + jack_half, zj),
        (xj - jack_half, zj),
        (xj - jack_half, height),
        (x0, height),
      ]
    };

    // Slot for the USB end of the board, from the underside of the PCB up.
    // The board rests on the faceplate below it.
    let slot_half = FEATHER_WIDTH / 2.0 + FIT;
    let (slot_z0, slot_z1) = (zt - PCB_THICKNESS, zt + SLOT_ABOVE_PCB_TOP);
    let slot = vec![
      (xf - slot_half, slot_z0),
      (xf - slot_half, slot_z1),
      (xf + slot_half, slot_z1),
      (xf + slot_half, slot_z0),
    ];

    // The layers overlap a little so they fuse into one body when sliced.
    let skin_holes = if jack { vec![slot.clone(), barrel_hole] } else { vec![slot.clone()] };
    shapes.push(self.face_layer(&skin_outline, &skin_holes, 0.0, JACK_SKIN + 0.02));
    shapes.push(self.face_layer(&body_outline, &[slot], JACK_SKIN, FLANGE_BACK));
    // Flanges.
    shapes.push(self.block(x0 - NOTCH_CLEARANCE - FLANGE_REACH, x0 + 0.5, FLANGE_FRONT, FLANGE_BACK, 0.0, height));
    shapes.push(self.block(x1 - 0.5, x1 + NOTCH_CLEARANCE + FLANGE_REACH, FLANGE_FRONT, FLANGE_BACK, 0.0, height));

    // A box with its front against the back of the faceplate.
    let behind_face = |xa: f64, xb: f64, y_back: f64, z0: f64, z1: f64| -> Shape {
      let d = FLANGE_BACK - 0.3;
      convex_prism(
        &[(xa, y_back), (xb, y_back), (xb, self.face_y(xb) - d), (xa, self.face_y(xa) - d)],
        z0,
        z1,
      )
    };

    //
    // TRRS jack: a pedestal with a cradle on top. The jack goes in upside
    // down (pins up, for soldering).
    //
    let jack_front = self.face_y(xj - jack_half).min(self.face_y(xj + jack_half)) - JACK_SKIN - 0.1;
    let jack_back = jack_front - JACK_LENGTH - FIT;
    let cradle_half = jack_half + JACK_WALL;
    let cradle_top = zj + JACK_HEIGHT - 1.0;
    if jack {
      shapes.push(behind_face(xj - cradle_half, xj + cradle_half, jack_back - JACK_WALL, 0.0, zj));
      shapes.push(behind_face(xj - cradle_half, xj - jack_half, jack_back - JACK_WALL, zj - 0.2, cradle_top));
      shapes.push(behind_face(xj + jack_half, xj + cradle_half, jack_back - JACK_WALL, zj - 0.2, cradle_top));
      shapes.push(convex_prism(
        &[
          (xj - cradle_half, jack_back - JACK_WALL),
          (xj + cradle_half, jack_back - JACK_WALL),
          (xj + cradle_half, jack_back),
          (xj - cradle_half, jack_back),
        ],
        zj - 0.2,
        zj + JACK_BACK_LIP,
      ));
      // Tie the floor of the Feather to the pedestal.
      shapes.push(behind_face(xj, xf, jack_back - JACK_WALL, 0.0, FLOOR_THICKNESS));
    }

    //
    // Feather
    //
    let board_front = self.board_front_y();
    let board_back = board_front - FEATHER_LENGTH;
    let floor_half = FEATHER_WIDTH / 2.0 + FIT + FACE_SIDE_MARGIN;
    shapes.push(behind_face(
      xf - floor_half,
      xf + floor_half,
      board_back - FIT - FACE_SIDE_MARGIN,
      0.0,
      FLOOR_THICKNESS,
    ));

    // A standoff with a peg under each of the four mounting holes. The ones at
    // the USB end are (mostly) inside the faceplate.
    let hole_x = FEATHER_WIDTH / 2.0 - FEATHER_HOLE_INSET;
    let standoff_z = FLOOR_THICKNESS - 0.2;
    let standoff = cylinder(STANDOFF_HEIGHT + 0.2, 2.0, 24, false).translate_z(standoff_z);
    let peg = cylinder(PEG_HEIGHT + 0.2, PEG_DIAMETER / 2.0, 16, false).translate_z(zt - PCB_THICKNESS - 0.2);
    for x in [xf - hole_x, xf + hole_x] {
      for y in [board_front - FEATHER_HOLE_INSET, board_back + FEATHER_HOLE_INSET] {
        shapes.push(standoff.translate(x, y, 0.0));
        shapes.push(peg.translate(x, y, 0.0));
      }
    }

    Shape::union_all(shapes)
  }

  /// Offset that moves the tray from its place in the case to the origin, for
  /// printing.
  pub fn print_offset(&self) -> (f64, f64) {
    (-self.left.0, -self.board_front_y() + FEATHER_LENGTH + 5.0)
  }
}
