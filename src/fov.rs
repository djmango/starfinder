use nalgebra::{SMatrix, Vector3};
use std::collections::HashSet;
use std::f64::consts::PI;

use crate::types::{CartesianCoords, EquatorialCoords};

pub const GRID_RESOLUTION: f64 = 360.0;

pub fn get_fov(
    center: EquatorialCoords,
    fov_w: f64,
    fov_h: f64,
    roll: f64,
) -> HashSet<EquatorialCoords> {
    // Config params for function
    let view_init_center = EquatorialCoords {
        ra: 180.0_f64.to_radians(),
        dec: 0.0_f64.to_radians(),
    };
    let ra_dif = center.ra - view_init_center.ra;
    let dec_dif = center.dec - view_init_center.dec;
    // Calculate FOV bounds in cartesian coords
    let fov_w_half = fov_w / 2.0;
    let fov_h_half = fov_h / 2.0;

    let step_size = 2.0 * PI / GRID_RESOLUTION;
    let bottom_left = EquatorialCoords {
        ra: view_init_center.ra - fov_w_half,
        dec: view_init_center.dec - fov_h_half,
    };
    let fov_steps_w = ((fov_w / step_size).ceil() + 1.0) as i32;
    let fov_steps_h = ((fov_h / step_size).ceil() + 1.0) as i32;
    let mut scatter_shot: Vec<CartesianCoords> = Vec::new();

    for y in 0..fov_steps_h {
        for x in 0..fov_steps_w {
            scatter_shot.push(
                EquatorialCoords {
                    ra: bottom_left.ra + (x as f64 * step_size),
                    dec: bottom_left.dec + (y as f64 * step_size),
                }
                .to_cartesian(),
            );
        }
    }

    let c_cartesian = center.to_cartesian();
    let y_roll_axis = CartesianCoords {
        x: -c_cartesian.y,
        y: c_cartesian.x,
        z: 0.0,
    };

    let x_roll = SMatrix::<f64, 3, 3>::new(
        // Row 1
        roll.cos(),
        ra_dif.sin(),
        0.0,
        // Row 2
        ra_dif.sin(),
        ra_dif.cos(),
        0.0,
        // Row 3
        0.0,
        0.0,
        (1.0 - ra_dif.cos()) + ra_dif.cos(),
    );
    let y_roll = SMatrix::<f64, 3, 3>::new(
        // Row 1
        y_roll_axis.x.powi(2) * (1.0 - dec_dif.cos()) + dec_dif.cos(),
        y_roll_axis.x * y_roll_axis.y * (1.0 - dec_dif.cos()) - (y_roll_axis.z * dec_dif.sin()),
        y_roll_axis.x * y_roll_axis.z * (1.0 - dec_dif.cos()) + (y_roll_axis.y * dec_dif.sin()),
        // Row 2
        y_roll_axis.x * y_roll_axis.y * (1.0 - dec_dif.cos()) + (y_roll_axis.z * dec_dif.sin()),
        y_roll_axis.y.powi(2) * (1.0 - dec_dif.cos()) + dec_dif.cos(),
        y_roll_axis.y * y_roll_axis.z * (1.0 - dec_dif.cos()) - (y_roll_axis.x * dec_dif.sin()),
        // Row 3
        y_roll_axis.x * y_roll_axis.z * (1.0 - dec_dif.cos()) - (y_roll_axis.y * dec_dif.sin()),
        y_roll_axis.y * y_roll_axis.z * (1.0 - dec_dif.cos()) + (y_roll_axis.x * dec_dif.sin()),
        y_roll_axis.z.powi(2) * (1.0 - dec_dif.cos()) + dec_dif.cos(),
    );
    let z_roll = SMatrix::<f64, 3, 3>::new(
        // Row 1
        c_cartesian.x.powi(2) * (1.0 - roll.cos()) + roll.cos(),
        c_cartesian.x * c_cartesian.y * (1.0 - roll.cos()) - (c_cartesian.z * roll.sin()),
        c_cartesian.x * c_cartesian.z * (1.0 - roll.cos()) + (c_cartesian.y * roll.sin()),
        // Row 2
        c_cartesian.x * c_cartesian.y * (1.0 - roll.cos()) + (c_cartesian.z * roll.sin()),
        c_cartesian.y.powi(2) * (1.0 - roll.cos()) + roll.cos(),
        c_cartesian.y * c_cartesian.z * (1.0 - roll.cos()) - (c_cartesian.x * roll.sin()),
        // Row 3
        c_cartesian.x * c_cartesian.z * (1.0 - roll.cos()) - (c_cartesian.y * roll.sin()),
        c_cartesian.y * c_cartesian.z * (1.0 - roll.cos()) + (c_cartesian.x * roll.sin()),
        c_cartesian.z.powi(2) * (1.0 - roll.cos()) + roll.cos(),
    );

    let transform = x_roll * y_roll * z_roll;
    let mut final_grid: HashSet<EquatorialCoords> = HashSet::new();
    for p in scatter_shot {
        let vec: Vector3<f64> = Vector3::new(p.x, p.y, p.z);
        let transformed = transform * vec;
        let grid_coord = CartesianCoords {
            x: transformed[(0, 0)],
            y: transformed[(1, 0)],
            z: transformed[(2, 0)],
        }
        .to_equatorial()
        .to_grid();
        final_grid.insert(grid_coord);
    }

    final_grid
}
/*
pub fn equatorial_to_grid(p: &EquatorialCoords) -> EquatorialCoords {
    EquatorialCoords {
        ra: ((p.ra / 2.0 * PI) * (1.0 - (2.0 * p.dec / PI).abs()) * APPROX_RES).round(),
        dec: ((2.0 * p.dec / PI) * APPROX_RES).round(),
    }
}*/
