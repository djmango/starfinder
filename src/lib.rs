pub mod fov;
pub mod parsing_utils;
pub mod rendering;
pub mod types;

use crate::parsing_utils::read_stars;
use crate::rendering::render_stars;
use crate::types::EquatorialCoords;

use clap::Parser;
use pyo3::prelude::*;
use std::time::Instant;

#[pyclass]
#[derive(Parser, Debug, Clone)]
pub struct StarCatalogArgs {
    #[pyo3(get, set)]
    pub source: String,
    #[pyo3(get, set)]
    pub center_ra: f64,
    #[pyo3(get, set)]
    pub center_dec: f64,
    #[pyo3(get, set)]
    pub fov_w: f64,
    #[pyo3(get, set)]
    pub fov_h: f64,
    #[pyo3(get, set)]
    pub roll: f64,
    #[pyo3(get, set)]
    pub max_magnitude: f64,
    #[pyo3(get, set)]
    pub lambda_nm: f64,
    #[pyo3(get, set)]
    pub pixel_size_m: f64,
    #[pyo3(get, set)]
    pub width: u32,
    #[pyo3(get, set)]
    pub height: u32,
    #[pyo3(get, set)]
    pub output: String,
}

#[pymethods]
impl StarCatalogArgs {
    #[new]
    fn new(
        source: String,
        center_ra: f64,
        center_dec: f64,
        fov_w: f64,
        fov_h: f64,
        roll: f64,
        max_magnitude: f64,
        lambda_nm: f64,
        pixel_size_m: f64,
        width: u32,
        height: u32,
        output: String,
    ) -> Self {
        Self {
            source,
            center_ra,
            center_dec,
            fov_w,
            fov_h,
            roll,
            max_magnitude,
            lambda_nm,
            pixel_size_m,
            width,
            height,
            output,
        }
    }
}

pub fn process_star_catalog(args: &StarCatalogArgs) -> Result<(), Box<dyn std::error::Error>> {
    let run_start = Instant::now();
    let center_ra = args.center_ra.to_radians();
    let center_dec = args.center_dec.to_radians();
    let roll = args.roll.to_radians();
    let fov_w = args.fov_w.to_radians();
    let fov_h = args.fov_h.to_radians();

    // 1) Rotate FOV by specified roll
    let get_fov_start = Instant::now();
    let center = EquatorialCoords {
        ra: center_ra,
        dec: center_dec,
    };
    let rolled_fov = fov::get_fov(center, fov_w, fov_h, roll);
    println!("Total FOV retrieval time: {:?}", get_fov_start.elapsed());

    // 2) Read stars and filter against rolled_fov to create subset of stars in view of the image
    let read_stars_start = Instant::now();
    let stars_in_fov = read_stars(&args.source, rolled_fov, args.max_magnitude)?;
    println!(
        "Total time to read and parse stars: {:?}",
        read_stars_start.elapsed()
    );

    // 3) Render stars in FOV
    let render_stars_start = Instant::now();
    let img = render_stars(
        stars_in_fov,
        args.width,
        args.height,
        center,
        fov_w,
        fov_h,
        roll,
    );
    img.save(&args.output)?;
    println!(
        "Total parse and write stars: {:?}",
        render_stars_start.elapsed()
    );

    println!("Total run time elapsed: {:?}", run_start.elapsed());

    Ok(())
}

#[pyfunction]
fn process_star_catalog_py(args: StarCatalogArgs) -> PyResult<()> {
    process_star_catalog(&args)
        .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
}

/// A Python module implemented in Rust. The name of this function must match
/// the `lib.name` setting in the `Cargo.toml`, else Python will not be able to
/// import the module.
#[pymodule]
fn starfinder(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<StarCatalogArgs>()?;
    m.add_function(wrap_pyfunction!(process_star_catalog_py, m)?)?;
    Ok(())
}
