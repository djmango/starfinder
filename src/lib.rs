pub mod coords;
pub mod fov;
pub mod parsing_utils;
pub mod rendering;
pub mod star;

pub mod render;
pub mod star_catalog;
pub mod types;

use clap::Parser;
use pyo3::prelude::*;
use std::time::Instant;

use crate::render::render_stars;
use crate::star_catalog::read_stars;

#[pyclass]
#[derive(Parser, Debug, Clone)]
pub struct StarCatalogArgs {
    #[pyo3(get, set)]
    pub file: String,
    #[pyo3(get, set)]
    pub display_count: usize,
    #[pyo3(get, set)]
    pub min_ra: f64,
    #[pyo3(get, set)]
    pub max_ra: f64,
    #[pyo3(get, set)]
    pub min_dec: f64,
    #[pyo3(get, set)]
    pub max_dec: f64,
    #[pyo3(get, set)]
    pub max_magnitude: f64,
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
        file: String,
        display_count: usize,
        min_ra: f64,
        max_ra: f64,
        min_dec: f64,
        max_dec: f64,
        max_magnitude: f64,
        width: u32,
        height: u32,
        output: String,
    ) -> Self {
        Self {
            file,
            display_count,
            min_ra,
            max_ra,
            min_dec,
            max_dec,
            max_magnitude,
            width,
            height,
            output,
        }
    }
}

pub fn process_star_catalog(args: StarCatalogArgs) -> Result<(), Box<dyn std::error::Error>> {
    println!("Reading stars from: {}", args.file);
    println!("RA range: {} to {}", args.min_ra, args.max_ra);
    println!("Dec range: {} to {}", args.min_dec, args.max_dec);
    println!("Max magnitude: {}", args.max_magnitude);

    let start = Instant::now();
    let stars = read_stars(
        &args.file,
        args.min_ra,
        args.max_ra,
        args.min_dec,
        args.max_dec,
        args.max_magnitude,
    )?;
    let read_duration = start.elapsed();

    println!("Time taken to read and filter stars: {:?}", read_duration);
    println!("Total stars after filtering: {}", stars.len());

    println!("\nFirst {} stars:", args.display_count);
    for (i, star) in stars.iter().enumerate() {
        if i >= args.display_count && args.display_count != 0 {
            break;
        }
        println!(
            "Star {}: RA={:.2}, Dec={:.2}, Mag={:.2}",
            i, star.ra_deg, star.de_deg, star.mag
        );
    }

    let render_start = Instant::now();
    let img = render_stars(
        &stars,
        args.width,
        args.height,
        args.min_ra,
        args.max_ra,
        args.min_dec,
        args.max_dec,
    );
    img.save(&args.output)?;
    let render_duration = render_start.elapsed();

    println!("Time taken to render and save image: {:?}", render_duration);
    println!("Image saved as: {}", args.output);
    println!("Total time elapsed: {:?}", start.elapsed());

    Ok(())
}

#[pyfunction]
fn process_star_catalog_py(args: StarCatalogArgs) -> PyResult<()> {
    process_star_catalog(args)
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
