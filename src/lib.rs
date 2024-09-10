use clap::Parser;
use pyo3::prelude::*;

pub mod coords;
pub mod fov;
pub mod parsing_utils;
pub mod rendering;
pub mod star;

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
