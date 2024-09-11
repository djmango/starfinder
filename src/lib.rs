pub mod fov;
pub mod parse_and_render;
pub mod parsing_utils;
pub mod rendering;
pub mod types;

use crate::types::StarCatalogArgs;
use crate::parse_and_render::parse_and_render;

use pyo3::prelude::*;

#[pyfunction]
fn process_star_catalog_py(args: StarCatalogArgs) -> PyResult<()> {
    parse_and_render(&args)
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
