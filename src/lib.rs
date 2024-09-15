pub mod constants;
pub mod fov;
pub mod optimize;
pub mod parse_and_render;
pub mod parsing_utils;
pub mod rendering;
pub mod types;

// use crate::parse_and_render::render_to_file;
// use crate::types::StarCatalogArgs;

// use pyo3::prelude::*;

// #[pyfunction]
// fn render_to_file_py(args: StarCatalogArgs) -> PyResult<()> {
//     render_to_file(&args)
//         .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
// }

// #[pyfunction]
// fn read_and_render_py(args: StarCatalogArgs) -> PyResult<ImageBuffer<Rgb<u8>, Vec<u8>>> {
//     parse_and_render::read_and_render(&args)
//         .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
// }

// A Python module implemented in Rust. The name of this function must match
// the `lib.name` setting in the `Cargo.toml`, else Python will not be able to
// import the module.
// #[pymodule]
// fn starfinder(m: &Bound<'_, PyModule>) -> PyResult<()> {
//     m.add_class::<StarCatalogArgs>()?;
//     m.add_function(wrap_pyfunction!(render_to_file_py, m)?)?;
//     Ok(())
// }
