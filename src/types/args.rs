use clap::Parser;
use pyo3::prelude::{pyclass, pymethods};

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