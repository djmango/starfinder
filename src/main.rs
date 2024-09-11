use clap::Parser;

use starfinder::parse_and_render::parse_and_render;
use starfinder::types::StarCatalogArgs;

/// CLI Arguments
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
pub struct Args {
    /// Path to the optimized catalog file. The optimize binary should be run before running this to
    /// generate the optimized data file.
    #[arg(short, long, default_value = "data/optimized.dat")]source: String,

    /// Right Ascension of camera view center point (degrees)
    #[arg(long, default_value_t = 180.0)]
    center_ra: f64,

    /// Declination of camera view center point (degrees)
    #[arg(long, default_value_t = 0.0)]
    center_dec: f64,

    /// Width of field of view. With 0 roll, corresponds to right ascension (degrees)
    #[arg(long, default_value_t = 60.0)]
    fov_w: f64,

    /// Height of field of view. With 0 roll, corresponds to declination (degrees)
    #[arg(long, default_value_t = 45.0)]
    fov_h: f64,

    /// Roll of the camera view (degrees)
    #[arg(long, default_value_t = 0.0)]
    roll: f64,

    /// Maximum visual magnitude (lower is brighter)
    #[arg(long, default_value_t = 12.0)]
    max_magnitude: f64,

    /// Targeted wavelength - critical for airy disc rendering (nanometers). Default to visible
    #[arg(long, default_value_t = 540.0)]
    lambda_nm: f64,

    /// Camera pixel size in meters (should be tiny, like e-6). Default is 3e-6, assuming higher
    /// precision optics
    #[arg(long, default_value_t = 3e-6)]
    pixel_size_m: f64,

    /// Output image width in pixels
    #[arg(long, default_value_t = 800)]
    width: u32,

    /// Output image height in pixels
    #[arg(long, default_value_t = 800)]
    height: u32,

    /// Output image file name
    #[arg(short, long, default_value = "renders/star_map.png")]
    output: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cmd_args = Args::parse();
    let args = StarCatalogArgs {
        source: cmd_args.source,
        center_ra: cmd_args.center_ra,
        center_dec: cmd_args.center_dec,
        fov_w: cmd_args.fov_w,
        fov_h: cmd_args.fov_h,
        roll: cmd_args.roll,
        max_magnitude: cmd_args.max_magnitude,
        lambda_nm: cmd_args.lambda_nm,
        pixel_size_m: cmd_args.pixel_size_m,
        width: cmd_args.width,
        height: cmd_args.height,
        output: cmd_args.output,
    };

    parse_and_render(&args)
}
