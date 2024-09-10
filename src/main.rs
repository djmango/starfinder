use clap::Parser;
use std::path::PathBuf;
use std::time::Instant;

use starfinder::fov::get_fov;
use starfinder::parsing_utils::read_stars;
use starfinder::rendering::render_stars;
use starfinder::types::EquatorialCoords;

/// CLI Arguments
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
pub struct Args {
    /// Path to the Tycho-2 catalog file
    #[arg(
        short,
        long,
        value_name = "FILE",
        default_value = "data/tycho2/catalog.dat"
    )]
    source: PathBuf,

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
    #[arg(long, default_value_t = 600)]
    height: u32,

    /// Output image file name
    #[arg(short, long, default_value = "star_map.png")]
    output: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let run_start = Instant::now();
    let args = Args::parse();
    let center_ra = args.center_ra.to_radians();
    let center_dec = args.center_dec.to_radians();
    let roll = args.roll.to_radians();
    let fov_w = args.fov_w.to_radians();
    let fov_h = args.fov_h.to_radians();

    /*println!("================ Cmd args list ===============");
    println!("Reading stars from: {:?}", args.source);
    println!("Center - RA (deg): {}", args.center_ra);
    println!("Center - RA (rad): {}", center_ra);
    println!("Center - Dec (deg): {}", args.center_dec);
    println!("Center - Dec (rad): {}", center_dec);
    println!("FOV width (deg): {}", args.fov_w_deg);
    println!("FOV width (rad): {}", fov_w);
    println!("FOV height (deg): {}", args.fov_h_deg);
    println!("FOV height (rad): {}", fov_h);
    println!("Roll (deg): {}", args.roll_deg);
    println!("Roll (rad): {}", roll);
    println!("Max magnitude: {}", args.max_magnitude);
    println!("Lambda (wavelength) nm: {}", args.lambda_nm);
    println!("Pixel size (meters): {}", args.pixel_size_m);
    println!("Output image width: {}", args.width);
    println!("Output image height: {}", args.height);
    println!("Output image height: {}", args.height);
    println!("Output filename: {}", args.output);
    println!("============ End of cmd args list ============");*/

    // 1) Rotate FOV by specified roll
    let get_fov_start = Instant::now();
    let center = EquatorialCoords {
        ra: center_ra,
        dec: center_dec,
    };
    let rolled_fov = get_fov(center, fov_w, fov_h, roll);
    println!("Total FOV retrieval time: {:?}", get_fov_start.elapsed());

    // 2) Read stars and filter against rolled_fov to create subset of stars in view of the image
    let read_stars_start = Instant::now();
    let stars_in_fov = read_stars(args.source, rolled_fov, args.max_magnitude)?;
    println!(
        "Total time to read and parse stars: {:?}",
        read_stars_start.elapsed()
    );

    // 4) Render stars in FOV
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
