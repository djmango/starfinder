use std::f64::consts::PI;
use clap::Parser;
use csv::ReaderBuilder;
use image::{ImageBuffer, Rgb};
use nalgebra::SMatrix;
use serde::{Deserialize, Serialize};
use spec_math::cephes64::j1;
use std::fs::File;
use std::io;
use std::path::PathBuf;
use std::time::Instant;
use thiserror::Error;

/// Represents a star with its right ascension, declination, and magnitude.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Star {
    coords: EquatorialCoords,
    mag: f64,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct EquatorialCoords {
    ra_deg: f64,
    dec_deg: f64,
}

pub struct CartesianCoords {
    x: f64,
    y: f64,
    z: f64,
}

pub struct BoundsEquatorial {
    max_ra: f64,
    min_ra: f64,
    max_dec: f64,
    min_dec: f64,
}

/// Errors that can occur during star catalog reading.
#[derive(Error, Debug)]
pub enum CatalogError {
    #[error("IO error: {0}")]
    Io(#[from] io::Error),
    #[error("CSV parsing error: {0}")]
    Csv(#[from] csv::Error),
    #[error("Missing field: {0}")]
    MissingField(String),
    #[error("Parse error: {0}")]
    Parse(String),
    #[error("Missing magnitude")]
    MissingMagnitude,
}

/// CLI Arguments
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the Tycho-2 catalog file
    #[arg(
        short,
        long,
        value_name = "FILE",
        default_value = "data/tycho2/catalog.dat"
    )]
    file: PathBuf,

    /// Number of stars to display (0 for all)
    #[arg(short, long, default_value_t = 10)]
    display_count: usize,

    /// Right Ascension of camera view center point (degrees)
    #[arg(long, default_value_t = 0.0)]
    center_ra: f64,

    /// Declination of camera view center point (degrees)
    #[arg(long, default_value_t = 360.0)]
    center_dec: f64,

    /// Width of field of view. With 0 roll, corresponds to right ascension (degrees)
    #[arg(long, default_value_t = 0.0)]
    fov_w_deg: f64,

    /// Height of field of view. With 0 roll, corresponds to declination (degrees)
    #[arg(long, default_value_t = 0.0)]
    fov_h_deg: f64,

    /// Roll of the camera view (degrees)
    #[arg(long, default_value_t = 0.0)]
    roll_deg: f64,

    /// Maximum visual magnitude (lower is brighter)
    #[arg(long, default_value_t = 1.0)]
    max_magnitude: f64,

    /// B magnitude wavelength - only used if using airy disc rendering (nanometers)
    #[arg(long, default_value_t = 442)]
    lambda_b_nm: f64,

    /// V magnitude wavelength - only used if using airy disc rendering (nanometers)
    #[arg(long, default_value_t = 540)]
    lambda_v_nm: f64,

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

pub fn to_cartesian(src: EquatorialCoords) -> CartesianCoords {
    CartesianCoords {
        x: src.dec_deg.cos() * src.ra_deg.cos(),
        y: src.dec_deg.cos() * src.ra_deg.sin(),
        z: src.dec_deg.sin(),
    }
}

pub fn to_equatorial(src: CartesianCoords) -> EquatorialCoords {
    EquatorialCoords {
        ra_deg: src.y.atan2(src.x),
        dec_deg: src.z.asin(),
    }
}

pub fn fov_outer_bounds(
    center_ra_deg: f64,
    center_dec_deg: f64,
    fov_w_deg: f64,
    fov_h_deg: f64,
    roll_deg: f64,
) -> BoundsEquatorial {
    // Calculate FOV bounds in cartesian coords
    let fov_w_half = fov_w_deg / 2.0;
    let fov_h_half = fov_h_deg / 2.0;

    let fov: [CartesianCoords; 4] = [
        to_cartesian(EquatorialCoords {
            ra_deg: center_ra_deg - fov_w_half,
            dec_deg: center_dec_deg + fov_h_half,
        }),
        to_cartesian(EquatorialCoords {
            ra_deg: center_ra_deg + fov_w_half,
            dec_deg: center_dec_deg + fov_h_half,
        }),
        to_cartesian(EquatorialCoords {
            ra_deg: center_ra_deg - fov_w_half,
            dec_deg: center_dec_deg - fov_h_half,
        }),
        to_cartesian(EquatorialCoords {
            ra_deg: center_ra_deg + fov_w_half,
            dec_deg: center_dec_deg - fov_h_half,
        })
    ];

    // Generate roll matrix and apply roll to coordinates, resulting in rolled FOV corner points
    let roll_matrix: SMatrix<f64, 3, 3> = SMatrix::new(
        roll_deg.cos(), -roll_deg.sin(), 0.0,
        roll_deg.sin(), roll_deg.cos(),  0.0,
        0.0,            0.0,             1.0,
    );

    let fov_rolled: [EquatorialCoords; 4] = fov
        // Apply the roll matrix, giving new cartesian coords of rolled FOV (use unit sphere z of 1
        .map(|pt| -> SMatrix<f64, 3, 1> { roll_matrix * SMatrix::new(pt.x, pt.y, 1) })
        // Convert rolled coords back into equatorial
        .map(|rolled| to_equatorial(
            CartesianCoords { x: rolled[(0,0)], y: rolled[(1,0)], z: rolled[(2,0)]}
        ));

    // Extract ra and dec values as sets to find min/max and return as Bounds
    let rolled_ra_vals = fov_rolled.iter().map(|c| c.ra_deg);
    let rolled_dec_vals = fov_rolled.iter().map(|c| c.dec_deg);

    BoundsEquatorial {
        max_ra: rolled_ra_vals.iter().fold(f64::NEG_INFINITY, f64::max),
        min_ra: rolled_ra_vals.iter().fold(f64::INFINITY, f64::min),
        max_dec: rolled_dec_vals.iter().fold(f64::NEG_INFINITY, f64::max),
        min_dec: rolled_dec_vals.iter().fold(f64::INFINITY, f64::min),
    }
}

/// Reads and filters stars from the Tycho-2 catalog file.
/// https://heasarc.gsfc.nasa.gov/w3browse/all/tycho2.html
/// http://tdc-www.harvard.edu/catalogs/tycho2.format.html
pub fn read_stars<P: AsRef<std::path::Path>>(
    path: P,
    center_ra_deg: f64,
    center_dec_deg: f64,
    fov_w_deg: f64,
    fov_h_deg: f64,
    roll_deg: f64,
    max_magnitude: f64,
) -> Result<Vec<Star>, CatalogError> {
    let file = File::open(path)?;
    let reader = io::BufReader::new(file);
    let mut csv_reader = ReaderBuilder::new()
        .delimiter(b'|')
        .has_headers(false)
        .from_reader(reader);

    let mut stars = Vec::new();
    let mut skipped_rows = 0;
    // Define bounding box of FOV in case of roll.
    let bound_min_ra: f64 = center_ra_deg - (fov_w_deg / 2.0);
    let bound_max_ra: f64 = center_ra_deg + (fov_w_deg / 2.0);
    let bound_min_dec: f64 = center_dec_deg - (fov_h_deg / 2.0);
    let bound_max_dec: f64 = center_dec_deg + (fov_h_deg / 2.0);

    println!("bound_min_ra {}", bound_min_ra);
    println!("bound_max_ra {}", bound_max_ra);
    println!("bound_min_dec {}", bound_min_dec);
    println!("bound_max_dec {}", bound_max_dec);
    // Build bounding box based on camera roll - outer box containing rolled FOV
    // Filter stars out of the outer bounding box before doing testing whether they fit inside the
    // actual FOV
    let bounds = fov_outer_bounds(center_ra_deg, center_dec_deg, fov_w_deg, fov_h_deg, roll_deg);
    /*
        for (i, result) in csv_reader.records().enumerate() {
            let record = result?;
            match parse_star_record(&record) {
                Ok(star) => {
                    if star.ra_deg >= min_ra
                        && star.ra_deg <= max_ra
                        && star.de_deg >= min_dec
                        && star.de_deg <= max_dec
                        && star.mag <= max_magnitude
                    {
                        if i % 10000 == 0 {
                            println!(
                                "Star {}: RA={}, Dec={}, Mag={}",
                                i, star.ra_deg, star.de_deg, star.mag
                            );
                        }
                        stars.push(star);
                    }
                }
                Err(e) => {
                    skipped_rows += 1;
                    if skipped_rows <= 10 {
                        println!("Skipping row {} due to error: {:?}", i, e);
                        println!("Problematic row: {:?}", record);
                    } else if skipped_rows == 11 {
                        println!("Further skipped rows will not be printed...");
                    }
                }
            }
        }
    */
    println!("Total stars read and filtered: {}", stars.len());
    println!("Total rows skipped: {}", skipped_rows);

    Ok(stars)
}

/// Parses a single record from the catalog into a Star struct.
fn parse_star_record(record: &csv::StringRecord) -> Result<Star, CatalogError> {
    let ra = parse_field(record, 24, "RA")?;
    let dec = parse_field(record, 25, "Dec")?;
    let mag = parse_magnitude(record)?;

    Ok(Star {
        coords: EquatorialCoords {
            ra_deg: ra,
            dec_deg: dec,
        },
        mag,
    })
}

/// Parses a field from the record, returning a helpful error if parsing fails.
fn parse_field(
    record: &csv::StringRecord,
    index: usize,
    field_name: &str,
) -> Result<f64, CatalogError> {
    record
        .get(index)
        .ok_or_else(|| CatalogError::MissingField(field_name.to_string()))?
        .trim()
        .parse()
        .map_err(|_| CatalogError::Parse(format!("Failed to parse {}", field_name)))
}

/// Airy disc radius - calculate physical radius, then convert to pixel scale
fn calc_airy_disc_radius(lambda: f64, aperture_diameter: f64, focal_length: f64, pixel_size_m: f64) -> f64 {
    // Physical radius on the sensor in meters
    let size = 1.22 * lambda * focal_length / aperture_diameter;
    // Return radius in pixels
    size / pixel_size_m
}

/// Spatial frequency scaled as related to the diffraction pattern. Ooh! Ahh!
fn calc_spatial_frequency(lambda: f64, aperture_diameter: f64, focal_length: f64) -> f64 {
    2.0 * PI / lambda * aperture_diameter / focal_length
}

/// Calculate the airy disc intensity at a given radius. Spatial frequency should be pre-calcualted
/// to avoid extra operations per loop. It is given by:
/// 2.0 * PI / lambda * aperture_diameter / focal_length
/// where lambda is wavelength of light. All arguments are in meters.
///
/// Airy Disc formula:
/// I(r) = I_0 * (2 * J1(kr) / kr)^2
/// Where:
/// I(r)  = Intensity at distance r from the center of the airy disc
/// I_0   = Max intensity at the center of the disc (the star itself/point light source, i.e. r = 0)
/// J1(x) = First order Bessel function of the first kind (the magic sauce)
/// k     = 2 * pi / wavelength * (aperture_diameter / focal_length) -- incorporates camera params
/// r     = Radial distance from the center of the disc

fn calc_airy_intensity_at_radius(r: f64, spatial_frequency: f64) -> f64 {
    // Bessel argument, which will give us our intensity at radius r
    let kr = spatial_frequency * r;

    // At the center, we're at full intensity
    if (kr == 0.0) {
        return 1.0;
    }

    let bessel = j1(kr);
    let intensity = (2.0 * bessel / kr).powi(2);
    intensity
}

fn parse_magnitude(record: &csv::StringRecord) -> Result<f64, CatalogError> {
    let bt_mag = parse_field(record, 17, "BT magnitude").ok();
    let vt_mag = parse_field(record, 19, "VT magnitude").ok();

    // println!("Debug: BT_Mag = {:?}, VT_Mag = {:?}", bt_mag, vt_mag);

    match (bt_mag, vt_mag) {
        (Some(bt), Some(vt)) => {
            let v_mag = vt - 0.090 * (bt - vt);
            // println!("Debug: Calculated V_Mag = {:.3}", v_mag);
            Ok(v_mag)
        }
        (None, Some(vt)) => {
            // println!("Debug: Using VT_Mag as V_Mag = {:.3}", vt);
            Ok(vt)
        }
        (Some(bt), None) => {
            // println!("Debug: Using BT_Mag as V_Mag = {:.3}", bt);
            Ok(bt)
        }
        (None, None) => Err(CatalogError::MissingMagnitude),
    }
}

fn apply_psf(image: &mut [Vec<f64>], star_x: usize, star_y: usize, intensity: f64, radius: f64, wavelength: f64, aperture_diameter: f64, focal_length: f64) {
    let size = (2.0 * radius).ceil() as usize;
    let half_size = size / 2;

    for dx in 0..size {
        for dy in 0..size {
            let x = star_x as isize + dx as isize - half_size as isize;
            let y = star_y as isize + dy as isize - half_size as isize;

            // Ensure the coordinates are within image bounds
            if x >= 0 && x < image.len() as isize && y >= 0 && y < image[0].len() as isize {
                let r = (((dx as f64 - half_size as f64).powi(2) + (dy as f64 - half_size as f64).powi(2)).sqrt());
                let airy_value = airy_disc_intensity(r, wavelength, aperture_diameter, focal_length);

                // Accumulate intensity at the pixel
                image[x as usize][y as usize] += intensity * airy_value;
            }
        }
    }
}


fn render_stars(
    stars: &[Star],
    output_width: u32,
    output_height: u32,
    center_ra_deg: f64,
    center_dec_deg: f64,
    fov_w_deg: f64,
    fov_h_deg: f64,
) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
    let mut img = ImageBuffer::new(output_width, output_height);

}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    println!("Reading stars from: {:?}", args.file);
    println!("Center - RA (deg): {}", args.center_ra);
    println!("Center - Dec (deg): {}", args.center_dec);
    println!("FOV width (deg): {}", args.fov_w_deg);
    println!("FOV height (deg): {}", args.fov_h_deg);
    println!("Roll (deg): {}", args.roll_deg);
    println!("Max magnitude: {}", args.max_magnitude);

    let start = Instant::now();
    let stars = read_stars(
        args.file,
        args.center_ra,
        args.center_dec,
        args.fov_w_deg,
        args.fov_h_deg,
        args.roll_deg,
        args.max_magnitude,
    )?;
    let read_duration = start.elapsed();




/*
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
*/
    println!("Time taken to render and save image: {:?}", render_duration);
    println!("Image saved as: {}", args.output);
    println!("Total time elapsed: {:?}", start.elapsed());

    Ok(())
}