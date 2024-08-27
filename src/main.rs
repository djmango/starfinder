use clap::Parser;
use csv::ReaderBuilder;
use image::{ImageBuffer, Rgb};
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{self};
use std::path::PathBuf;
use std::time::Instant;
use thiserror::Error;

/// Represents a star with its right ascension, declination, and magnitude.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Star {
    ra_deg: f64,
    de_deg: f64,
    mag: f64,
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

    /// Minimum Right Ascension (degrees)
    #[arg(long, default_value_t = 0.0)]
    min_ra: f64,

    /// Maximum Right Ascension (degrees)
    #[arg(long, default_value_t = 360.0)]
    max_ra: f64,

    /// Minimum Declination (degrees)
    #[arg(long, default_value_t = -90.0)]
    min_dec: f64,

    /// Maximum Declination (degrees)
    #[arg(long, default_value_t = 90.0)]
    max_dec: f64,

    /// Maximum visual magnitude (lower is brighter)
    #[arg(short, long, default_value_t = 6.0)]
    max_magnitude: f64,

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

/// Reads and filters stars from the Tycho-2 catalog file.
pub fn read_stars<P: AsRef<std::path::Path>>(
    path: P,
    min_ra: f64,
    max_ra: f64,
    min_dec: f64,
    max_dec: f64,
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
        ra_deg: ra,
        de_deg: dec,
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

fn render_stars(
    stars: &[Star],
    width: u32,
    height: u32,
    min_ra: f64,
    max_ra: f64,
    min_dec: f64,
    max_dec: f64,
) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
    let mut img = ImageBuffer::new(width, height);

    // Find the minimum and maximum magnitudes in the dataset
    let min_mag = stars.iter().map(|s| s.mag).fold(f64::INFINITY, f64::min);
    let max_mag = stars
        .iter()
        .map(|s| s.mag)
        .fold(f64::NEG_INFINITY, f64::max);

    println!("Magnitude range: {:.3} to {:.3}", min_mag, max_mag);

    for star in stars {
        let x = ((star.ra_deg - min_ra) / (max_ra - min_ra) * width as f64) as u32;
        let y = ((star.de_deg - min_dec) / (max_dec - min_dec) * height as f64) as u32;

        if x < width && y < height {
            // Inverse the magnitude scale (brighter stars have lower magnitudes)
            let normalized_mag = (max_mag - star.mag) / (max_mag - min_mag);

            // Apply a non-linear scaling to emphasize brighter stars
            let brightness = (normalized_mag.powf(2.5) * 255.0) as u8;

            let color = Rgb([brightness, brightness, brightness]);
            img.put_pixel(x, y, color);
        }
    }

    img
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    println!("Reading stars from: {:?}", args.file);
    println!("RA range: {} to {}", args.min_ra, args.max_ra);
    println!("Dec range: {} to {}", args.min_dec, args.max_dec);
    println!("Max magnitude: {}", args.max_magnitude);

    let start = Instant::now();
    let stars = read_stars(
        args.file,
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
