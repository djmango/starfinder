use clap::Parser;
use csv::ReaderBuilder;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{self};
use std::path::PathBuf;
use std::time::Instant;
use thiserror::Error;

/// Represents a star with its right ascension, declination, and magnitude.
#[derive(Debug, Deserialize, Serialize)]
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
}

/// Reads stars from the Tycho-2 catalog file.
///
/// # Arguments
///
/// * `path` - A path to the catalog file.
///
/// # Returns
///
/// A vector of `Star` structs representing the successfully read stars.
///
/// # Errors
///
/// Returns a `CatalogError` if there's an issue reading the file or parsing its contents.
pub fn read_stars<P: AsRef<std::path::Path>>(path: P) -> Result<Vec<Star>, CatalogError> {
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
                if i % 10000 == 0 {
                    println!(
                        "Star {}: RA={}, Dec={}, Mag={}",
                        i, star.ra_deg, star.de_deg, star.mag
                    );
                }
                stars.push(star);
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

    println!("Total stars read: {}", stars.len());
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

/// Parses the magnitude from a record, trying VT first, then BT.
fn parse_magnitude(record: &csv::StringRecord) -> Result<f64, CatalogError> {
    if let Ok(vt_mag) = parse_field(record, 20, "VT magnitude") {
        Ok(vt_mag)
    } else if let Ok(bt_mag) = parse_field(record, 17, "BT magnitude") {
        Ok(bt_mag)
    } else {
        Err(CatalogError::MissingMagnitude)
    }
}

fn main() -> Result<(), CatalogError> {
    let args = Args::parse();

    println!("Reading stars from: {:?}", args.file);

    let start = Instant::now();
    let stars = read_stars(args.file)?;
    let read_duration = start.elapsed();

    println!("Time taken to read stars: {:?}", read_duration);
    println!("Total stars read: {}", stars.len());

    let display_start = Instant::now();
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
    let display_duration = display_start.elapsed();

    println!("Time taken to display stars: {:?}", display_duration);
    println!("Total time elapsed: {:?}", start.elapsed());

    Ok(())
}
