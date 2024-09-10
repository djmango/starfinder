use csv::ReaderBuilder;
use std::collections::HashSet;
use std::fs::File;
use std::io;
use thiserror::Error;

use crate::types::{EquatorialCoords, Star};

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

/// Reads and filters stars from the Tycho-2 catalog file.
/// https://heasarc.gsfc.nasa.gov/w3browse/all/tycho2.html
/// http://tdc-www.harvard.edu/catalogs/tycho2.format.html
/// All values should be in radians. While star coords in the catalog are in degrees, the coordinate
/// calculations make use of trig functions which deal in radians - this will be our default mode
pub fn read_stars<P: AsRef<std::path::Path>>(
    path: P,
    filter_grid: HashSet<EquatorialCoords>,
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
                let grid_coords = &star.coords.to_grid();
                if star.mag < max_magnitude && filter_grid.contains(&grid_coords) {
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
pub fn parse_star_record(record: &csv::StringRecord) -> Result<Star, CatalogError> {
    let ra = parse_field(record, 24, "RA")?;
    let dec = parse_field(record, 25, "Dec")?;
    let mag = parse_magnitude(record)?;

    Ok(Star {
        coords: EquatorialCoords {
            ra: ra.to_radians(),
            dec: dec.to_radians(),
        },
        mag,
    })
}

/// Get magnitude off a star record
pub fn parse_magnitude(record: &csv::StringRecord) -> Result<f64, CatalogError> {
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

/// Parses a field from the record, returning a helpful error if parsing fails.
pub fn parse_field(
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
