use csv::ReaderBuilder;
use std::collections::HashSet;
use std::fs::File;
use std::io;
use thiserror::Error;
use tracing::{error, info, warn};

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

/// Reads and filters stars from the optimized data file.
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
        // Use the indexes for the optimized data file!
        match parse_optimized_star_record(&record) {
            Ok(data) => {
                let star = data.0;
                let grid_coords = data.1;

                if star.mag < max_magnitude && filter_grid.contains(&grid_coords) {
                    stars.push(star);
                }
            }
            Err(e) => {
                skipped_rows += 1;
                if skipped_rows <= 10 {
                    error!("Skipping row {} due to error: {:?}", i, e);
                    warn!("Problematic row: {:?}", record);
                } else if skipped_rows == 11 {
                    warn!("Further skipped rows will not be printed...");
                }
            }
        }
    }

    info!("Total stars read and filtered: {}", stars.len());
    info!("Total rows skipped: {}", skipped_rows);

    Ok(stars)
}

/// Parses the *optimize* star data file and pulls out the star as well as its pre-computed ra/dec
/// in the localization grid format as a separate EquatorialCoordinate
pub fn parse_optimized_star_record(
    record: &csv::StringRecord,
) -> Result<(Star, EquatorialCoords), CatalogError> {
    Ok((
        Star {
            coords: EquatorialCoords {
                ra: parse_field(record, 0, "RA")?,
                dec: parse_field(record, 1, "Dec")?,
            },
            mag: parse_field(record, 4, "Magnitude")?,
        },
        EquatorialCoords {
            ra: parse_field(record, 2, "GridRA")?,
            dec: parse_field(record, 3, "GridDec")?,
        },
    ))
}

/// Parses a single record from the catalog into a Star struct.
/// Defaults for the index positions of data fields are set to Tycho 2 catalog format. This can be
/// specified differently if a different star catalog is to be used
pub fn parse_star_record(
    record: &csv::StringRecord,
    idx_ra: Option<u64>,
    idx_dec: Option<u64>,
    idx_bt_mag: Option<u64>,
    idx_vt_mag: Option<u64>,
) -> Result<Star, CatalogError> {
    let ra = parse_field(record, idx_ra.unwrap_or(24), "RA")?;
    let dec = parse_field(record, idx_dec.unwrap_or(25), "Dec")?;
    let mag = parse_magnitude(record, idx_bt_mag, idx_vt_mag)?;

    Ok(Star {
        coords: EquatorialCoords {
            ra: ra.to_radians(),
            dec: dec.to_radians(),
        },
        mag,
    })
}

/// Get magnitude off a star record
/// Defaults for the index positions of data fields are set to Tycho 2 catalog format. This can be
/// specified differently if a different star catalog is to be used
pub fn parse_magnitude(
    record: &csv::StringRecord,
    idx_bt_mag: Option<u64>,
    idx_vt_mag: Option<u64>,
) -> Result<f64, CatalogError> {
    let bt_mag = parse_field(record, idx_bt_mag.unwrap_or(17), "BT magnitude").ok();
    let vt_mag = parse_field(record, idx_vt_mag.unwrap_or(19), "VT magnitude").ok();

    match (bt_mag, vt_mag) {
        (Some(bt), Some(vt)) => {
            let v_mag = vt - 0.090 * (bt - vt);
            Ok(v_mag)
        }
        (None, Some(vt)) => Ok(vt),
        (Some(bt), None) => Ok(bt),
        (None, None) => Err(CatalogError::MissingMagnitude),
    }
}

/// Parses a field from the record, returning a helpful error if parsing fails.
pub fn parse_field(
    record: &csv::StringRecord,
    index: u64,
    field_name: &str,
) -> Result<f64, CatalogError> {
    record
        .get(index.try_into().unwrap())
        .ok_or_else(|| CatalogError::MissingField(field_name.to_string()))?
        .trim()
        .parse()
        .map_err(|_| CatalogError::Parse(format!("Failed to parse {}", field_name)))
}
