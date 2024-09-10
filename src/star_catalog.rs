use csv::ReaderBuilder;
use std::fs::File;
use std::io::{self};

use crate::types::{CatalogError, Star};

/// Reads and filters stars from the Tycho-2 catalog file.
/// https://heasarc.gsfc.nasa.gov/w3browse/all/tycho2.html
/// http://tdc-www.harvard.edu/catalogs/tycho2.format.html
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
