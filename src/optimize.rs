use anyhow::Result;
use csv::ReaderBuilder;
use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::path::PathBuf;
use std::time::Instant;
use tracing::{error, info, warn};

use crate::parsing_utils::parse_star_record;

const CSV_SEPARATOR: char = '|';

/// Optimizes the given source data and writes the result to the output.
///
/// # Arguments
///
/// * `source` - A String that holds the path to the source file.
/// * `output` - A String that holds the path to the output file.
/// * `separator` - A String that specifies the delimiter used in the source file.
/// * `idx_vt_mag` - The index of the VT magnitude column in the source file.
/// * `idx_bt_mag` - The index of the BT magnitude column in the source file.
/// * `idx_ra` - The index of the Right Ascension column in the source file.
/// * `idx_dec` - The index of the Declination column in the source file.
/// * `fov_max` - The maximum field of view as a f64.
///
/// # Returns
///
/// Returns a `Result<()>` which is `Ok(())` if the operation was successful,
/// or an `Err` containing the error information if something went wrong.
pub fn optimize(
    source: &PathBuf,
    output: &PathBuf,
    separator: &str,
    idx_vt_mag: Option<u64>,
    idx_bt_mag: Option<u64>,
    idx_ra: u64,
    idx_dec: u64,
    fov_max: f64,
) -> Result<()> {
    // First check if the tycho2 catalog already exists in the cache
    if output.exists() {
        info!("Tycho2 optimized catalog already exists at: {:?}", &output);
        return Ok(());
    }
    let run_start = Instant::now();

    if idx_bt_mag.is_none() && idx_vt_mag.is_none() {
        panic!("Must specify at least one of idx-bt-mag or idx-vt-mag (ideally both).");
    }
    if separator.len() > 1 {
        panic!("Separator must be a single character.");
    }

    info!("Reading catalog data and optimizing to starfinder format. This may take a while...");
    let src_file = File::open(source)?;
    let src_reader = BufReader::new(src_file);
    let mut csv_reader = ReaderBuilder::new()
        .delimiter(separator.to_string().into_bytes()[0])
        .has_headers(false)
        .from_reader(src_reader);

    let out_file = File::create(output)?;
    let mut out_buffer = BufWriter::new(out_file);
    let fov_scale = fov_max.to_radians() / (4.0 * PI);

    for (i, result) in csv_reader.records().enumerate() {
        let record = result?;

        let mut skipped_rows = 0;

        match parse_star_record(&record, Some(idx_ra), Some(idx_dec), idx_bt_mag, idx_vt_mag) {
            Ok(star) => {
                let grid_coords = star.coords.to_grid(fov_scale);
                let ra = star.coords.ra;
                let dec = star.coords.dec;
                let grid_ra = grid_coords.ra;
                let grid_dec = grid_coords.dec;
                let mag = star.mag;
                let s = CSV_SEPARATOR;

                out_buffer.write_all(
                    format!("{ra}{s}{dec}{s}{grid_ra}{s}{grid_dec}{s}{mag}\n").as_bytes(),
                )?;
            }
            Err(e) => {
                skipped_rows += 1;
                if skipped_rows <= 10 {
                    error!("Skipping row {} due to error: {:?}", i, e);
                    error!("Problematic row: {:?}", record);
                } else if skipped_rows == 11 {
                    warn!("Further skipped rows will not be printed...");
                }
            }
        }
    }
    out_buffer.flush()?;

    info!("Total run time elapsed: {:?}", run_start.elapsed());
    Ok(())
}
