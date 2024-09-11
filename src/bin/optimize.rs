use clap::Parser;
use csv::ReaderBuilder;
use starfinder::parsing_utils::parse_star_record;
use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::time::Instant;
use std::f64::consts::PI;

const CSV_SEPARATOR: char = '|';

/// CLI Arguments
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
pub struct Args {
    /// Default to attempting to ingest and optimize the tycho2 data set per library standard
    #[arg(short, long, default_value = "data/tycho2/catalog.dat")]
    source: String,

    /// Output to standard pre-parsed file location - deviating from this means changing source when
    /// calling the renderer
    #[arg(short, long, default_value = "data/optimized.dat")]
    output: String,

    #[arg(long, default_value = "|")]
    separator: String,

    #[arg(long)]
    idx_vt_mag: Option<usize>,

    #[arg(long)]
    idx_bt_mag: Option<usize>,

    #[arg(long)]
    idx_ra: usize,

    #[arg(long)]
    idx_dec: usize,

    #[arg(long)]
    fov_max: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let run_start = Instant::now();
    let args = Args::parse();

    if args.idx_bt_mag.is_none() && args.idx_vt_mag.is_none() {
        panic!("Must specify at least one of idx-bt-mag or idx-vt-mag (ideally both).");
    }
    if args.separator.len() > 1 {
        panic!("Separator must be a single character.");
    }

    println!("Reading catalog data and optimizing to starfinder format. This may take a while...");
    let src_file = File::open(args.source)?;
    let src_reader = BufReader::new(src_file);
    let mut csv_reader = ReaderBuilder::new()
        .delimiter(args.separator.to_string().into_bytes()[0])
        .has_headers(false)
        .from_reader(src_reader);

    let out_file = File::create(args.output)?;
    let mut out_buffer = BufWriter::new(out_file);
    let fov_scale = args.fov_max.to_radians()/(4.0*PI);

    for (i, result) in csv_reader.records().enumerate() {
        let record = result?;

        let mut skipped_rows = 0;

        match parse_star_record(
            &record,
            Some(args.idx_ra),
            Some(args.idx_dec),
            args.idx_bt_mag,
            args.idx_vt_mag,
        ) {
            Ok(star) => {
                let grid_coords = star.coords.to_grid(fov_scale);
                let ra = star.coords.ra;
                let dec = star.coords.dec;
                let grid_ra = grid_coords.ra;
                let grid_dec = grid_coords.dec;
                let mag = star.mag;
                let s = CSV_SEPARATOR;

                out_buffer.write_all(
                    format!("{ra}{s}{dec}{s}{grid_ra}{s}{grid_dec}{s}{mag}\n").as_bytes()
                )?;
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
    out_buffer.flush()?;

    println!("Total run time elapsed: {:?}", run_start.elapsed());
    Ok(())
}
