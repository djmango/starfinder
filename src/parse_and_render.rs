use anyhow::Result;
use directories::BaseDirs;
use reqwest;
use std::fs;
use std::path::PathBuf;
use std::time::Duration;
use std::time::Instant;
use tracing::info;

use crate::constants::file_paths::{TYCHO2_CATALOG, TYCHO2_OPTIMIZED};
use crate::fov;
use crate::optimize::optimize;
use crate::parsing_utils::read_stars;
use crate::rendering::render_stars;
use crate::types::{EquatorialCoords, StarCatalogArgs};

pub fn download_tycho2(tycho2_path: &PathBuf) -> Result<()> {
    let tycho2_url = "https://archive.eso.org/ASTROM/TYC-2/data/catalog.dat";
    // Create all parent directories
    if let Some(parent) = tycho2_path.parent() {
        fs::create_dir_all(parent)?;
    }

    // First check if the tycho2 catalog already exists in the cache
    if tycho2_path.exists() {
        info!("Tycho2 catalog already exists at: {:?}", &tycho2_path);
        return Ok(());
    }

    info!("Downloading Tycho2 catalog from: {}", tycho2_url);
    let client = reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(120)) // 2 minutes
        .build()?;

    let response = client.get(tycho2_url).send()?;
    // let response = reqwest::blocking::get(tycho2_url)?;
    let mut file = std::fs::File::create(&tycho2_path)?;
    std::io::copy(&mut response.bytes().unwrap().as_ref(), &mut file)?;
    info!("Tycho2 catalog downloaded to: {:?}", &tycho2_path);

    Ok(())
}

pub fn parse_and_render(args: &StarCatalogArgs) -> Result<()> {
    let tycho2_path = BaseDirs::new().unwrap().cache_dir().join(TYCHO2_CATALOG);

    let tycho2_path_optimized = BaseDirs::new().unwrap().cache_dir().join(TYCHO2_OPTIMIZED);

    // TODO: a hash function to check if the file that is in the cache is the same as the one that we expect

    download_tycho2(&tycho2_path)?;
    optimize(
        &tycho2_path,
        &tycho2_path_optimized,
        "|",
        Some(19),
        Some(17),
        24,
        25,
        args.fov_max,
    )?;

    let run_start = Instant::now();
    let center_ra = args.center_ra.to_radians();
    let center_dec = args.center_dec.to_radians();
    let roll = args.roll.to_radians();
    let fov_w = args.fov_w.to_radians();
    let fov_h = args.fov_h.to_radians();

    // 1) Rotate FOV by specified roll
    let get_fov_start = Instant::now();
    let center = EquatorialCoords {
        ra: center_ra,
        dec: center_dec,
    };
    let rolled_fov = fov::get_fov(center, fov_w, fov_h, roll);
    info!("Total FOV retrieval time: {:?}", get_fov_start.elapsed());

    // 2) Read stars and filter against rolled_fov to create subset of stars in view of the image
    let read_stars_start = Instant::now();
    let stars_in_fov = read_stars(&tycho2_path_optimized, rolled_fov, args.max_magnitude)?;
    info!(
        "Total time to read and parse stars: {:?}",
        read_stars_start.elapsed()
    );

    // 3) Render stars in FOV
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

    // Create all parent directories
    if let Some(parent) = PathBuf::from(&args.output).parent() {
        fs::create_dir_all(parent)?;
    }
    img.save(&args.output)?;

    info!(
        "Total parse and write stars: {:?}",
        render_stars_start.elapsed()
    );

    info!("Total run time elapsed: {:?}", run_start.elapsed());
    info!("Starfield render complete! File output at: {}", args.output);

    Ok(())
}
