use anyhow::Result;
use directories::BaseDirs;
use image::ImageBuffer;
use image::Rgb;
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

/// Download the Tycho2 catalog from the ESO archive
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

/// Read stars from the optimized Tycho2 catalog, filter against the FOV, and render the starfield
pub fn read_and_render(
    center_ra: f64,
    center_dec: f64,
    roll: f64,
    fov_w: f64,
    fov_h: f64,
    max_magnitude: f64,
    width: u32,
    height: u32,
    fov_max: f64,
) -> Result<ImageBuffer<Rgb<u8>, Vec<u8>>> {
    let run_start = Instant::now();

    let tycho2_path = BaseDirs::new().unwrap().cache_dir().join(TYCHO2_CATALOG);
    let fov_path = TYCHO2_OPTIMIZED.replace("FOV", fov_max.to_string().as_str());
    let tycho2_path_optimized = BaseDirs::new().unwrap().cache_dir().join(fov_path);

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
        fov_max,
    )?;

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
    let stars_in_fov = read_stars(&tycho2_path_optimized, rolled_fov, max_magnitude)?;
    info!(
        "Total time to read and parse stars: {:?}",
        read_stars_start.elapsed()
    );

    // 3) Render stars in FOV
    let render_stars_start = Instant::now();
    let img = render_stars(stars_in_fov, width, height, center, fov_w, fov_h, roll);

    info!(
        "Total parse and write stars: {:?}",
        render_stars_start.elapsed()
    );
    info!("Total run time elapsed: {:?}", run_start.elapsed());
    Ok(img)
}

/// Run the full pipeline to render a starfield to a file
pub fn render_to_file(args: &StarCatalogArgs) -> Result<()> {
    let img = read_and_render(
        args.center_ra,
        args.center_dec,
        args.roll,
        args.fov_w,
        args.fov_h,
        args.max_magnitude,
        args.width,
        args.height,
        args.fov_max,
    )?;

    // Create all parent directories
    if let Some(parent) = PathBuf::from(&args.output).parent() {
        fs::create_dir_all(parent)?;
    }
    img.save(&args.output)?;

    info!("Starfield render complete! File output at: {}", args.output);
    Ok(())
}
