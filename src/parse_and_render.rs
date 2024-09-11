use std::time::Instant;

use crate::fov;
use crate::parsing_utils::read_stars;
use crate::rendering::render_stars;
use crate::types::{EquatorialCoords, StarCatalogArgs};

pub fn parse_and_render(args: &StarCatalogArgs) -> Result<(), Box<dyn std::error::Error>> {
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
    println!("Total FOV retrieval time: {:?}", get_fov_start.elapsed());

    // 2) Read stars and filter against rolled_fov to create subset of stars in view of the image
    let read_stars_start = Instant::now();
    let stars_in_fov = read_stars(&args.source, rolled_fov, args.max_magnitude)?;
    println!(
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
    img.save(&args.output)?;
    println!(
        "Total parse and write stars: {:?}",
        render_stars_start.elapsed()
    );

    println!("Total run time elapsed: {:?}", run_start.elapsed());

    Ok(())
}
