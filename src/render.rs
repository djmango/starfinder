use crate::types::Star;
use image::{ImageBuffer, Rgb};

pub fn render_stars(
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
