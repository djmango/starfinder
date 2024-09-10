use std::f64::consts::PI;
use image::{ImageBuffer, Rgb};
use nalgebra::SMatrix;
use spec_math::cephes64::j1;
use crate::coords::EquatorialCoords;
use crate::star::Star;

/// Airy disc radius - calculate physical radius, then convert to pixel scale
pub fn calc_airy_disc_radius(lambda: f64, aperture_diameter: f64, focal_length: f64, pixel_size_m: f64) -> f64 {
    // Physical radius on the sensor in meters
    let size = 1.22 * lambda * focal_length / aperture_diameter;
    // Return radius in pixels
    size / pixel_size_m
}

/// Spatial frequency scaled as related to the diffraction pattern. Ooh! Ahh!
pub fn calc_spatial_frequency(lambda: f64, aperture_diameter: f64, focal_length: f64) -> f64 {
    2.0 * PI / lambda * aperture_diameter / focal_length
}

/// Calculate the airy disc intensity at a given radius. Spatial frequency should be pre-calcualted
/// to avoid extra operations per loop. It is given by:
/// 2.0 * PI / lambda * aperture_diameter / focal_length
/// where lambda is wavelength of light. All arguments are in meters. There is a helper method
/// defined to do this for you (see calc_spatial_frequency)
///
/// Airy Disc formula:
/// I(r) = I_0 * (2 * J1(kr) / kr)^2
/// Where:
/// I(r)  = Intensity at distance r from the center of the airy disc
/// I_0   = Max intensity at the center of the disc (the star itself/point light source, i.e. r = 0)
/// J1(x) = First order Bessel function of the first kind (the magic sauce)
/// k     = 2 * pi / wavelength * (aperture_diameter / focal_length) -- incorporates camera params
/// r     = Radial distance from the center of the disc

pub fn calc_airy_intensity_at_radius(r: f64, spatial_frequency: f64) -> f64 {
    // Bessel argument, which will give us our intensity at radius r
    let kr = spatial_frequency * r;

    // At the center, we're at full intensity
    if (kr == 0.0) {
        return 1.0;
    }

    let bessel = j1(kr);
    let intensity = (2.0 * bessel / kr).powi(2);
    intensity
}

/// Appy PSF to an image at a given location
fn apply_psf(image: &mut [Vec<f64>], star_x: usize, star_y: usize, intensity: f64, radius: f64, wavelength: f64, aperture_diameter: f64, focal_length: f64) {
    let size = (2.0 * radius).ceil() as usize;
    let half_size = size / 2;

    for dx in 0..size {
        for dy in 0..size {
            let x = star_x as isize + dx as isize - half_size as isize;
            let y = star_y as isize + dy as isize - half_size as isize;

            // Ensure the coordinates are within image bounds
            if x >= 0 && x < image.len() as isize && y >= 0 && y < image[0].len() as isize {
                let r = ((dx as f64 - half_size as f64).powi(2) + (dy as f64 - half_size as f64).powi(2)).sqrt();
                // let airy_value = airy_disc_intensity(r, wavelength, aperture_diameter, focal_length);

                // Accumulate intensity at the pixel
                // image[x as usize][y as usize] += intensity * airy_value;
            }
        }
    }
}

pub fn render_stars(
    stars: &Vec<Star>,
    width: u32,
    height: u32,
    center: &EquatorialCoords,
    fov_w: f64,
    fov_h: f64,
    roll: f64,
) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
    let mut img = ImageBuffer::new(width, height);

    // Find the minimum and maximum magnitudes in the dataset
    let min_mag = stars.iter().map(|s| s.mag).fold(f64::INFINITY, f64::min);
    let max_mag = stars
        .iter()
        .map(|s| s.mag)
        .fold(f64::NEG_INFINITY, f64::max);
    let z_roll_mat = SMatrix::<f64, 2, 2>::new(
        roll.cos(), -roll.sin(),
        roll.sin(), roll.cos(),
    );
    let pixel_ratio_w = width as f64 / fov_w;
    let pixel_ratio_h = height as f64 / fov_h;

    for star in stars {
        let std_star_coords = star.coords.to_standard(center);
        let std_star_mat = SMatrix::<f64, 2, 1>::new(std_star_coords.x, std_star_coords.y);
        let final_star_pos = z_roll_mat * std_star_mat;

        let x = final_star_pos.x * pixel_ratio_w;
        let y = final_star_pos.y * pixel_ratio_h;
        if (x < 0.0 || x > width as f64 || y < 0.0 || y > height as f64) {
            break;
        }

        // Inverse the magnitude scale (brighter stars have lower magnitudes)
        let normalized_mag = (max_mag - star.mag) / (max_mag - min_mag);
        // Apply a non-linear scaling to emphasize brighter stars
        let brightness = (normalized_mag.powf(2.5) * 255.0) as u8;
        let color = Rgb([brightness, brightness, brightness]);

        //img.put_pixel((x + ((width as f64) / 2.0)) as u32, (y + (height as f64) / 2.0) as u32, color);
        img.put_pixel(x as u32, y as u32, color);
    }
    /*let std_star_coords = stars[0].coords.to_standard(center);
    let std_star_mat = SMatrix::<f64, 2, 1>::new(std_star_coords.x, std_star_coords.y);
    let final_star_pos = z_roll_mat * std_star_mat;

    let x = final_star_pos.x * pixel_ratio_w;
    let y = final_star_pos.y * pixel_ratio_h;

    // Inverse the magnitude scale (brighter stars have lower magnitudes)
    let normalized_mag = (max_mag - stars[0].mag) / (max_mag - min_mag);
    // Apply a non-linear scaling to emphasize brighter stars
    let brightness = 255;
    let color = Rgb([brightness, brightness, brightness]);
println!("x, y: {}, {}", x, y);
    img.put_pixel((x + ((width as f64) / 2.0)) as u32, (y + (height as f64) / 2.0) as u32, color);*/

    img
}