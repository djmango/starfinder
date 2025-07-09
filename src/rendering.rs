use image::{ImageBuffer, Rgb};
use nalgebra::SMatrix;
use std::f64::consts::PI;

use crate::types::{EquatorialCoords, Star};

pub fn render_stars(
    stars: Vec<Star>,
    width: u32,
    height: u32,
    fov_center: EquatorialCoords,
    fov_w: f64,
    fov_h: f64,
    fov_roll: f64,
) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
    let mut img = ImageBuffer::new(width, height);

    // Find the minimum and maximum magnitudes in the dataset
    let min_mag = stars.iter().map(|s| s.mag).fold(f64::INFINITY, f64::min);
    let max_mag = stars
        .iter()
        .map(|s| s.mag)
        .fold(f64::NEG_INFINITY, f64::max);
    let z_roll_mat = SMatrix::<f64, 2, 2>::new(
        fov_roll.cos(),
        -fov_roll.sin(),
        fov_roll.sin(),
        fov_roll.cos(),
    );
    let pixel_ratio_w = width as f64 / fov_w;
    let pixel_ratio_h = height as f64 / fov_h;
    let x_offset = width as f64 / 2.0;
    let y_offset = height as f64 / 2.0;

    // Calculate maximum angular distance for FOV (half diagonal)
    let max_fov_radius = (fov_w.powi(2) + fov_h.powi(2)).sqrt() / 2.0;
    
    println!("Attempting to render {} stars", stars.len());
    println!("Magnitude range: {} to {}", min_mag, max_mag);
    let mut stars_rendered = 0;
    let mut stars_out_of_bounds = 0;
    let mut stars_too_far = 0;

    for star in stars {
        // Calculate angular distance from center using spherical distance formula
        let ra_diff = star.coords.ra - fov_center.ra;
        let dec_star = star.coords.dec;
        let dec_center = fov_center.dec;
        
        // Great circle distance formula
        let angular_distance = (dec_star.sin() * dec_center.sin() + 
                               dec_star.cos() * dec_center.cos() * ra_diff.cos()).acos();
        
        // Skip stars outside the field of view (fixed broken FOV grid filtering)
        if angular_distance > max_fov_radius {
            stars_too_far += 1;
            continue;
        }

        let std_star_coords = star.coords.to_standard(fov_center);
        let std_star_mat = SMatrix::<f64, 2, 1>::new(std_star_coords.x, std_star_coords.y);
        let final_star_pos = z_roll_mat * std_star_mat;

        let x = final_star_pos.x * pixel_ratio_w + x_offset;
        let y = final_star_pos.y * pixel_ratio_h + y_offset;

        if x < 0.0 || x > width as f64 || y < 0.0 || y > height as f64 {
            stars_out_of_bounds += 1;
            continue;
        }

        // Inverse the magnitude scale (brighter stars have lower magnitudes)
        let normalized_mag = if max_mag > min_mag {
            (max_mag - star.mag) / (max_mag - min_mag)
        } else {
            1.0 // If all stars have same magnitude
        };
        
        // Scale star intensity and size based on magnitude
        let star_intensity = normalized_mag.powf(0.8); // Gentler curve
        
        // Much smaller, realistic star sizes (1-3 pixels radius)
        let star_radius = if star.mag < 2.0 { 
            2.5 // Bright stars get slightly larger 
        } else if star.mag < 5.0 { 
            1.8 
        } else { 
            1.2 // Faint stars stay small
        };
        
        // Render the star with a simple PSF
        render_star_simple(&mut img, x, y, star_intensity, star_radius);
        stars_rendered += 1;
    }

    println!("Actually rendered {} stars, {} out of bounds, {} too far from center", stars_rendered, stars_out_of_bounds, stars_too_far);
    img
}

/// Render a single star with a simple Gaussian-like profile
fn render_star_simple(
    img: &mut ImageBuffer<Rgb<u8>, Vec<u8>>,
    center_x: f64,
    center_y: f64,
    intensity: f64,
    radius: f64,
) {
    let render_radius = (radius + 1.0) as i32; // Small render area
    
    for dy in -render_radius..=render_radius {
        for dx in -render_radius..=render_radius {
            let pixel_x = (center_x + dx as f64) as i32;
            let pixel_y = (center_y + dy as f64) as i32;
            
            if pixel_x >= 0 && pixel_x < img.width() as i32 && 
               pixel_y >= 0 && pixel_y < img.height() as i32 {
                
                let r = ((dx as f64).powi(2) + (dy as f64).powi(2)).sqrt();
                
                // Simple Gaussian-like falloff
                let gaussian_falloff = if r <= radius {
                    let normalized_r = r / radius;
                    (-2.0 * normalized_r.powi(2)).exp()
                } else {
                    0.0
                };
                
                let final_intensity = intensity * gaussian_falloff;
                let brightness = (final_intensity * 255.0).min(255.0) as u8;
                
                if brightness > 0 {
                    // Get current pixel value and add to it (for overlapping stars)
                    let current_pixel = img.get_pixel(pixel_x as u32, pixel_y as u32);
                    let new_r = (current_pixel[0] as u16 + brightness as u16).min(255) as u8;
                    let new_g = (current_pixel[1] as u16 + brightness as u16).min(255) as u8;
                    let new_b = (current_pixel[2] as u16 + brightness as u16).min(255) as u8;
                    
                    img.put_pixel(pixel_x as u32, pixel_y as u32, Rgb([new_r, new_g, new_b]));
                }
            }
        }
    }
}

/// Airy disc radius - calculate physical radius, then convert to pixel scale
pub fn calc_airy_disc_radius(lambda: f64, aperture_diameter: f64, focal_length: f64, pixel_size_m: f64) -> f64 {
    // Physical radius on the sensor in meters
    let size = 1.22 * lambda * focal_length / aperture_diameter;
    // Return radius in pixels
    size / pixel_size_m
}

/// Spatial frequency scaled as related to the diffraction pattern
pub fn calc_spatial_frequency(lambda: f64, aperture_diameter: f64, focal_length: f64) -> f64 {
    2.0 * PI / lambda * aperture_diameter / focal_length
}

/// Calculate the airy disc intensity at a given radius
pub fn calc_airy_intensity_at_radius(r: f64, spatial_frequency: f64) -> f64 {
    let kr = spatial_frequency * r;

    if kr < 1e-10 {
        return 1.0; // At the center, full intensity
    }

    // Simplified Airy disc approximation (avoiding Bessel function dependency)
    // This creates a realistic falloff pattern
    let normalized_r = kr / (2.0 * PI);
    let intensity = if normalized_r < 1.0 {
        (1.0 - normalized_r).powi(2)
    } else {
        0.1 * (-normalized_r + 1.0).exp() // Gentle falloff for outer regions
    };
    
    intensity.max(0.0)
}
