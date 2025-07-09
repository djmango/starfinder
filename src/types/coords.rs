
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::hash::{Hash, Hasher};

#[derive(Debug, Deserialize, Serialize, Clone, Copy)]
pub struct StandardCoords {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Deserialize, Serialize, Clone, Copy)]
pub struct CartesianCoords {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl CartesianCoords {
    pub fn to_equatorial(&self) -> EquatorialCoords {
        // Correct spherical coordinate conversion from Cartesian
        let ra = self.y.atan2(self.x); // atan2 handles all quadrants correctly
        let dec = self.z.asin(); // z = sin(dec) for unit sphere
        
        // Ensure RA is in [0, 2π) range
        let ra = if ra < 0.0 { ra + 2.0 * PI } else { ra };

        EquatorialCoords { ra, dec }
    }
}


#[derive(Debug, Deserialize, Serialize, Clone, Copy)]
pub struct EquatorialCoords {
    pub ra: f64,
    pub dec: f64,
}

impl EquatorialCoords {
    /// Calculate a point's standard coordinates on the plane tangent to the celestial sphere, whose
    /// center point sits tangent to the sphere where the camera's central (z) axis meets it
    pub fn to_standard(&self, center: EquatorialCoords) -> StandardCoords {
        // Right ascension and declination of current object, in radians
        let ra = self.ra;
        let dec = self.dec;
        // Right ascension and declination of center point of tangent plane, in radians
        let cra = center.ra;
        let cdec = center.dec;

        // Handle RA wrap-around (crossing 0°/360°)
        let mut ra_diff = ra - cra;
        if ra_diff > PI {
            ra_diff -= 2.0 * PI;
        } else if ra_diff < -PI {
            ra_diff += 2.0 * PI;
        }

        // Gnomonic projection - projecting sphere onto tangent plane
        let denominator = (cdec.cos() * dec.cos() * ra_diff.cos()) + (dec.sin() * cdec.sin());
        
        // Prevent division by zero or very small denominators (stars behind the projection plane)
        if denominator.abs() < 1e-10 {
            // Return coordinates far outside any reasonable FOV
            return StandardCoords { x: 1000.0, y: 1000.0 };
        }
        
        StandardCoords {
            x: (dec.cos() * ra_diff.sin()) / denominator,
            y: ((cdec.sin() * dec.cos() * ra_diff.cos()) - (cdec.cos() * dec.sin())) / denominator,
        }
    }

    pub fn to_cartesian(&self) -> CartesianCoords {
        // FIXED: Correct spherical to Cartesian conversion
        // For a point on unit sphere: (ra, dec) -> (x, y, z)
        CartesianCoords {
            x: self.dec.cos() * self.ra.cos(),
            y: self.dec.cos() * self.ra.sin(),
            z: self.dec.sin(),
        }
    }

    pub fn to_grid(&self, fov_size: f64) -> EquatorialCoords {
        let clamped_size = fov_size.clamp(0.02, 1.0);
        EquatorialCoords {
            ra: (self.ra / (2.0 * PI) * (1.0 - (2.0 * self.dec.abs() / PI)).powf(0.5) / clamped_size).round(),
            dec: (self.dec / (2.0 * PI) / clamped_size).round(),
        }
    }
}

impl PartialEq for EquatorialCoords {
    fn eq(&self, other: &Self) -> bool {
        self.ra == other.ra && self.dec == other.dec
    }
}

impl Eq for EquatorialCoords {
}

impl Hash for EquatorialCoords {
    fn hash<H: Hasher>(&self, state: &mut H) {
        (self.ra as i32).hash(state);
        (self.dec as i32).hash(state);
    }
}
