
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
        let ra: f64;

        if self.y < 0.0 {
            ra = (2.0 * PI) - self.x.clamp(-1.0, 1.0).acos();
        } else {
            ra = self.x.clamp(-1.0, 1.0).acos();
        }

        EquatorialCoords {
            ra,
            dec: self.z.clamp(-1.0, 1.0).asin(),
        }
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

        StandardCoords {
            x: (dec.cos() * (ra - cra).sin())
                / ((cdec.cos() * dec.cos() * (ra - cra).cos()) + (dec.sin() * cdec.sin())),
            y: ((cdec.sin() * dec.cos() * (ra - cra).cos()) - (cdec.cos() * dec.sin()))
                / ((cdec.cos() * dec.cos() * (ra - cra).cos()) + (dec.sin() * cdec.sin())),
        }
    }

    pub fn to_cartesian(&self) -> CartesianCoords {
        CartesianCoords {
            x: self.ra.cos(),
            y: self.ra.sin(),
            z: self.dec.sin(),
        }
    }

    pub fn to_grid(&self, fov_size: f64) -> EquatorialCoords {
        EquatorialCoords {
            ra: (self.ra / (2.0 * PI) * (1.0 - (2.0 * self.dec.abs() / PI)).powf(0.5) / fov_size).round(),
            dec: (self.dec / (2.0 * PI) / fov_size).round(),
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

