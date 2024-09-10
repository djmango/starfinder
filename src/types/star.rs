use serde::{Deserialize, Serialize};

use crate::types::EquatorialCoords;

/// Represents a star with its right ascension, declination, and magnitude.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Star {
    pub coords: EquatorialCoords,
    pub mag: f64,
}
