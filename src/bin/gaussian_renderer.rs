use std::fs::File;
use std::io;
use std::path::PathBuf;

/// Represents a star with its right ascension, declination, and magnitude.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Star {
    ra_deg: f64,
    de_deg: f64,
    mag: f64,
}

/// Errors that can occur during star catalog reading.
#[derive(Error, Debug)]
pub enum CatalogError {
    #[error("IO error: {0}")]
    Io(#[from] io::Error),
    #[error("CSV parsing error: {0}")]
    Csv(#[from] csv::Error),
    #[error("Missing field: {0}")]
    MissingField(String),
    #[error("Parse error: {0}")]
    Parse(String),
    #[error("Missing magnitude")]
    MissingMagnitude,
}

/// CLI Arguments
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the Tycho-2 catalog file
    #[arg(
        short,
        long,
        value_name = "FILE",
        default_value = "data/tycho2/catalog.dat"
    )]
    file: PathBuf,

    /// Number of stars to display (0 for all)
    #[arg(short, long, default_value_t = 10)]
    display_count: usize,

    /// Minimum Right Ascension (degrees)
    #[arg(long, default_value_t = 0.0)]
    min_ra: f64,

    /// Maximum Right Ascension (degrees)
    #[arg(long, default_value_t = 360.0)]
    max_ra: f64,

    /// Minimum Declination (degrees)
    #[arg(long, default_value_t = -90.0)]
    min_dec: f64,

    /// Maximum Declination (degrees)
    #[arg(long, default_value_t = 90.0)]
    max_dec: f64,

    /// Maximum visual magnitude (lower is brighter)
    #[arg(short, long, default_value_t = 6.0)]
    max_magnitude: f64,

    /// Output image width in pixels
    #[arg(long, default_value_t = 800)]
    width: u32,

    /// Output image height in pixels
    #[arg(long, default_value_t = 600)]
    height: u32,

    /// Output image file name
    #[arg(short, long, default_value = "star_map.png")]
    output: String,
}

/// Reads and filters stars from the Tycho-2 catalog file.
/// https://heasarc.gsfc.nasa.gov/w3browse/all/tycho2.html
/// http://tdc-www.harvard.edu/catalogs/tycho2.format.html
pub fn read_stars<P: AsRef<std::path::Path>>(
    path: P,
    min_ra: f64,
    max_ra: f64,
    min_dec: f64,
    max_dec: f64,
    max_magnitude: f64,
) -> Result<Vec<Star>, CatalogError> {
    let file = File::open(path)?;
    let reader = io::BufReader::new(file);
    let mut csv_reader = ReaderBuilder::new()
        .delimiter(b'|')
        .has_headers(false)
        .from_reader(reader);

    let mut stars = Vec::new();
    let mut skipped_rows = 0;

    for (i, result) in csv_reader.records().enumerate() {
        let record = result?;
        match parse_star_record(&record) {
            Ok(star) => {
                if star.ra_deg >= min_ra
                    && star.ra_deg <= max_ra
                    && star.de_deg >= min_dec
                    && star.de_deg <= max_dec
                    && star.mag <= max_magnitude
                {
                    if i % 10000 == 0 {
                        println!(
                            "Star {}: RA={}, Dec={}, Mag={}",
                            i, star.ra_deg, star.de_deg, star.mag
                        );
                    }
                    stars.push(star);
                }
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

    println!("Total stars read and filtered: {}", stars.len());
    println!("Total rows skipped: {}", skipped_rows);

    Ok(stars)
}

/// Parses a single record from the catalog into a Star struct.
fn parse_star_record(record: &csv::StringRecord) -> Result<Star, CatalogError> {
    let ra = parse_field(record, 24, "RA")?;
    let dec = parse_field(record, 25, "Dec")?;
    let mag = parse_magnitude(record)?;

    Ok(Star {
        ra_deg: ra,
        de_deg: dec,
        mag,
    })
}

/// Parses a field from the record, returning a helpful error if parsing fails.
fn parse_field(
    record: &csv::StringRecord,
    index: usize,
    field_name: &str,
) -> Result<f64, CatalogError> {
    record
        .get(index)
        .ok_or_else(|| CatalogError::MissingField(field_name.to_string()))?
        .trim()
        .parse()
        .map_err(|_| CatalogError::Parse(format!("Failed to parse {}", field_name)))
}

fn parse_magnitude(record: &csv::StringRecord) -> Result<f64, CatalogError> {
    let bt_mag = parse_field(record, 17, "BT magnitude").ok();
    let vt_mag = parse_field(record, 19, "VT magnitude").ok();

    // println!("Debug: BT_Mag = {:?}, VT_Mag = {:?}", bt_mag, vt_mag);

    match (bt_mag, vt_mag) {
        (Some(bt), Some(vt)) => {
            let v_mag = vt - 0.090 * (bt - vt);
            // println!("Debug: Calculated V_Mag = {:.3}", v_mag);
            Ok(v_mag)
        }
        (None, Some(vt)) => {
            // println!("Debug: Using VT_Mag as V_Mag = {:.3}", vt);
            Ok(vt)
        }
        (Some(bt), None) => {
            // println!("Debug: Using BT_Mag as V_Mag = {:.3}", bt);
            Ok(bt)
        }
        (None, None) => Err(CatalogError::MissingMagnitude),
    }
}

/// Gaussian blurring function
fn gaussian(x: f64, y: f64, sigma: f64) -> f64 {
    let exponent = -(x * x + y * y) / (2.0 * sigma * sigma);
    (1.0 / (2.0 * PI * sigma * sigma)) * (-exponent).exp()
}



fn apply_psf(image: &mut [Vec<f64>], star_x: usize, star_y: usize, intensity: f64, radius: f64, wavelength: f64, aperture_diameter: f64, focal_length: f64) {
    let size = (2.0 * radius).ceil() as usize;
    let half_size = size / 2;

    for dx in 0..size {
        for dy in 0..size {
            let x = star_x as isize + dx as isize - half_size as isize;
            let y = star_y as isize + dy as isize - half_size as isize;

            // Ensure the coordinates are within image bounds
            if x >= 0 && x < image.len() as isize && y >= 0 && y < image[0].len() as isize {
                let r = (((dx as f64 - half_size as f64).powi(2) + (dy as f64 - half_size as f64).powi(2)).sqrt());
                let airy_value = airy_intensity(r, wavelength, aperture_diameter, focal_length);

                // Accumulate intensity at the pixel
                image[x as usize][y as usize] += intensity * airy_value;
            }
        }
    }
}


fn render_stars(
    stars: &[Star],
    width: u32,
    height: u32,
    min_ra: f64,
    max_ra: f64,
    min_dec: f64,
    max_dec: f64,
) {
    let mut img = ImageBuffer::new(width, height);


}