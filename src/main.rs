use clap::Parser;

use starfinder::{process_star_catalog, StarCatalogArgs};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct CliArgs {
    /// Path to the Tycho-2 catalog file
    #[arg(
        short,
        long,
        value_name = "FILE",
        default_value = "data/tycho2/catalog.dat"
    )]
    file: String,

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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli_args = CliArgs::parse();

    let args = StarCatalogArgs {
        file: cli_args.file,
        display_count: cli_args.display_count,
        min_ra: cli_args.min_ra,
        max_ra: cli_args.max_ra,
        min_dec: cli_args.min_dec,
        max_dec: cli_args.max_dec,
        max_magnitude: cli_args.max_magnitude,
        width: cli_args.width,
        height: cli_args.height,
        output: cli_args.output,
    };

    process_star_catalog(args)
}
