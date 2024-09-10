use clap::Parser;
use starfield_renderer::parsing_utils::{parse_field, parse_star_record, parse_magnitude};

/// CLI Arguments
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
pub struct Args {

}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    Ok(())
}
