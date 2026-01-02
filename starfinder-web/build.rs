use std::env;
use std::fs;
use std::io::{self, Write};
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    // Get the cache directory path
    let cache_dir = if let Some(base_dirs) = directories::BaseDirs::new() {
        base_dirs.cache_dir().to_path_buf()
    } else {
        // Fallback to OUT_DIR if we can't get cache dir
        PathBuf::from(env::var("OUT_DIR").unwrap())
    };

    let tycho2_dir = cache_dir.join("starfinder/data/tycho2");
    let tycho2_path = tycho2_dir.join("catalog.dat");

    // Skip if already downloaded
    if tycho2_path.exists() {
        println!(
            "cargo:warning=Tycho-2 catalog already exists at {:?}",
            tycho2_path
        );
        return;
    }

    println!("cargo:warning=Downloading Tycho-2 catalog (~500MB)...");

    // Create directory
    fs::create_dir_all(&tycho2_dir).expect("Failed to create tycho2 directory");

    // Download the catalog with streaming to avoid timeout
    let url = "https://archive.eso.org/ASTROM/TYC-2/data/catalog.dat";
    let mut response = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(1800)) // 30 minutes total timeout
        .connect_timeout(std::time::Duration::from_secs(60))
        .build()
        .expect("Failed to build HTTP client")
        .get(url)
        .send()
        .expect("Failed to download Tycho-2 catalog");

    // Stream the response to file instead of loading into memory
    let mut file = fs::File::create(&tycho2_path).expect("Failed to create catalog file");
    std::io::copy(&mut response, &mut file)
        .expect("Failed to write catalog file");

    println!(
        "cargo:warning=Tycho-2 catalog downloaded to {:?}",
        tycho2_path
    );
}
