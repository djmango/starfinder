# starfinder

`starfinder` is a Rust & Python package that provides functionality to read, process, and render star data from the Tycho-2 catalog. It's built with Rust for performance and exposes a Python API for ease of use.

https://archive.eso.org/ASTROM/TYC-2/data/

Download catalog.dat from there, put it in data/tycho2/

```
.
├── Cargo.lock
├── Cargo.toml
├── README.md
├── data
│   └── tycho2
│   ├── catalog.dat
│   ├── index.dat
│   ├── suppl_1.dat
│   └── suppl_2.dat
├── poetry.lock
├── pyproject.toml
├── src
│   └── main.rs
```

# Rust

```
cargo run --release -- --min-ra=0 --max-ra=60 --min-dec=-30 --max-dec=30 --max-magnitude=11 --width=1000 --height=800 --output=example.png
```

# C++

```
mkdir build
cd build
cmake ..
make
render --max-ra=60 --min-dec=-30 --max-dec=30 --max-magnitude=11 --width=1000 --height=800 --output=example.png ../data/tycho2/catalog.dat
```

## Installation

To install `starfinder`, you can use pip:
https://pyo3.rs/v0.22.2/getting-started

```bash
pipx install maturin
maturin develop
```

Note: This package requires Python 3.8 or later.

## Usage

Here's a basic example of how to use `starfinder`:

```python
from starfinder import StarCatalogArgs, process_star_catalog_py

# Create arguments for star catalog processing
args = StarCatalogArgs(
    file="path/to/your/tycho2_catalog.dat",
    display_count=10,
    min_ra=0.0,
    max_ra=360.0,
    min_dec=-90.0,
    max_dec=90.0,
    max_magnitude=6.0,
    width=800,
    height=600,
    output="star_map.png"
)

# Process the star catalog
process_star_catalog_py(args)
```

This will read the Tycho-2 catalog, filter the stars based on the given parameters, and generate a star map image.

## API Reference

### `StarCatalogArgs`

This class represents the arguments for star catalog processing.

Parameters:

- `file` (str): Path to the Tycho-2 catalog file
- `display_count` (int): Number of stars to display in the console output (0 for all)
- `min_ra` (float): Minimum Right Ascension in degrees
- `max_ra` (float): Maximum Right Ascension in degrees
- `min_dec` (float): Minimum Declination in degrees
- `max_dec` (float): Maximum Declination in degrees
- `max_magnitude` (float): Maximum visual magnitude (lower is brighter)
- `width` (int): Output image width in pixels
- `height` (int): Output image height in pixels
- `output` (str): Output image file name

### `process_star_catalog_py(args: StarCatalogArgs) -> None`

This function processes the star catalog based on the provided arguments.

## Example

Here's a more detailed example that demonstrates how to use `starfinder` to create a star map of the brightest stars:

```python
from starfinder import StarCatalogArgs, process_star_catalog_py

# Create arguments for star catalog processing
args = StarCatalogArgs(
    file="tycho2_catalog.dat",
    display_count=20,  # Display info for the 20 brightest stars
    min_ra=0.0,
    max_ra=360.0,
    min_dec=-90.0,
    max_dec=90.0,
    max_magnitude=3.0,  # Only include stars brighter than magnitude 3
    width=1200,
    height=800,
    output="bright_stars_map.png"
)

# Process the star catalog
process_star_catalog_py(args)

print(f"Star map has been generated: {args.output}")
```

This script will create a star map of the brightest stars (magnitude 3.0 or brighter) across the entire sky, output information about the 20 brightest stars to the console, and save the star map as "bright_stars_map.png".
