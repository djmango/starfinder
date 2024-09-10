# starfinder

`starfinder` is a Rust & Python package that provides functionality to read, process, and render star data from the Tycho-2 catalog. It's built with Rust for performance and exposes a Python API for ease of use.

## Setup
https://archive.eso.org/ASTROM/TYC-2/data/

Download catalog.dat from there, put it in data/tycho2/

```
.
├── Cargo.lock
├── Cargo.toml
├── README.md
├── data
│   └── tycho2
│   ├── catalog.dat
│   ├── index.dat
│   ├── suppl_1.dat
│   └── suppl_2.dat
├── poetry.lock
├── pyproject.toml
├── src
│   └── bin
│       └── main.rs
│       └── data-transformer.rs
```

# Run
To run the renderer with defaults
```
cargo run
```

To run the renderer with cmd arg overrides:
```
cargo run -- --roll-deg 0.0 --fov-w-deg 75.0 --fov-h-deg 50.0
```
### Cmd args
<table>
    <tr>
        <td>██ Flag ██</td>
        <td>██ Description ██</td>
        <td>██ Default value ██</td>
        <td>██ Notes ██</td>
    </tr>
    <tr>
        <td>--source, -s</td>
        <td>The source file to run</td>
        <td>`data/tycho2/catalog.dat`</td>
        <td></td>
    </tr>
    <tr>
        <td>--center-ra</td>
        <td>FOV center point right ascension</td>
        <td>`180.0`</td>
        <td>In degrees</td>
    </tr>
    <tr>
        <td>--center-dec</td>
        <td>FOV center point declination</td>
        <td>`0.0`</td>
        <td>In degrees</td>
    </tr>
    <tr>
        <td>--fov-w</td>
        <td>Width of FOV</td>
        <td>`60.0`</td>
        <td>In degrees</td>
    </tr>
    <tr>
        <td>--fov-h</td>
        <td>Height of FOV</td>
        <td>`45.0`</td>
        <td>In degrees</td>
    </tr>
    <tr>
        <td>--roll</td>
        <td>Critical for determining FOV and should always be specified. Camera sensor roll with respect to the celestial sphere.</td>
        <td>`0.0`</td>
        <td>In degrees</td>
    </tr>
    <tr>
        <td>--max-magnitude</td>
        <td>Maximum visual magnitude (lower is brighter).</td>
        <td>`12.0`</td>
        <td>This is essentially a filter, and dimmer stars will be ignored. A higher value can have an effect on performance at the cost of realism. Defaults to a maximum dimmer than Tycho2 dataset - i.e. shows all stars.</td>
    </tr>
    <tr>
        <td>--lambda-nm</td>
        <td>Targeted wavelength - critical for airy disc rendering (nanometers). Default to visible spectrum</td>
        <td>`540.0`</td>
        <td>In nanometers. Currently unused - for future revision with PSF rendering</td>
    </tr>
    <tr>
        <td>--pixel-size-m</td>
        <td>Simulated sensor physical pixel size</td>
        <td>`3e-6`</td>
        <td>In meters. Currently unused - for future revision with PSF rendering</td>
    </tr>
    <tr>
        <td>--width</td>
        <td>Output image width. Should match sensor pixel count.</td>
        <td>`800`</td>
        <td>In pixels. Larger values will impact performance.</td>
    </tr>
    <tr>
        <td>--height</td>
        <td>Output image height. Should match sensor pixel count.</td>
        <td>`600`</td>
        <td>In pixels. Larger values will impact performance.</td>
    </tr>
    <tr>
        <td>--output, -o</td>
        <td>Output filename</td>
        <td>`star_map.png`</td>
        <td></td>
    </tr>
</table>


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
