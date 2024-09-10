# starfinder

`starfinder` is a Rust & Python package that provides functionality to read, process, and render star data from the Tycho-2 catalog. It's built with Rust for performance and exposes a Python API for ease of use.

## Setup

1. Download the Tycho-2 catalog:

   - Visit https://archive.eso.org/ASTROM/TYC-2/data/
   - Download `catalog.dat` and place it in `data/tycho2/`

2. Ensure your project structure looks like this:

```
.
├── Cargo.lock
├── Cargo.toml
├── README.md
├── data
│   └── tycho2
│       ├── catalog.dat
│       ├── index.dat
│       ├── suppl_1.dat
│       └── suppl_2.dat
├── poetry.lock
├── pyproject.toml
└── src
```

## Running the Renderer

### Using Cargo (Rust)

To run the renderer with default settings:

```bash
cargo run
```

To run with custom arguments:

```bash
cargo run -- --roll 0.0 --fov-w 75.0 --fov-h 50.0
```

### Command-line Arguments

| Flag            | Description                      | Default                   | Notes             |
| --------------- | -------------------------------- | ------------------------- | ----------------- |
| --source, -s    | Source file path                 | `data/tycho2/catalog.dat` |                   |
| --center-ra     | FOV center point right ascension | 180.0                     | In degrees        |
| --center-dec    | FOV center point declination     | 0.0                       | In degrees        |
| --fov-w         | Width of FOV                     | 60.0                      | In degrees        |
| --fov-h         | Height of FOV                    | 45.0                      | In degrees        |
| --roll          | Camera sensor roll               | 0.0                       | In degrees        |
| --max-magnitude | Maximum visual magnitude         | 12.0                      | Lower is brighter |
| --lambda-nm     | Targeted wavelength              | 540.0                     | In nanometers     |
| --pixel-size-m  | Simulated sensor pixel size      | 3e-6                      | In meters         |
| --width         | Output image width               | 800                       | In pixels         |
| --height        | Output image height              | 600                       | In pixels         |
| --output, -o    | Output filename                  | `star_map.png`            |                   |

## Python Installation and Usage

### Installation

Ensure you have Python 3.8 or later, then:

`bash`
pip install starfinder

````
Or if you want to install the package in development mode:

```bash
pipx install maturin
maturin develop
````

### Basic Usage

```python
from starfinder import StarCatalogArgs, process_star_catalog_py

args = StarCatalogArgs(
    source="data/tycho2/catalog.dat",
    center_ra=180.0,
    center_dec=0.0,
    fov_w=60.0,
    fov_h=45.0,
    roll=0.0,
    max_magnitude=6.0,
    lambda_nm=540.0,
    pixel_size_m=3e-6,
    width=800,
    height=600,
    output="star_map.png"
)

process_star_catalog_py(args)
```

## API Reference

### `StarCatalogArgs`

Parameters:

- `source` (str): Path to the Tycho-2 catalog file
- `center_ra` (float): Right Ascension of FOV center (degrees)
- `center_dec` (float): Declination of FOV center (degrees)
- `fov_w` (float): FOV width (degrees)
- `fov_h` (float): FOV height (degrees)
- `roll` (float): Camera roll (degrees)
- `max_magnitude` (float): Maximum visual magnitude
- `lambda_nm` (float): Targeted wavelength (nanometers)
- `pixel_size_m` (float): Sensor pixel size (meters)
- `width` (int): Output image width (pixels)
- `height` (int): Output image height (pixels)
- `output` (str): Output image filename

### `process_star_catalog_py(args: StarCatalogArgs) -> None`

Processes the star catalog based on the provided arguments.

## Contributing

Contributions to `starfinder` are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the GPLv3 License - see the LICENSE file for details.
