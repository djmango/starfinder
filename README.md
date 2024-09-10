# Setup
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
        