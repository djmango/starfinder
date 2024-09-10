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
│   └── main.rs
```

To run the airy disc renderer
```
cargo run --bin airy_renderer
```

To run the original proof of concept generator:
```
cargo run -- -- --min-ra 0 --max-ra 60 --min-dec -30 --max-dec 30 --max-magnitude 5 --width 1000 --height 800 --output star_map.png
```
