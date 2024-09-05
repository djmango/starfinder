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