from starfinder import StarCatalogArgs, process_star_catalog_py

# Create arguments for star catalog processing. Remember the data needs to be optimized before use
args = StarCatalogArgs(
    source="data/optimized.dat",
    display_count=10,
    min_ra=0.0,
    max_ra=360.0,
    min_dec=-90.0,
    max_dec=90.0,git
    max_magnitude=6.0,
    width=800,
    height=600,
    output="renders/star_map.png",
)

# Process the star catalog
process_star_catalog_py(args)
