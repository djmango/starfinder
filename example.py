from starfinder import StarCatalogArgs, process_star_catalog_py

# Create arguments for star catalog processing
args = StarCatalogArgs(
    file="data/tycho2/catalog.dat",
    display_count=10,
    min_ra=0.0,
    max_ra=360.0,
    min_dec=-90.0,
    max_dec=90.0,
    max_magnitude=6.0,
    width=800,
    height=600,
    output="star_map.png",
)

# Process the star catalog
process_star_catalog_py(args)
