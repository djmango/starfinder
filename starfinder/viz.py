import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import csv
# from starfinder.tycho2 import Tycho2Reader


class Tycho2Reader:
    def __init__(self, filename):
        self.filename = filename

    def read_stars(self):
        stars = []
        skipped_rows = 0
        with open(self.filename, "r") as file:
            reader = csv.reader(file, delimiter="|")
            for i, row in enumerate(reader):
                try:
                    ra = float(row[24].strip())
                    dec = float(row[25].strip())

                    # Try to get VT magnitude, fallback to BT if VT is empty
                    vt_mag = row[20].strip()
                    if vt_mag:
                        mag = float(vt_mag)
                    else:
                        bt_mag = row[17].strip()
                        if bt_mag:
                            mag = float(bt_mag)
                        else:
                            # Skip this star if both VT and BT are missing
                            raise ValueError("Both VT and BT magnitudes are missing")

                    stars.append({"RAdeg": ra, "DEdeg": dec, "Mag": mag})

                    # Debug output for every 10000th star
                    if i % 10000 == 0:
                        print(f"Star {i}: RA={ra}, Dec={dec}, Mag={mag}")

                except (ValueError, IndexError) as e:
                    skipped_rows += 1
                    if skipped_rows <= 10:
                        print(f"Skipping row {i} due to error: {e}")
                        print(f"Problematic row: {row}")
                    elif skipped_rows == 11:
                        print("Further skipped rows will not be printed...")

        print(f"Total stars read: {len(stars)}")
        print(f"Total rows skipped: {skipped_rows}")
        return pd.DataFrame(stars)


def ra_dec_to_eci(ra, dec):
    ra_rad = np.radians(ra)
    dec_rad = np.radians(dec)
    x = np.cos(ra_rad) * np.cos(dec_rad)
    y = np.sin(ra_rad) * np.cos(dec_rad)
    z = np.sin(dec_rad)
    return np.array([x, y, z])


def eci_to_body_frame(eci_coords, attitude_matrix):
    return attitude_matrix @ eci_coords


def project_to_image_plane(body_coords, fov, resolution):
    f = 1 / np.tan(np.radians(fov) / 2)
    x, y, z = body_coords
    if z > 0:
        u = f * x / z
        v = f * y / z
        width, height = resolution
        pixel_x = int((u + 1) * width / 2)
        pixel_y = int((v + 1) * height / 2)
        if 0 <= pixel_x < width and 0 <= pixel_y < height:
            return pixel_x, pixel_y
    return None


def calculate_star_visibility(magnitude, iso, shutter_speed):
    brightness = 10 ** (-(magnitude - 6) / 2.5)
    exposure = iso * shutter_speed / 100
    return brightness * exposure


def add_star_to_image(image, x, y, visibility):
    height, width = image.shape
    y_min, y_max = max(0, y - 2), min(height, y + 3)
    x_min, x_max = max(0, x - 2), min(width, x + 3)

    y_range = np.arange(y_min, y_max)
    x_range = np.arange(x_min, x_max)

    y_grid, x_grid = np.meshgrid(y_range - y, x_range - x, indexing="ij")

    r_squared = x_grid**2 + y_grid**2
    gaussian = np.exp(-r_squared / (2 * 0.5**2))

    image[y_min:y_max, x_min:x_max] += visibility * gaussian


def generate_star_image(stars_df, attitude_matrix, fov, resolution, iso, shutter_speed):
    width, height = resolution
    image = np.zeros((height, width))
    visible_stars = 0
    total_stars = len(stars_df)
    stars_in_fov = 0

    for _, star in stars_df.iterrows():
        ra, dec, magnitude = star["RAdeg"], star["DEdeg"], star["Mag"]
        if np.isnan(ra) or np.isnan(dec) or np.isnan(magnitude):
            print("Skipping star with missing data")
            continue  # Skip stars with missing data

        eci_coords = ra_dec_to_eci(ra, dec)
        body_coords = eci_to_body_frame(eci_coords, attitude_matrix)

        # Check if star is in front of the camera
        if body_coords[2] > 0:
            stars_in_fov += 1

        projection = project_to_image_plane(body_coords, fov, resolution)

        if projection:
            x, y = projection
            visibility = calculate_star_visibility(magnitude, iso, shutter_speed)
            add_star_to_image(image, x, y, visibility)
            visible_stars += 1
            if visible_stars <= 10:  # Print details for first 10 visible stars
                print(
                    f"Star: RA={ra:.2f}, Dec={dec:.2f}, Mag={magnitude:.2f}, Pixel=({x},{y})"
                )

    print(f"Total stars: {total_stars}")
    print(f"Stars in front of camera: {stars_in_fov}")
    print(f"Visible stars: {visible_stars}")
    return image


# Load Tycho-2 data
reader = Tycho2Reader("data/tycho2/catalog.dat")
stars_df = reader.read_stars()

# Convert columns to numeric type and filter brighter stars
stars_df["RAdeg"] = pd.to_numeric(stars_df["RAdeg"], errors="coerce")
stars_df["DEdeg"] = pd.to_numeric(stars_df["DEdeg"], errors="coerce")
stars_df["Mag"] = pd.to_numeric(stars_df["Mag"], errors="coerce")

# Filter brighter stars
bright_stars = stars_df[stars_df["Mag"] <= 9].dropna(subset=["RAdeg", "DEdeg", "Mag"])

print(f"Number of bright stars: {len(bright_stars)}")
print(
    f"RA range: {bright_stars['RAdeg'].min():.2f} to {bright_stars['RAdeg'].max():.2f}"
)
print(
    f"Dec range: {bright_stars['DEdeg'].min():.2f} to {bright_stars['DEdeg'].max():.2f}"
)

# Set parameters
fov = 90  # Field of view in degrees
resolution = (800, 600)
iso = 3200
shutter_speed = 2

# Set camera attitude (pointing towards a region with more vertical spread)
ra_center = 0  # Right ascension of vernal equinox
dec_center = 45  # Midway between celestial equator and north celestial pole
rot = Rotation.from_euler("zyx", [-ra_center, 90 - dec_center, 0], degrees=True)
attitude_matrix = rot.as_matrix()

# Generate image
image = generate_star_image(
    bright_stars, attitude_matrix, fov, resolution, iso, shutter_speed
)

# Apply logarithmic scaling and normalization
image = np.log1p(image)
image /= np.max(image) if np.max(image) > 0 else 1

# Display the image
plt.figure(figsize=(12, 9))
plt.imshow(image, cmap="viridis", origin="lower")
plt.title(f"Tycho-2 Star Field (FOV: {fov}°, ISO: {iso}, Shutter: {shutter_speed}s)")
plt.axis("off")
plt.colorbar(label="Relative Brightness (log scale)")
plt.show()

# Print some statistics about the image
print(f"Image shape: {image.shape}")
print(f"Min pixel value: {image.min()}")
print(f"Max pixel value: {image.max()}")
print(f"Mean pixel value: {image.mean()}")
