import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Simplified star catalog (RA in hours, Dec in degrees, magnitude)
STAR_CATALOG = [
    (0.14, 29.09, 2.07),  # Alpheratz
    (2.52, 89.26, 1.97),  # Polaris
    (5.92, 7.41, 0.50),  # Betelgeuse
    (6.75, -16.72, -1.46),  # Sirius
    (14.66, -60.83, 0.01),  # Alpha Centauri
    (18.62, 38.78, 0.03),  # Vega
    (22.96, -29.62, 1.17),  # Fomalhaut
]


def load_star_catalog():
    return np.array(STAR_CATALOG)


def ra_dec_to_eci(ra, dec):
    ra_rad = ra * np.pi / 12  # Convert RA to radians
    dec_rad = dec * np.pi / 180  # Convert Dec to radians
    x = np.cos(ra_rad) * np.cos(dec_rad)
    y = np.sin(ra_rad) * np.cos(dec_rad)
    z = np.sin(dec_rad)
    return np.array([x, y, z])


def quaternion_to_rotation_matrix(q):
    q0, q1, q2, q3 = q
    return np.array(
        [
            [
                1 - 2 * q2**2 - 2 * q3**2,
                2 * q1 * q2 - 2 * q0 * q3,
                2 * q1 * q3 + 2 * q0 * q2,
            ],
            [
                2 * q1 * q2 + 2 * q0 * q3,
                1 - 2 * q1**2 - 2 * q3**2,
                2 * q2 * q3 - 2 * q0 * q1,
            ],
            [
                2 * q1 * q3 - 2 * q0 * q2,
                2 * q2 * q3 + 2 * q0 * q1,
                1 - 2 * q1**2 - 2 * q2**2,
            ],
        ]
    )


def eci_to_body_frame(eci_coords, attitude_quaternion):
    R = quaternion_to_rotation_matrix(attitude_quaternion)
    return R.T @ eci_coords


def project_to_image_plane(body_coords, fov, resolution):
    f = 1 / np.tan(np.radians(fov) / 2)
    x, y, z = body_coords
    if z > 0:
        u = f * x / z
        v = f * y / z
        width, height = resolution
        pixel_x = int((u + 1) * width / 2)
        pixel_y = int((1 - v) * height / 2)
        if 0 <= pixel_x < width and 0 <= pixel_y < height:
            return pixel_x, pixel_y
    return None


def calculate_star_visibility(magnitude, iso, shutter_speed):
    # Simplified visibility calculation
    brightness = 10 ** (-magnitude / 2.5)  # Convert magnitude to linear scale
    exposure = iso * shutter_speed / 100  # Simplified exposure calculation
    return brightness * exposure


def generate_star_image(stars, attitude, fov, resolution, iso, shutter_speed):
    width, height = resolution
    image = np.zeros((height, width))

    for ra, dec, magnitude in stars:
        eci_coords = ra_dec_to_eci(ra, dec)
        body_coords = eci_to_body_frame(eci_coords, attitude)
        projection = project_to_image_plane(body_coords, fov, resolution)

        if projection:
            x, y = projection
            visibility = calculate_star_visibility(magnitude, iso, shutter_speed)
            image[y, x] += visibility

    # Normalize and apply a simple brightness curve
    image = np.log1p(image)
    image /= np.max(image)
    return image


def main():
    stars = load_star_catalog()

    # Example inputs (you can modify these or add user input)
    attitude = np.array([1, 0, 0, 0])  # Identity quaternion (no rotation)
    fov = 50  # degrees
    resolution = (800, 600)
    iso = 1600
    shutter_speed = 1  # second

    image = generate_star_image(stars, attitude, fov, resolution, iso, shutter_speed)

    plt.figure(figsize=(10, 7.5))
    plt.imshow(image, cmap="gray")
    plt.title(f"Star Field (FOV: {fov}°, ISO: {iso}, Shutter: {shutter_speed}s)")
    plt.axis("off")
    plt.show()


if __name__ == "__main__":
    main()
