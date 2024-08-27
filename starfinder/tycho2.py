import pandas as pd
import csv
from pathlib import Path


class Tycho2Reader:
    def __init__(self, data_dir):
        self.data_dir = Path(data_dir)

    def read_catalog(self):
        file_path = self.data_dir / "catalog.dat"
        columns = [
            "TYC1",
            "TYC2",
            "TYC3",
            "pflag",
            "mRAdeg",
            "mDEdeg",
            "pmRA*",
            "pmDE",
            "e_mRA*",
            "e_mDE",
            "e_pmRA*",
            "e_pmDE",
            "mepRA",
            "mepDE",
            "Num",
            "g_mRA",
            "g_mDE",
            "g_pmRA",
            "g_pmDE",
            "BT",
            "e_BT",
            "VT",
            "e_VT",
            "prox",
            "TYC",
            "HIP",
            "CCDM",
            "RAdeg",
            "DEdeg",
            "epRA",
            "epDE",
            "e_RA*",
            "e_DE",
            "posflg",
            "corr",
        ]

        df = pd.read_csv(
            file_path,
            sep="|",
            names=columns,
            header=None,
            low_memory=False,
        )
        return df

    def read_supplement(self, supplement_num):
        file_path = self.data_dir / f"suppl_{supplement_num}.dat"
        columns = [
            "TYC1",
            "TYC2",
            "TYC3",
            "pflag",
            "RAdeg",
            "DEdeg",
            "pmRA",
            "pmDE",
            "e_RA",
            "e_DE",
            "e_pmRA",
            "e_pmDE",
            "epRA",
            "epDE",
            "Num",
            "BT",
            "e_BT",
            "VT",
            "e_VT",
            "prox",
        ]
        df = pd.read_csv(file_path, sep="|", names=columns, header=None)
        return df

    def read_index(self):
        file_path = self.data_dir / "index.dat"
        columns = ["rec_t2", "rec_s2", "RAmin", "RAmax", "DEmin", "DEmax"]
        df = pd.read_csv(file_path, sep=r"\s+", names=columns, header=None)
        return df

    def read_stars(self):
        """Read stars from the Tycho-2 catalog and return a DataFrame with RA, Dec, and magnitude. Simpler than the original read_catalog method."""
        stars = []
        skipped_rows = 0
        with open(self.data_dir / "catalog.dat", "r") as file:
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
