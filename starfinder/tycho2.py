import pandas as pd
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
            file_path, sep="|", names=columns, header=None, low_memory=False
        )
        return df

    def read_supplement(self, supplement_num):
        file_path = self.data_dir / f"suppl_{supplement_num}.dat"
        columns = [
            "TYC1",
            "TYC2",
            "TYC3",
            "flag",
            "RAdeg",
            "DEdeg",
            "pmRA*",
            "pmDE",
            "e_RA*",
            "e_DE",
            "e_pmRA*",
            "e_pmDE",
            "mflag",
            "BT",
            "e_BT",
            "VT",
            "e_VT",
            "prox",
            "TYC",
            "HIP",
            "CCDM",
        ]
        df = pd.read_csv(
            file_path, sep="|", names=columns, header=None, low_memory=False
        )
        return df

    def read_index(self):
        file_path = self.data_dir / "index.dat"
        columns = ["rec_t2", "rec_s2", "RAmin", "RAmax", "DEmin", "DEmax"]
        df = pd.read_csv(
            file_path, sep=r"\s+", names=columns, header=None, low_memory=False
        )
        return df
