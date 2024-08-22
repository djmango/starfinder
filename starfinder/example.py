from starfinder.tycho2 import Tycho2Reader

reader = Tycho2Reader("data/tycho2")

# Read main catalog
catalog_df = reader.read_catalog()
print("Catalog shape:", catalog_df.shape)
print(catalog_df.head())

# Read supplement 1
suppl1_df = reader.read_supplement(1)
print("\nSupplement 1 shape:", suppl1_df.shape)
print(suppl1_df.head())

# Read supplement 2
suppl2_df = reader.read_supplement(2)
print("\nSupplement 2 shape:", suppl2_df.shape)
print(suppl2_df.head())

# Read index
index_df = reader.read_index()
print("\nIndex shape:", index_df.shape)
print(index_df.head())
