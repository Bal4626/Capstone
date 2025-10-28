import pickle

# === edit this path ===
filename = "/home/balraj/bc_data/gello/1028_172831/2025-10-28T17:28:31.492082.pkl"   # replace with your pickle file name

with open(filename, "rb") as f:
    data = pickle.load(f)

print(" Pickle loaded successfully!")
print("Type:", type(data))
print("Keys/Length:", len(data) if hasattr(data, "__len__") else "N/A")
print("\nPreview:")
print(data if len(str(data)) < 500 else str(data)[:500] + "...")
