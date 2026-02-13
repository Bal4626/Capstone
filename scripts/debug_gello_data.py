import pickle
import os
from pathlib import Path

def debug_gello_data(gello_data_dir):
    """Debug script to understand the structure of GELLO pickle files"""
    data_dir = Path(gello_data_dir)
    
    # Get all pickle files
    pkl_files = list(data_dir.glob("*.pkl"))
    if not pkl_files:
        print(f"No .pkl files found in {gello_data_dir}")
        return
    
    print(f"Found {len(pkl_files)} pickle files")
    pkl_files.sort()
    
    # Inspect first 5 files in detail
    for i, file_path in enumerate(pkl_files[:5]):
        print(f"\n{'='*60}")
        print(f"File {i+1}: {file_path.name}")
        print(f"{'='*60}")
        
        try:
            with open(file_path, 'rb') as f:
                data = pickle.load(f)
            
            print(f"Data type: {type(data)}")
            print(f"Data: {data}")
            
            if hasattr(data, '__dict__'):
                print(f"Object attributes: {data.__dict__.keys()}")
                for key, value in data.__dict__.items():
                    print(f"  {key}: {type(value)} = {value}")
            
            elif isinstance(data, dict):
                print(f"Dictionary keys: {list(data.keys())}")
                for key, value in data.items():
                    if isinstance(value, (list, np.ndarray)) and hasattr(value, 'shape'):
                        print(f"  {key}: {type(value)} shape {value.shape}")
                    else:
                        print(f"  {key}: {type(value)} = {value}")
            
            elif isinstance(data, (list, tuple)):
                print(f"Sequence length: {len(data)}")
                for j, item in enumerate(data[:3]):  # First 3 items
                    print(f"  Item {j}: {type(item)}")
                    if hasattr(item, '__dict__'):
                        print(f"    Attributes: {item.__dict__.keys()}")
            
            else:
                print(f"Simple data type: {type(data)}")
                print(f"Value: {data}")
                
        except Exception as e:
            print(f"ERROR reading file: {e}")
            continue
    
    # Also check what keys are common across multiple files
    print(f"\n{'='*60}")
    print("CHECKING COMMON STRUCTURE ACROSS FILES")
    print(f"{'='*60}")
    
    all_keys = []
    for i, file_path in enumerate(pkl_files[:10]):  # Check first 10 files
        try:
            with open(file_path, 'rb') as f:
                data = pickle.load(f)
            
            if hasattr(data, '__dict__'):
                keys = list(data.__dict__.keys())
            elif isinstance(data, dict):
                keys = list(data.keys())
            else:
                keys = [f"type_{type(data).__name__}"]
            
            all_keys.append(set(keys))
            print(f"File {i+1} keys: {keys}")
            
        except Exception as e:
            print(f"File {i+1} error: {e}")
            continue
    
    if all_keys:
        common_keys = set.intersection(*all_keys)
        print(f"\nCommon keys across all files: {common_keys}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--gello-data-dir', type=str, required=True)
    args = parser.parse_args()
    debug_gello_data(args.gello_data_dir)