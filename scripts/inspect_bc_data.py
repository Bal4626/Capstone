import zipfile
import pickle
import json
import numpy as np
from pathlib import Path
import os

def inspect_bc_zipfile(zip_path):
    """Inspect the contents of a BC data zipfile"""
    zip_path = Path(zip_path)
    
    if not zip_path.exists():
        print(f"Zipfile not found: {zip_path}")
        return
    
    print(f"Inspecting: {zip_path}")
    print("=" * 60)
    
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        # List all files in zip
        file_list = zip_ref.namelist()
        print(f"Total files in zip: {len(file_list)}")
        
        # Group by file types
        pkl_files = [f for f in file_list if f.endswith('.pkl')]
        json_files = [f for f in file_list if f.endswith('.json')]
        npy_files = [f for f in file_list if f.endswith('.npy')]
        
        print(f"Pickle files: {len(pkl_files)}")
        print(f"JSON files: {len(json_files)}")
        print(f"NPY files: {len(npy_files)}")
        print(f"Other files: {len(file_list) - len(pkl_files) - len(json_files) - len(npy_files)}")
        
        # Inspect directory structure
        directories = set()
        for file in file_list:
            dir_path = os.path.dirname(file)
            if dir_path:
                directories.add(dir_path)
        
        print(f"\nDirectory structure:")
        for dir_path in sorted(directories):
            files_in_dir = [f for f in file_list if f.startswith(dir_path + '/')]
            print(f"  {dir_path}/ -> {len(files_in_dir)} files")
        
        # Sample inspection of first few files of each type
        print(f"\n{'='*60}")
        print("SAMPLE FILE INSPECTION")
        print(f"{'='*60}")
        
        # Inspect first 3 pickle files
        if pkl_files:
            print(f"\nInspecting pickle files:")
            for pkl_file in pkl_files[:3]:
                print(f"\n  {pkl_file}:")
                try:
                    with zip_ref.open(pkl_file) as f:
                        data = pickle.load(f)
                    print(f"    Type: {type(data)}")
                    if isinstance(data, dict):
                        print(f"    Keys: {list(data.keys())}")
                        for key, value in data.items():
                            if hasattr(value, 'shape'):
                                print(f"      {key}: shape {value.shape}, dtype {value.dtype}")
                            else:
                                print(f"      {key}: {type(value)}")
                    elif hasattr(data, '__dict__'):
                        print(f"    Attributes: {data.__dict__.keys()}")
                    else:
                        print(f"    Data: {data}")
                except Exception as e:
                    print(f"    ERROR reading: {e}")
        
        # Inspect JSON files
        if json_files:
            print(f"\nInspecting JSON files:")
            for json_file in json_files[:2]:
                print(f"\n  {json_file}:")
                try:
                    with zip_ref.open(json_file) as f:
                        data = json.load(f)
                    print(f"    Content: {data}")
                except Exception as e:
                    print(f"    ERROR reading: {e}")
        
        # Inspect NPY files
        if npy_files:
            print(f"\nInspecting NPY files:")
            for npy_file in npy_files[:3]:
                print(f"\n  {npy_file}:")
                try:
                    with zip_ref.open(npy_file) as f:
                        data = np.load(f)
                    print(f"    Shape: {data.shape}, dtype: {data.dtype}")
                    print(f"    Range: [{data.min():.3f}, {data.max():.3f}]")
                except Exception as e:
                    print(f"    ERROR reading: {e}")

def extract_and_inspect_bc_data(zip_path, extract_dir="bc_data_extracted"):
    """Extract and thoroughly inspect BC data"""
    zip_path = Path(zip_path)
    extract_dir = Path(extract_dir)
    
    print(f"Extracting {zip_path} to {extract_dir}")
    print("=" * 60)
    
    # Extract all files
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_dir)
    
    # Analyze extracted structure
    print(f"Extracted to: {extract_dir}")
    
    # Find all demonstration episodes
    episode_dirs = []
    for item in extract_dir.rglob("*"):
        if item.is_dir() and any(item.glob("*.pkl")):
            episode_dirs.append(item)
    
    print(f"\nFound {len(episode_dirs)} potential episode directories")
    
    # Analyze each episode
    for episode_dir in episode_dirs[:3]:  # First 3 episodes
        print(f"\n{'='*50}")
        print(f"EPISODE: {episode_dir.relative_to(extract_dir)}")
        print(f"{'='*50}")
        
        pkl_files = list(episode_dir.glob("*.pkl"))
        json_files = list(episode_dir.glob("*.json")) 
        npy_files = list(episode_dir.glob("*.npy"))
        
        print(f"  PKL files: {len(pkl_files)}")
        print(f"  JSON files: {len(json_files)}")
        print(f"  NPY files: {len(npy_files)}")
        
        # Analyze timestamp pattern
        if pkl_files:
            timestamps = [f.name for f in pkl_files]
            print(f"  Timestamp range: {timestamps[0]} to {timestamps[-1]}")
            
            # Check if this is the GELLO format we expect
            sample_file = pkl_files[0]
            try:
                with open(sample_file, 'rb') as f:
                    data = pickle.load(f)
                print(f"  Sample data keys: {list(data.keys()) if isinstance(data, dict) else 'N/A'}")
            except Exception as e:
                print(f"  Error reading sample: {e}")
        
        # Check for instruction files
        if json_files:
            for json_file in json_files:
                if 'instruction' in json_file.name.lower():
                    try:
                        with open(json_file, 'r') as f:
                            instruction = json.load(f)
                        print(f"  Instruction: {instruction}")
                    except:
                        pass

def find_bc_zipfiles(search_dir="."):
    """Find all BC data zipfiles in directory"""
    search_dir = Path(search_dir)
    zip_files = list(search_dir.rglob("*.zip"))
    bc_zip_files = [f for f in zip_files if 'bc' in f.name.lower() or 'data' in f.name.lower()]
    
    print(f"Found {len(bc_zip_files)} potential BC data zipfiles:")
    for zip_file in bc_zip_files:
        print(f"  {zip_file}")
    
    return bc_zip_files

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Inspect BC data zipfiles")
    parser.add_argument('--zip-path', type=str, help='Path to specific zipfile')
    parser.add_argument('--search-dir', type=str, default='.', help='Directory to search for zipfiles')
    parser.add_argument('--extract', action='store_true', help='Extract and analyze contents')
    
    args = parser.parse_args()
    
    if args.zip_path:
        if args.extract:
            extract_and_inspect_bc_data(args.zip_path)
        else:
            inspect_bc_zipfile(args.zip_path)
    else:
        find_bc_zipfiles(args.search_dir)