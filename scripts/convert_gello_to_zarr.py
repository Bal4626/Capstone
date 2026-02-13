#!/usr/bin/env python3
"""
Convert GELLO demonstration data from ZIP file to Zarr format for Diffusion Policy.

This script converts GELLO data stored in a zipfile into the exact format expected
by Diffusion Policy, matching the structure created by convert_gello_to_zarr.py.

Key differences from original script:
- Uses correct Zarr structure: data/obs/joint_pos, data/action (not data/episode_0)
- Separates joint_pos and gripper_pos in observations
- Creates proper episode_ends array in meta/
- No shape_meta in attributes (Diffusion Policy doesn't use it)

Usage:
    python convert_gello_zip_to_zarr.py --zip-path bc_data.zip --output ur5_demos.zarr
"""

import pickle
import zarr
import numpy as np
import os
from pathlib import Path
import json
import zipfile
from typing import Dict, List, Tuple
from tqdm import tqdm
from numcodecs import Blosc


def group_files_by_episode(pkl_files: List[str]) -> Dict[str, List[str]]:
    """
    Group pickle files by episode directory.
    
    GELLO structure in zip:
        bc_data/gello/1114_120210/timestep1.pkl
        bc_data/gello/1114_120210/timestep2.pkl
        bc_data/gello/1115_093045/timestep1.pkl
        ...
    
    Args:
        pkl_files: List of pickle file paths from zip
        
    Returns:
        Dictionary mapping episode_dir -> list of pkl files in that episode
    """
    episodes = {}
    
    for pkl_file in pkl_files:
        # Get parent directory (episode identifier)
        parts = Path(pkl_file).parts
        if len(parts) >= 2:
            # Use the parent directory as episode identifier
            episode_dir = str(Path(*parts[:-1]))  # Everything except filename
            
            if episode_dir not in episodes:
                episodes[episode_dir] = []
            episodes[episode_dir].append(pkl_file)
    
    # Sort files within each episode by filename (chronological order)
    for episode_dir in episodes:
        episodes[episode_dir].sort()
    
    return episodes


def process_timestep(data: Dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Extract observations and actions from a single GELLO timestep.
    
    Args:
        data: Dictionary from pickle file with GELLO format
        
    Returns:
        Tuple of (joint_pos, gripper_pos, joint_vel, action)
    """
    # Joint positions (7D: 6 arm joints + 1 gripper)
    joint_pos = np.asarray(data['joint_positions'], dtype=np.float32)
    
    # Gripper position (separate from joint_positions for clarity)
    gripper_pos = np.asarray([data['gripper_position']], dtype=np.float32)
    
    # Joint velocities (7D)
    joint_vel = np.asarray(data['joint_velocities'], dtype=np.float32)
    
    # Action (control commands, 7D)
    action = np.asarray(data['control'], dtype=np.float32)
    
    return joint_pos, gripper_pos, joint_vel, action


def load_episode_from_zip(zip_ref, pkl_files: List[str], 
                          include_velocity: bool = False) -> Dict[str, np.ndarray]:
    """
    Load a complete episode from a list of pickle files in the zip.
    
    Args:
        zip_ref: ZipFile object
        pkl_files: List of pickle files for this episode (sorted by time)
        include_velocity: Whether to include joint velocities
        
    Returns:
        Dictionary with episode data
    """
    joint_positions = []
    gripper_positions = []
    joint_velocities = []
    actions = []
    
    for pkl_file in pkl_files:
        try:
            with zip_ref.open(pkl_file, 'r') as f:
                data = pickle.load(f)
            
            # Verify GELLO format
            required_keys = {'joint_positions', 'control', 'joint_velocities', 
                           'gripper_position'}
            if not isinstance(data, dict) or not required_keys.issubset(data.keys()):
                continue
            
            # Extract data
            joint_pos, gripper_pos, joint_vel, action = process_timestep(data)
            
            joint_positions.append(joint_pos)
            gripper_positions.append(gripper_pos)
            joint_velocities.append(joint_vel)
            actions.append(action)
            
        except Exception as e:
            print(f"Warning: Failed to load {pkl_file}: {e}")
            continue
    
    if len(actions) == 0:
        return None
    
    # Convert to arrays
    episode = {
        'qpos': np.array(joint_positions, dtype=np.float32),      # (T, 7)
        'gripper_pos': np.array(gripper_positions, dtype=np.float32),  # (T, 1)
        'actions': np.array(actions, dtype=np.float32),           # (T, 7)
    }
    
    if include_velocity:
        episode['qvel'] = np.array(joint_velocities, dtype=np.float32)  # (T, 7)
    
    return episode


def convert_gello_zip_to_zarr(zip_path: str, 
                               output_zarr_path: str,
                               include_velocity: bool = False,
                               subpath: str = None) -> Path:
    """
    Convert GELLO data from ZIP file to Diffusion Policy Zarr format.
    
    Creates the exact structure expected by Diffusion Policy:
        data/
        ├── obs/
        │   ├── joint_pos       # (T, 7) joint positions
        │   ├── gripper_pos     # (T, 1) gripper positions
        │   └── joint_vel       # (T, 7) velocities (optional)
        ├── action              # (T, 7) actions
        meta/
        └── episode_ends        # (N,) episode boundaries
    
    Args:
        zip_path: Path to zipfile containing GELLO data
        output_zarr_path: Output path for Zarr dataset
        include_velocity: Whether to include joint velocities
        subpath: Optional subdirectory filter within zip
        
    Returns:
        Path to created Zarr file
    """
    zip_path = Path(zip_path)
    output_path = Path(output_zarr_path)
    
    if not zip_path.exists():
        raise ValueError(f"Zipfile not found: {zip_path}")
    
    print(f"Converting GELLO data from: {zip_path}")
    print(f"Output: {output_path}")
    print("=" * 60)
    
    # Load data from zip
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        # Get all pkl files
        all_files = zip_ref.namelist()
        pkl_files = [f for f in all_files if f.endswith('.pkl')]
        
        # Filter by subpath if specified
        if subpath:
            pkl_files = [f for f in pkl_files if subpath in f]
        
        print(f"Found {len(pkl_files)} pickle files in zip")
        
        # Group files by episode
        episodes_dict = group_files_by_episode(pkl_files)
        print(f"Identified {len(episodes_dict)} episodes")
        
        # Load each episode
        episodes = []
        for episode_dir, episode_files in tqdm(episodes_dict.items(), 
                                                desc="Loading episodes"):
            episode = load_episode_from_zip(zip_ref, episode_files, include_velocity)
            if episode is not None:
                episodes.append(episode)
                print(f"  Loaded episode from {Path(episode_dir).name}: "
                      f"{len(episode['qpos'])} timesteps")
    
    if len(episodes) == 0:
        raise ValueError("No valid episodes were loaded from zipfile!")
    
    print(f"\nSuccessfully loaded {len(episodes)} episodes")
    
    # Concatenate all episodes
    all_qpos = []
    all_gripper_pos = []
    all_qvel = [] if include_velocity else None
    all_actions = []
    episode_ends = []
    
    cumulative_len = 0
    for ep in episodes:
        all_qpos.append(ep['qpos'])
        all_gripper_pos.append(ep['gripper_pos'])
        all_actions.append(ep['actions'])
        
        if include_velocity and 'qvel' in ep:
            all_qvel.append(ep['qvel'])
        
        cumulative_len += len(ep['qpos'])
        episode_ends.append(cumulative_len)
    
    # Concatenate arrays
    all_qpos = np.concatenate(all_qpos, axis=0).astype(np.float32)
    all_gripper_pos = np.concatenate(all_gripper_pos, axis=0).astype(np.float32)
    all_actions = np.concatenate(all_actions, axis=0).astype(np.float32)
    episode_ends = np.array(episode_ends, dtype=np.int64)
    
    if include_velocity and all_qvel:
        all_qvel = np.concatenate(all_qvel, axis=0).astype(np.float32)
    
    # Print statistics
    print("\nDataset statistics:")
    print(f"  Total timesteps: {len(all_qpos)}")
    print(f"  Number of episodes: {len(episode_ends)}")
    print(f"  Joint positions shape: {all_qpos.shape}")
    print(f"  Gripper positions shape: {all_gripper_pos.shape}")
    print(f"  Actions shape: {all_actions.shape}")
    if include_velocity and all_qvel is not None:
        print(f"  Joint velocities shape: {all_qvel.shape}")
    
    # Create Zarr dataset
    print("\nCreating Zarr file structure...")
    
    # Remove existing file if it exists
    if output_path.exists():
        import shutil
        shutil.rmtree(output_path)
    
    # Create directory for zarr
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Create root group using zarr.open (works with all zarr versions)
    #zarr_root = zarr.open(str(output_path), mode='w')
    # Create root group - FIXED for Zarr v2 compatibility
    # Create root group
    zarr_root = zarr.open(str(output_path), mode='w')
    
    # Create data group
    data_group = zarr_root.create_group('data')
    
    # Create observation group
    obs_group = data_group.create_group('obs')

    # Simpler version without compression
    obs_group.create_array(
        name="joint_pos",
        data=all_qpos.astype(np.float32),
        chunks=(100, all_qpos.shape[1])
    )

    obs_group.create_array(
        name="gripper_pos",
        data=all_gripper_pos.astype(np.float32),
        chunks=(100, all_gripper_pos.shape[1])
    )

    # ... and similarly for others

    # Add joint velocities if included
    if include_velocity and all_qvel is not None:
        obs_group.create_array(
            name="joint_vel",
            data=all_qvel.astype(np.float32),
            chunks=(100, all_qvel.shape[1])
        )

    # Add actions
    data_group.create_array(
        name="action",
        data=all_actions.astype(np.float32),
        chunks=(100, all_actions.shape[1])
    )

    # Create meta group
    meta_group = zarr_root.create_group("meta")
    meta_group.create_array(
        name="episode_ends",
        data=episode_ends.astype(np.int64)
    )

    
    print(f"✓ Zarr file created successfully at {output_path}")
    
    # Save metadata as JSON for reference
    metadata = {
        'num_episodes': len(episode_ends),
        'total_timesteps': int(cumulative_len),
        'observation_dim': int(all_qpos.shape[1]),
        'gripper_dim': int(all_gripper_pos.shape[1]),
        'action_dim': int(all_actions.shape[1]),
        'include_velocity': include_velocity,
        'source_zip': str(zip_path)
    }
    
    metadata_path = output_path.parent / f"{output_path.stem}_metadata.json"
    with open(metadata_path, 'w') as f:
        json.dump(metadata, indent=2, fp=f)
    
    print(f"✓ Metadata saved to {metadata_path}")
    
    return output_path


def verify_zarr_dataset(zarr_path: str) -> bool:
    """
    Verify the Zarr dataset matches Diffusion Policy expectations.
    
    Args:
        zarr_path: Path to Zarr file
        
    Returns:
        True if verification passes
    """
    print(f"\nVerifying Zarr dataset: {zarr_path}")
    print("=" * 60)
    
    try:
        #zarr_root = zarr.open(str(zarr_path), mode='r')
        zarr_root = zarr.open_group(str(zarr_path), mode='r')
        
        # Check required structure
        required_paths = {
            'data': "Missing 'data' group",
            'meta': "Missing 'meta' group",
            'data/obs': "Missing 'obs' group",
            'data/action': "Missing 'action' dataset",
            'data/obs/joint_pos': "Missing 'joint_pos' dataset",
            'data/obs/gripper_pos': "Missing 'gripper_pos' dataset",
            'meta/episode_ends': "Missing 'episode_ends' dataset"
        }
        
        all_good = True
        for path, error_msg in required_paths.items():
            if path not in zarr_root:
                print(f"❌ {error_msg}")
                all_good = False
            else:
                try:
                    data = zarr_root[path]
                    if hasattr(data, 'shape'):
                        print(f"✓ {path}: shape={data.shape}, dtype={data.dtype}")
                    else:
                        print(f"✓ {path}: group exists")
                except Exception as e:
                    print(f"❌ Error accessing {path}: {e}")
                    all_good = False
        
        if all_good:
            # Verify data consistency
            qpos = zarr_root['data']['obs']['joint_pos']
            gripper = zarr_root['data']['obs']['gripper_pos']
            actions = zarr_root['data']['action']
            episode_ends = zarr_root['meta']['episode_ends']
            
            if qpos.shape[0] != actions.shape[0]:
                print(f"❌ Observation and action lengths don't match: "
                      f"{qpos.shape[0]} vs {actions.shape[0]}")
                all_good = False
            
            if gripper.shape[0] != actions.shape[0]:
                print(f"❌ Gripper and action lengths don't match: "
                      f"{gripper.shape[0]} vs {actions.shape[0]}")
                all_good = False
            
            # Verify first episode
            ep0_end = int(episode_ends[0])
            print(f"\nFirst episode check:")
            print(f"  Length: {ep0_end} timesteps")
            print(f"  Joint positions sample: {qpos[0]}")
            print(f"  Gripper position sample: {gripper[0]}")
            print(f"  Action sample: {actions[0]}")
        
        if all_good:
            print(f"\n✓ All checks passed! Dataset is ready for Diffusion Policy training.")
        else:
            print(f"\n❌ Dataset verification failed.")
        
        return all_good
        
    except Exception as e:
        print(f"❌ Verification failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Convert GELLO data from ZIP to Diffusion Policy Zarr format"
    )
    parser.add_argument(
        '--zip-path',
        type=str,
        required=True,
        help='Path to BC data zipfile'
    )
    parser.add_argument(
        '--output',
        type=str,
        required=True,
        help='Output Zarr path (e.g., ur5_demos.zarr)'
    )
    parser.add_argument(
        '--subpath',
        type=str,
        default=None,
        help='Subdirectory within zip containing data (e.g., "bc_data/gello")'
    )
    parser.add_argument(
        '--include-velocity',
        action='store_true',
        help='Include joint velocities in observations'
    )
    parser.add_argument(
        '--verify-only',
        action='store_true',
        help='Only verify existing dataset'
    )
    
    args = parser.parse_args()
    
    if args.verify_only:
        verify_zarr_dataset(args.output)
    else:
        # Convert
        output_path = convert_gello_zip_to_zarr(
            zip_path=args.zip_path,
            output_zarr_path=args.output,
            include_velocity=args.include_velocity,
            subpath=args.subpath
        )
        
        # Verify
        print("\n" + "=" * 60)
        verify_zarr_dataset(output_path)
        
        print("\n" + "=" * 60)
        print("CONVERSION COMPLETE!")
        print("=" * 60)
        print(f"\nYou can now use this dataset for training:")
        print(f"  python train.py \\")
        print(f"    --config-name=train_diffusion_unet_real_image_workspace \\")
        print(f"    task=ur5_gello \\")
        print(f"    task.dataset_path={output_path}")
