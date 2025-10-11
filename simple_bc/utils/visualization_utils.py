import math
import numpy as np
import mediapy as mp

def make_grid_video_from_numpy(seqs, ncols, out_path, fps=30):
    """
    seqs: list of videos, each (T,H,W,3) uint8 or float [0,1]
    ncols: grid columns
    out_path: mp4 path
    """
    if not seqs:
        raise ValueError("No sequences provided")
    T = min(s.shape[0] for s in seqs)
    seqs = [s[:T] for s in seqs]
    seqs = [s if (s.ndim == 4 and s.shape[-1] == 3) else np.tile(s[..., None], [1,1,1,3]) for s in seqs]
    seqs = [np.clip(s, 0, 1) if s.dtype.kind == 'f' else s for s in seqs]

    n = len(seqs)
    ncols = max(1, int(ncols))
    nrows = math.ceil(n / ncols)
    H, W = seqs[0].shape[1], seqs[0].shape[2]

    pad = nrows * ncols - n
    if pad > 0:
        black = np.zeros((T, H, W, 3), dtype=seqs[0].dtype)
        seqs += [black] * pad

    frames = []
    for t in range(T):
        rows = []
        for r in range(nrows):
            row = np.concatenate([seqs[r * ncols + c][t] for c in range(ncols)], axis=1)
            rows.append(row)
        grid = np.concatenate(rows, axis=0)
        frames.append(grid)
    video = np.stack(frames, axis=0)
    mp.write_video(out_path, video, fps=fps)
