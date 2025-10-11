from typing import Any, Dict, List, Union
import numpy as np

def _stack_list(items: List[Any]):
    """Recursively stack a list of dict/arrays along axis=0."""
    first = items[0]
    if isinstance(first, dict):
        return {k: _stack_list([x[k] for x in items]) for k in first.keys()}
    return np.stack(items, axis=0)

class DictArray(dict):
    @staticmethod
    def stack(seq: List[Dict[str, Any]]) -> "DictArray":
        return DictArray(_stack_list(seq))

class GDict(dict):
    """Minimal stand-in; supports HDF5 save."""
    @staticmethod
    def to_hdf5(obj: Union[dict, np.ndarray], path: str):
        try:
            import h5py
        except Exception:
            import pickle
            with open(path, "wb") as f:
                pickle.dump(obj, f)
            print(f"[gdict] h5py not found; pickled to {path}")
            return

        def _write(h, key, val):
            if isinstance(val, dict):
                grp = h.create_group(key)
                for k, v in val.items():
                    _write(grp, k, v)
            else:
                arr = np.asarray(val)
                h.create_dataset(key, data=arr)

        with h5py.File(path, "w") as h:
            for k, v in obj.items():
                _write(h, k, v)
