import numpy as np
import torch

def _to_numpy(x):
    # torch Tensor → numpy (CPU)
    if torch.is_tensor(x):
        return x.detach().cpu().numpy()
    # already numpy / list / tuple → numpy
    return np.asarray(x)

def rollout_optimized(start, path, obstacle_points, solver, model, device, epoch, folder=""):
    p = _to_numpy(start).copy()
    traj = [p.copy()]
    nominal = [_to_numpy(start).copy()]
    for wp in path:
        wp_np = _to_numpy(wp)
        nominal.append(wp_np.copy())
        p, debug = solver(p, wp_np, obstacle_points, model, device, epoch, folder)
        p = _to_numpy(p).copy()
        traj.append(p.copy())

    return traj