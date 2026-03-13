import os
import numpy as np
import torch
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.stats import norm
from load_njsdf.inference import mu_sigma_grad_nn, BETA
import pinocchio as pin

URDF_PATH = "/manipulator_simulator/src/ur5e_rgbd/urdf/ur5e_rgbd.urdf"
pin_model = pin.buildModelFromUrdf(URDF_PATH)
pin_data = pin_model.createData()

EE_FRAME = pin_model.getFrameId("tool0")

debug_step_counter = 0
last_debug_epoch = -1

def fk_ur5e(q):
    """
    Returns 6D pose [x,y,z,rx,ry,rz] of tool0
    """
    pin.forwardKinematics(pin_model, pin_data, q)
    pin.updateFramePlacements(pin_model, pin_data)

    pose = pin_data.oMf[EE_FRAME]

    x = pose.translation

    R = pose.rotation
    rpy = pin.rpy.matrixToRpy(R)

    return np.concatenate([x, rpy])

def jacobian_ur5e(q):
    """
    Returns 6x6 geometric Jacobian of tool0
    """
    pin.forwardKinematics(pin_model, pin_data, q)
    pin.updateFramePlacements(pin_model, pin_data)

    J = pin.computeFrameJacobian(
        pin_model,
        pin_data,
        q,
        EE_FRAME,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )

    return J

def wrap_angle_diff(a, b):
    """
    Returns wrapped angle difference a - b in [-pi, pi].
    Works elementwise.
    """
    d = a - b
    return (d + np.pi) % (2 * np.pi) - np.pi


def pose_error_6d(x_curr, x_goal):
    """
    Compute 6D pose error x_curr - x_goal.
    Assumes first 3 are translation, last 3 are angles.
    If your orientation is not Euler-like, replace this.
    """
    pos_err = x_curr[:3] - x_goal[:3]
    rot_err = wrap_angle_diff(x_curr[3:], x_goal[3:])
    return np.concatenate([pos_err, rot_err])


def solve_step_ur5e(q0, q_goal, obstacle_points, model, device, epoch, folder="", safety_margin=0.0, verbose=True):
    """
    Solve one 6D local chance-constrained step for UR5e.

    Parameters
    ----------
    q0 : (6,)
        Current waypoint in joint configuration.
    q_goal : (6,)
        Next waypoint in joint configuration
    obstacle_points : array-like
        Obstacle point cloud or active obstacle set.
    model, device :
        Passed into mu_sigma_grad_nn.
    q_min, q_max : (6,)
        Joint limits.
    epoch : int
        For debug plots / naming.
    folder : str
        Plot output folder.
    Q, D : (6,6)
        Weight matrices for dq and slack.
    safety_margin : float
        Optional deterministic clearance buffer.
    dq_limit : float or (6,), optional
        Per-step max absolute joint change.
    verbose : bool

    Returns
    -------
    q_next : (6,)
        Updated joint configuration.
    result : OptimizeResult
        Raw scipy result.
    debug : dict
        Contains mu, sigma, grad, obs_k, J, x_curr, x_goal.
    """
    global debug_step_counter, last_debug_epoch

    if epoch != last_debug_epoch:
        debug_step_counter = 0
        last_debug_epoch = epoch

    step_id = debug_step_counter
    debug_step_counter += 1

    # -----------------------------
    # Convert inputs
    # -----------------------------
    if torch.is_tensor(q0):
        q0 = q0.detach().cpu().numpy()
    if torch.is_tensor(q_goal):
        q_goal = q_goal.detach().cpu().numpy()

    q0 = np.asarray(q0, dtype=np.float64).reshape(6)
    q_goal = np.asarray(q_goal, dtype=np.float64).reshape(6)

    
    Q = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    D = np.diag([100.0, 100.0, 100.0, 20.0, 20.0, 20.0])
    dq_limit = 0.15  # rad per step (~8.6 degrees)
    # -----------------------------
    # Current FK + Jacobian
    # -----------------------------
    # x_curr = np.asarray(fk_fn(q0), dtype=np.float64).reshape(6)
    # J = np.asarray(jacobian_fn(q0), dtype=np.float64).reshape(6, 6)
    # -----------------------------
    # Current FK + Jacobian (Pinocchio)
    # -----------------------------
    x_curr = fk_ur5e(q0)
    x_goal = fk_ur5e(q_goal)
    J = jacobian_ur5e(q0)
    # -----------------------------
    # Stochastic distance model
    # Assumes grad is wrt q (6D)
    # -----------------------------
    mu, sigma, grad, obs_k = mu_sigma_grad_nn(
        q0, obstacle_points, model, device
    )

    mu = np.asarray(mu, dtype=np.float64).reshape(-1)
    sigma = np.asarray(sigma, dtype=np.float64).reshape(-1)
    grad = np.asarray(grad, dtype=np.float64).reshape(-1, 6)

    if verbose:
        print("\n--- 6D Chance-Constrained IK Debug ---")
        print("q0:", q0)
        print("x_curr:", x_curr)
        print("x_goal:", x_goal)
        print("min(mu):", np.min(mu))
        print("min(mu - beta*sigma):", np.min(mu - BETA * sigma))
        print("mean(sigma):", np.mean(sigma))
        print("J condition number:", np.linalg.cond(J))

    # ------------------------------------
    # Optimization variable:
    # z = [dq(6), slack(6)] -> total 12 vars
    # ------------------------------------
    def objective(z):
        dq = z[:6]
        slack = z[6:]
        return dq @ Q @ dq + slack @ D @ slack

    def tracking_constraint(z):
        """
        x_curr + J dq = x_goal + slack
        => x_curr + J dq - x_goal - slack = 0
        """
        dq = z[:6]
        slack = z[6:]
        return x_curr + J @ dq - x_goal - slack

    def chance_constraints(z):
        """
        Linearized chance-safe distance:
            mu_k + grad_k^T dq - beta*sigma_k - safety_margin >= 0
        """
        dq = z[:6]
        return mu + grad @ dq - BETA * sigma - safety_margin
    
    q_min = np.array([-6.28]*6)
    q_max = np.array([ 6.28]*6)
    def joint_lower_constraint(z):
        dq = z[:6]
        return (q0 + dq) - q_min   # >= 0

    def joint_upper_constraint(z):
        dq = z[:6]
        return q_max - (q0 + dq)   # >= 0

    cons = [
        {"type": "eq",   "fun": tracking_constraint},
        {"type": "ineq", "fun": chance_constraints},
        {"type": "ineq", "fun": joint_lower_constraint},
        {"type": "ineq", "fun": joint_upper_constraint},
    ]


    z0 = np.zeros(12, dtype=np.float64)

    res = minimize(
        objective,
        z0,
        method="SLSQP",
        constraints=cons,
        options={"maxiter": 200, "ftol": 1e-9, "disp": verbose},
    )

    if not res.success:
        print("\n[WARN] Optimization did not fully succeed:", res.message)

    dq_opt = res.x[:6]
    slack_opt = res.x[6:]
    q_next = q0 + dq_opt
    # ------------------------------------
    # dq limit debug
    # ------------------------------------
    for i in range(6):
        if abs(dq_opt[i]) > dq_limit:
            print(f"[DEBUG] dq_limit exceeded on joint {i}: {dq_opt[i]:.4f} > {dq_limit}")
    if verbose:
        print("dq_opt:", dq_opt)
        print("slack_opt:", slack_opt)
        print("q_next:", q_next)
        print("min chance residual:", np.min(chance_constraints(res.x)))
        print("tracking residual norm:", np.linalg.norm(tracking_constraint(res.x)))

    debug = {
        "mu": mu,
        "sigma": sigma,
        "grad": grad,
        "obs_k": obs_k,
        "J": J,
        "x_curr": x_curr,
        "x_goal": x_goal,
        "dq_opt": dq_opt,
        "slack_opt": slack_opt,
        "result": res,
    }

    # Optional debugging plot
    # if folder:
    #     plot_chance_debug_jointspace(
    #         q0=q0,
    #         q_next=q_next,
    #         x_curr=x_curr,
    #         x_goal=x_goal,
    #         mu=mu,
    #         sigma=sigma,
    #         grad=grad,
    #         folder=folder,
    #         epoch=epoch,
    #         step_id=step_id,
    #     )

    return q_next#, res, debug


def plot_chance_debug_jointspace(q0, q_next, x_curr, x_goal, mu, sigma, grad, folder, epoch, step_id):
    """
    Simple debugging plots for the 6D optimizer.
    """
    epoch_folder = os.path.join(folder, f"epoch_{epoch:04d}")
    os.makedirs(epoch_folder, exist_ok=True)

    # -------- Plot 1: mu and risk margin --------
    risk = mu - BETA * sigma
    idx = np.arange(len(mu))

    plt.figure(figsize=(8, 4))
    plt.plot(idx, mu, label="mu")
    plt.plot(idx, risk, label="mu - beta*sigma")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("Active constraint index")
    plt.ylabel("Distance / risk margin")
    plt.title("Chance constraint margins")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{epoch_folder}/step_{step_id:05d}_chance_margins.png")
    plt.close()

    # -------- Plot 2: grad norms --------
    grad_norm = np.linalg.norm(grad, axis=1)
    plt.figure(figsize=(8, 4))
    plt.plot(idx, grad_norm)
    plt.xlabel("Active constraint index")
    plt.ylabel("||grad||")
    plt.title("Constraint gradient norms in joint space")
    plt.tight_layout()
    plt.savefig(f"{epoch_folder}/step_{step_id:05d}_grad_norms.png")
    plt.close()

    # -------- Plot 3: joint update --------
    plt.figure(figsize=(8, 4))
    plt.plot(np.arange(6), q0, 'o-', label="q0")
    plt.plot(np.arange(6), q_next, 'o-', label="q_next")
    plt.xlabel("Joint index")
    plt.ylabel("Joint value [rad]")
    plt.title("Joint update")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{epoch_folder}/step_{step_id:05d}_joint_update.png")
    plt.close()