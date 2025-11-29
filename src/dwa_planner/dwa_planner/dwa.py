# dwa_planner/dwa.py
import numpy as np

class DWAConfig:
    def __init__(self):
        # kinematic limits
        self.max_speed = 0.26       # m/s (turtlebot3 waffle default)
        self.min_speed = -0.26
        self.max_yaw_rate = 1.82    # rad/s
        self.max_accel = 0.5        # m/s^2
        self.max_dyaw_rate = 2.5    # rad/s^2

        # sampling resolution
        self.v_reso = 0.02
        self.yaw_rate_reso = 0.1

        # prediction/time
        self.dt = 0.1
        self.predict_time = 2.0

        # robot geometry
        self.robot_radius = 0.18  # meters (used for collision checks)

        # cost weights
        self.weight_goal = 1.0
        self.weight_speed = 0.1
        self.weight_obstacle = 1.5

def motion(state, control, dt):
    # state: [x, y, yaw, v, omega]
    x = float(state[0])
    y = float(state[1])
    yaw = float(state[2])
    v = control[0]
    w = control[1]

    x += v * np.cos(yaw) * dt
    y += v * np.sin(yaw) * dt
    yaw += w * dt
    return np.array([x, y, yaw, v, w])

def calc_dynamic_window(state, cfg: DWAConfig):
    # Vs: limits from robot spec
    Vs = [cfg.min_speed, cfg.max_speed, -cfg.max_yaw_rate, cfg.max_yaw_rate]

    # Vd: limits based on current state and acceleration
    v = state[3]
    w = state[4]
    Vd = [
        v - cfg.max_accel * cfg.dt,
        v + cfg.max_accel * cfg.dt,
        w - cfg.max_dyaw_rate * cfg.dt,
        w + cfg.max_dyaw_rate * cfg.dt
    ]

    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw

def calc_trajectory(state, v, y, cfg: DWAConfig):
    traj = np.array(state)
    trajectory = [traj.copy()]
    t = 0.0
    s = traj.copy()
    while t < cfg.predict_time:
        s = motion(s, [v, y], cfg.dt)
        trajectory.append(s.copy())
        t += cfg.dt
    return np.array(trajectory)  # shape (N,5)

def calc_goal_cost(traj, goal):
    # Euclidean distance from final point to goal
    dx = goal[0] - traj[-1, 0]
    dy = goal[1] - traj[-1, 1]
    return np.hypot(dx, dy)

def calc_obstacle_cost(traj, obstacles, cfg: DWAConfig):
    # obstacles: Nx2 array in same frame as traj positions
    if obstacles is None or obstacles.size == 0:
        return 0.0

    # compute distance from each trajectory point to each obstacle
    px = traj[:, 0][:, None]  # (T,1)
    py = traj[:, 1][:, None]
    ox = obstacles[None, :, 0]  # (1,N)
    oy = obstacles[None, :, 1]
    dists = np.hypot(px - ox, py - oy)  # (T,N)
    min_dist = np.min(dists)

    # collision -> very high cost
    if min_dist <= cfg.robot_radius:
        return float('inf')

    # inverse distance (closer -> larger cost)
    return 1.0 / min_dist

def dwa_control(state, cfg: DWAConfig, goal, obstacles):
    """
    state: [x,y,yaw,v,omega]
    goal: [gx, gy]
    obstacles: Nx2 array
    returns: best_u (v, omega), best_traj (array)
    """
    dw = calc_dynamic_window(state, cfg)

    best_u = (0.0, 0.0)
    best_traj = np.array([state])
    min_cost = float('inf')

    # iterate over velocities with the given resolution
    v_range = np.arange(dw[0], dw[1] + 1e-6, cfg.v_reso)
    w_range = np.arange(dw[2], dw[3] + 1e-6, cfg.yaw_rate_reso)

    for v in v_range:
        for w in w_range:
            traj = calc_trajectory(state, v, w, cfg)

            # costs
            goal_cost = cfg.weight_goal * calc_goal_cost(traj, goal)
            obs_cost = cfg.weight_obstacle * calc_obstacle_cost(traj, obstacles, cfg)
            speed_cost = cfg.weight_speed * (cfg.max_speed - v)

            # if obstacle cost is infinite, skip (collision)
            if obs_cost == float('inf'):
                continue

            total = goal_cost + obs_cost + speed_cost

            if total < min_cost:
                min_cost = total
                best_u = (v, w)
                best_traj = traj

    return best_u, best_traj
