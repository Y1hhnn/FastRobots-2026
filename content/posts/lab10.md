+++
title = "Lab 10: Grid Localization using Bayes Filter"
date = "2026-04-22"
+++

# Alogrithm

## Compute Control

```python
def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """

    # Convert to numpy arrays 
    cur_pose = np.array(cur_pose)
    prev_pose = np.array(prev_pose)
    x_prev, y_prev, yaw_prev = prev_pose[..., 0], prev_pose[..., 1], prev_pose[..., 2]
    x_cur,  y_cur,  yaw_cur  = cur_pose[..., 0],  cur_pose[..., 1],  cur_pose[..., 2]
    x_cur, x_prev = np.broadcast_arrays(x_cur, x_prev)
    y_cur, y_prev = np.broadcast_arrays(y_cur, y_prev)
    yaw_cur, yaw_prev = np.broadcast_arrays(yaw_cur, yaw_prev)

    # 1. Translation
    dx = x_cur - x_prev
    dy = y_cur - y_prev
    delta_trans = np.hypot(dx, dy)

    # If translation is  small, treat it as a pure rotation
    mask = delta_trans >= 1e-4

    # 2. Rotation 1
    delta_rot_1 = np.zeros_like(delta_trans)
    delta_rot_1[mask] = np.degrees(np.arctan2(dy[mask], dx[mask])) - yaw_prev[mask]
    delta_rot_1 = norm_angle(delta_rot_1)
    
    # 3. Rotation 2
    delta_rot_2 = np.zeros_like(delta_trans)
    delta_rot_2[mask] = yaw_cur[mask] - yaw_prev[mask] - delta_rot_1[mask]
    delta_rot_2[~mask] = yaw_cur[~mask] - yaw_prev[~mask]
    delta_rot_2 = norm_angle(delta_rot_2)

    return np.squeeze(delta_rot_1), np.squeeze(delta_trans), np.squeeze(delta_rot_2)
```

# Odometry Motion Model

```python
def odom_motion_model(cur_pose, prev_pose, u):
    """ Odometry Motion Model

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose
        (rot1, trans, rot2) (float, float, float): A tuple with control data in the format 
                                                   format (rot1, trans, rot2) with units (degrees, meters, degrees)


    Returns:
        prob [float]: Probability p(x'|x, u)
    """
    rot1_u, trans_u, rot2_u = u
    rot1_exp, trans_exp, rot2_exp = compute_control(cur_pose, prev_pose)

    # Compute Errors
    err_rot1 = norm_angle(rot1_u - rot1_exp)
    err_trans = trans_u - trans_exp
    err_rot2 = norm_angle(rot2_u - rot2_exp)

    # Compute Gaussian Probabilities
    p1 = loc.gaussian(err_rot1, 0.0, loc.odom_rot_sigma)
    p2 = loc.gaussian(err_trans, 0.0, loc.odom_trans_sigma)
    p3 = loc.gaussian(err_rot2, 0.0, loc.odom_rot_sigma)

    # Combine probabilities (assuming independence)
    prob = p1 * p2 * p3
    return prob
```

# Prediction Step

```python
def prediction_step(cur_odom, prev_odom):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current Pose
        prev_odom ([Pose]): Previous Pose
    """
    u = compute_control(cur_odom, prev_odom)
    loc.bel_bar = np.zeros_like(loc.bel)

    all_x = loc.mapper.x_values.flatten()
    all_y = loc.mapper.y_values.flatten()
    all_a = loc.mapper.a_values.flatten()
    all_cur_poses = np.column_stack((all_x, all_y, all_a))


    # Skip cells with negligible belief
    valid_prev_indices = np.argwhere(loc.bel > 0.0001)

    # Iterate all possible prev poses with non-negligible belief
    for idx in valid_prev_indices:
        # Get previous pose and its belief
        cx, cy, ca = idx
        prev_prob = loc.bel[cx, cy, ca]
        x_prev, y_prev, a_prev = loc.mapper.from_map(cx, cy, ca)
        prev_pose = np.array([x_prev, y_prev, a_prev])

        # Compute transition probabilities for all current poses
        prob_transition = odom_motion_model(all_cur_poses, prev_pose, u)
        loc.bel_bar += prob_transition.reshape(loc.bel_bar.shape) * prev_prob
    
    # Normalize to prevent arithmetic underflow 
    bel_bar_sum = np.sum(loc.bel_bar)
    if bel_bar_sum > 0:
        loc.bel_bar /= bel_bar_sum  
```

# Sensor Model

```python
def sensor_model(obs):
    """ This is the equivalent of p(z|x).


    Args:
        obs ([ndarray]): A 1D array consisting of the true observations for a specific robot pose in the map 

    Returns:
        [ndarray]: Returns a 1D array of size 18 (=loc.OBS_PER_CELL) with the likelihoods of each individual sensor measurement
    """
    error = loc.obs_range_data - obs
    prob_array = loc.gaussian(error, 0.0, loc.sensor_sigma)
    return prob_array
```

# Update Step

```python
def update_step():
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    """
    # Compute the expected observations & their probabilities 
    actual_obs = loc.obs_range_data.flatten().reshape(1, 1, 1, -1)
    expected_obs = loc.mapper.obs_views
    likelihoods = loc.gaussian(actual_obs - expected_obs, 0.0, loc.sensor_sigma)

    # Compute Naive Likelihood p(z|x) 
    p_z_given_x = np.prod(likelihoods, axis=-1)

    # Bayesian Update: Prior * Naive Likelihood = Posterior
    loc.bel =  p_z_given_x *loc.bel_bar

    # Normalize to prevent arithmetic underflow 
    bel_sum = np.sum(loc.bel)
    if bel_sum > 0:
        loc.bel /= bel_sum
    else:
        loc.bel = loc.bel_bar
```

# Simulation

{{ image(path="content/posts/lab10/sim.png", alt="sim", width=1200, class="center" )}}


[Video Here](https://youtube.com/shorts/BR1tXvoKRNw)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/BR1tXvoKRNw"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>
