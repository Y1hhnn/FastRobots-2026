+++
title = "Lab 10: Grid Localization using Bayes Filter"
date = "2026-04-22"
+++

This lab focuses on representing the robot’s belief over a discretized 3D state space (x, y, $\theta$). I implemented a grid localization for a sample trajectory in simulation, compute the most probable robot state after each Bayes filter iteration, compare the estimated pose with the ground truth, and evaluate how the localization algorithm performs. 

# Alogrithm

The localization algorithm is a discrete Bayes filter operating on a grid of cells covering the $(x, y, \theta)$ state space. Each cell stores the belief $bel(x_t) = p(x_t \mid z_{1:t}, u_{1:t})$ — the posterior probability that the robot is in that cell, conditioned on all past sensor readings and controls. Under the Markov assumption (Lec. 17), the filter alternates two steps every time the robot moves and senses:

$$\overline{bel}(x_t) = \sum_{x_{t-1}} p(x_t \mid u_t, x_{t-1})\, bel(x_{t-1}) \quad \text{(Prediction)}$$

$$bel(x_t) = \eta \, p(z_t \mid x_t) \, \overline{bel}(x_t) \quad \text{(Update)}$$

The prediction step propagates the previous belief through the odometry motion model $p(x_t \mid u_t, x_{t-1})$, which spreads probability mass and *increases* uncertainty. The update step multiplies by the sensor likelihood $p(z_t \mid x_t)$, which sharpens the belief and *decreases* uncertainty. The constant $\eta$ normalizes the result so the belief sums to one.

## Compute Control

Instead of treating the raw odometry pose difference as the control directly, the odometry motion model (Lec. 18) decomposes the relative motion between two odometry readings into a sequence of three primitive actions: an initial rotation $\delta_{rot1}$, a straight-line translation $\delta_{trans}$, and a final rotation $\delta_{rot2}$.

<div style="display: flex; gap: 30px; align-items: center; flex-wrap: wrap;">

<div style="flex: 1; min-width: 300px;">

$$\delta_{rot1} = \mathrm{atan2}(y_{cur} - y_{prev},\, x_{cur} - x_{prev}) - \theta_{prev}$$

$$\delta_{trans} = \sqrt{(x_{cur} - x_{prev})^2 + (y_{cur} - y_{prev})^2}$$

$$\delta_{rot2} = \theta_{cur} - \theta_{prev} - \delta_{rot1}$$

</div>

<div style="flex: 1; min-width: 300px;">

{{ image(path="content/posts/lab10/control.png", alt="parameters", width=600, class="center" )}}

</div>

</div>

In my implementation I broadcast `cur_pose` and `prev_pose` over numpy arrays so that the same function can compute the control either for a single pair of poses or for an entire grid of candidate poses at once — this turns the prediction step from a triple `for` loop into a single vectorized call. When $\delta_{trans}$ is below $10^{-4}$ m I treat the motion as a pure rotation and fold the entire heading change into $\delta_{rot2}$, otherwise $\delta_{rot1}$ becomes ill-defined because `atan2` of near-zero arguments is numerically unstable. All angles are wrapped to $[-180^\circ, 180^\circ]$ with `norm_angle` so that, e.g., a turn from $+170^\circ$ to $-170^\circ$ is correctly interpreted as $+20^\circ$ rather than $-340^\circ$.

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

## Odometry Motion Model

The motion model returns the transition probability $p(x_t \mid u_t, x_{t-1})$ — i.e. how likely it is that the robot landed at the candidate current pose $x_t$ given that it was at $x_{t-1}$ and reported odometry control $u_t$.

Following the algorithm from Lec. 18, I run `compute_control(cur_pose, prev_pose)` once more — but this time on the candidate poses $x_{t-1}, x_t$ — to get the expected control that would have moved the robot exactly between those two cells. I then subtract this expected control from the actually-reported $u_t$ to obtain three errors $e_{rot1}, e_{trans}, e_{rot2}$. Each error is evaluated under a zero-mean Gaussian, and the three sub-probabilities are multiplied under the assumption that the rotation, translation, and rotation errors are independent:

$$p(x_t \mid u_t, x_{t-1}) = \mathcal{N}(e_{rot1};\, 0, \sigma_{rot}^2) \cdot \mathcal{N}(e_{trans};\, 0, \sigma_{trans}^2) \cdot \mathcal{N}(e_{rot2};\, 0, \sigma_{rot}^2)$$

The two noise parameters $\sigma_{rot}$ and $\sigma_{trans}$ (loaded from `world.yaml`) effectively control how noisy the motion model is. Large values produce a wide cloud of plausible next poses, while small values pin the prediction near the reported odometry.

{{ image(path="content/posts/lab10/sensor.png", alt="sensor cloud", class="center" )}}


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

## Prediction Step

The prediction step implements line 3 of the Bayes filter algorithm: it sweeps over all pairs $(x_{t-1}, x_t)$ in the grid, weights the transition probability $p(x_t \mid u_t, x_{t-1})$ by the previous belief $bel(x_{t-1})$, and accumulates the result into $\overline{bel}(x_t)$. The state space is $12 \times 9 \times 18 = 1944$ cells, so a naive double loop would require $1944^2 \approx 3.8$ million transition evaluations per time step.

I made two optimizations:

1. **Skip cells with negligible prior.** Only previous cells with $bel(x_{t-1}) > 10^{-4}$ are iterated over (`valid_prev_indices = np.argwhere(loc.bel > 0.0001)`). After a few sensor updates the belief becomes very peaked, so usually only a handful of cells actually contribute.
2. **Vectorize over current poses.** Rather than looping over each candidate $x_t$, I build the full $1944 \times 3$ array of all current poses once, then call `odom_motion_model(all_cur_poses, prev_pose, u)` to get all 1944 transition probabilities in one numpy call. Each previous cell thus contributes a full slab of the belief grid in a single line.

Finally I normalize $\overline{bel}$ so it sums to 1. This is not strictly required by the math (the update step will renormalize anyway), but it prevents arithmetic underflow when many small Gaussian probabilities are multiplied together.

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

## Sensor Model

The sensor model returns the measurement likelihood $p(z_t \mid x_t)$ — i.e. how likely the recorded 18-ray scan would be if the robot were in candidate pose $x_t$. From Lec. 19, individual range rays are treated as conditionally independent given $x_t$ and the map:

$$p(z_t \mid x_t) = \prod_{k=1}^{K=18} p(z_t^k \mid x_t, m)$$

For each ray I use a simple Gaussian centered on the ray-cast expected reading $z_t^{k*}$, which is a stripped-down version of the likelihood-field model from Lec. 19 (just the $\mathcal{N}(\hat{z}, \sigma_{hit}^2)$ component, with the $p_{max}$ and $p_{rand}$ terms folded into a single $\sigma_{sensor}$):

$$p(z_t^k \mid x_t, m) = \mathcal{N}(z_t^k - z_t^{k*};\, 0, \sigma_{sensor}^2)$$

In the code, `sensor_model(obs)` returns the per-ray likelihood vector of length 18 rather than the product, so the caller (the update step) can decide how to combine them. The expected scan $z_t^{k*}$ for every pose has been pre-computed by the mapper and stored in `loc.mapper.obs_views`, so the update step does not need to do ray casting at runtime — it just looks up the table.

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

## Update Step

The update step implements line 4 of the Bayes filter. It multiplies the predicted belief $\overline{bel}(x_t)$ pointwise by the sensor likelihood and renormalizes:

$$bel(x_t) = \eta \, p(z_t \mid x_t) \, \overline{bel}(x_t), \quad \eta = \left(\sum_{x_t} p(z_t \mid x_t)\,\overline{bel}(x_t)\right)^{-1}$$

The implementation is fully vectorized: `loc.obs_range_data` is the $(18,)$ measurement vector and `loc.mapper.obs_views` is the pre-computed $(12, 9, 18, 18)$ expected-scan tensor, so the per-ray error `actual_obs - expected_obs` is computed for every grid cell at once via broadcasting. Taking `np.prod(..., axis=-1)` then collapses the 18 rays into a single likelihood per cell, which is exactly the product from the sensor-model equation above.

Multiplying 18 Gaussian values together can produce extremely small numbers. If $\sum bel$ underflows to zero after the update, I fall back to $bel = \overline{bel}$ rather than dividing by zero, so the filter degrades to "trust the motion model" instead of crashing.

<!-- The 18 rays are not truly independent in practice (a small pose error shifts all rays together), so the product over-confidently sharpens the posterior. The mismatch is one of the Markov-assumption violations from Lec. 18 — but as the lecture notes, Bayes filters are surprisingly robust to it. -->

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

The simulator is split into two processes: a *robot display* that draws the robot's body and the simulated trajectory inside the virtual world, and a *plotter* that overlays the Bayes-filter belief and the ground-truth pose on the same map for direct comparison. The two processes communicate through the `Commander`, so each iteration of the filter pushes both the current belief and the current ground-truth pose to the plotter.

Each run executes a pre-defined trajectory consisting of a sequence of motions. At every step the robot first does an initial rotation, then a straight translation, then a final rotation (i.e. the three odometry primitives $\delta_{rot1}, \delta_{trans}, \delta_{rot2}$ from the motion model), and finally spins in place to collect the 18-ray observation that drives the update step. After each iteration three trajectories are plotted: ground truth (green), pure odometry (red), and the Bayes-filter belief over the most-probable cell (blue).

{{ image(path="content/posts/lab10/sim.png", alt="sim", width=1200, class="center" )}}

In the result above, the green and blue lines track each other closely throughout the run, while the red odometry-only line drifts off the map after just a few steps because odometry errors accumulate without any sensor correction. This is exactly the qualitative behavior predicted by the Bayes filter theory: the prediction step alone increases uncertainty unboundedly, but the update step pulls the belief back to whichever cells are consistent with the latest ToF scan.

However, a few limitations are visible in the trace:

- **Discretization.** The belief lives on a grid of cell centers, so the blue line is forced onto a fixed lattice (cell sizes $0.3048$ m in $x,y$ and $20^\circ$ in $\theta$). It cannot follow the green trajectory exactly — even with a perfect sensor it can only align with the closest cell center.
- **Feature-poor regions.** The belief lags noticeably along the rightmost hallway. There, most rays hit the same two long parallel walls, so several cells along the hallway produce nearly identical expected scans. The likelihood is then flat along the hallway axis (the $y$ direction), and the filter cannot disambiguate which cell the robot is actually in until it reaches a corner.
- **Bias from limited ray angles.** Only 18 rays cover the full 360°, so small obstacles or thin gaps in the map can be missed entirely between consecutive rays.

Overall the blue trajectory confirms that my prediction step, sensor model, and update step are working correctly: the filter recovers the true pose after each measurement and never diverges, even though the prior belief was initialized uniform.


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
