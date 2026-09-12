# Mathematical Methodology & Formulations — SIH26037

## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

---

## 1. Kinematic Bicycle Vehicle Dynamics Model

The ego vehicle motion is governed by the 2D Kinematic Bicycle Model referenced at the center of the rear axle:

$$\dot{x} = v \cos(\theta)$$
$$\dot{y} = v \sin(\theta)$$
$$\dot{\theta} = \frac{v}{L} \tan(\delta)$$
$$\dot{v} = a$$

Where:
- $(x, y)$: Cartesian coordinates of rear axle in world frame (meters)
- $\theta$: Vehicle heading angle relative to global X-axis (radians)
- $v$: Longitudinal velocity ($v \in [0, v_{\max}]$, where $v_{\max} = 12\,\text{m/s}$)
- $\delta$: Front wheel steering angle ($\delta \in [-\delta_{\max}, \delta_{\max}]$, where $\delta_{\max} = 35^\circ$)
- $a$: Longitudinal acceleration/braking ($a \in [-a_{\text{decel}}, a_{\text{accel}}]$)
- $L$: Wheelbase length ($L = 2.7\,\text{m}$)

### Numerical Integration (Runge-Kutta 2nd Order / Midpoint Method)
To ensure numerical stability at $\Delta t = 0.05\,\text{s}$:
$$\mathbf{s}_{\text{mid}} = \mathbf{s}_k + \frac{\Delta t}{2} f(\mathbf{s}_k, \mathbf{u}_k)$$
$$\mathbf{s}_{k+1} = \mathbf{s}_k + \Delta t \cdot f(\mathbf{s}_{\text{mid}}, \mathbf{u}_k)$$

---

## 2. Occupancy Grid & Spatial Inflation

The continuous road environment is discretized into a 2D matrix $\mathcal{M}(i, j)$ with spatial resolution $\Delta s = 0.25\,\text{m/cell}$:
$$\mathcal{M}(i, j) \in \{0.0 \,(\text{Free}), 0.5 \,(\text{Unknown}), 1.0 \,(\text{Obstacle})\}$$

For each detected obstacle $k$ with center $(x_k, y_k)$, base radius $r_k$, and safety margin $r_{\text{margin}} = 1.2\,\text{m}$:
$$\text{Cost}(x, y) = \begin{cases}
1.0 & \text{if } \sqrt{(x - x_k)^2 + (y - y_k)^2} \le r_k \\
0.85 & \text{if } r_k < \sqrt{(x - x_k)^2 + (y - y_k)^2} \le (r_k + r_{\text{margin}}) \\
\mathcal{M}_{\text{road}}(x, y) & \text{otherwise}
\end{cases}$$

---

## 3. Adaptive A* & Kinematic Motion Planning

The A* algorithm evaluates nodes using the cost function:
$$f(n) = g(n) + h(n)$$

Where:
- $g(n)$: Accumulated path cost from start node $s$ to node $n$:
  $$g(n) = g(\text{parent}) + \Delta d \cdot \left(1.0 + w_{\text{obs}} \mathcal{M}(n) + w_{\text{steer}} |\Delta \theta| + w_{\text{center}} |y_n - y_{\text{center}}|\right)$$
- $h(n)$: Euclidean distance heuristic to goal $(x_g, y_g)$:
  $$h(n) = \epsilon \cdot \sqrt{(x_n - x_g)^2 + (y_n - y_g)^2}, \quad (\epsilon = 1.1)$$

### Path Smoothing (PCHIP + Gaussian Curvature Regularization)
The raw discrete waypoints are parameterized by arc length $s$ and smoothed:
$$y_{\text{smooth}}(s) = \sum_{k=-W}^W w_k \cdot y(s + k \cdot \Delta s)$$

---

## 4. Collision Checking & Time-To-Collision (TTC)

For each obstacle $k$ with relative position $\mathbf{r}_k = \mathbf{p}_k - \mathbf{p}_{\text{ego}}$ and relative velocity $\mathbf{v}_{\text{rel}} = \mathbf{v}_{\text{ego}} - \mathbf{v}_k$:

### Closing Speed
$$v_{\text{closing}} = \frac{\mathbf{r}_k \cdot \mathbf{v}_{\text{rel}}}{\|\mathbf{r}_k\|}$$

### Time-To-Collision (TTC)
$$\text{TTC} = \begin{cases}
\frac{\|\mathbf{r}_k\| - (r_k + L/2)}{v_{\text{closing}}} & \text{if } v_{\text{closing}} > 0.1\,\text{m/s} \\
\infty & \text{otherwise (diverging/static)}
\end{cases}$$

### 3-Tier Risk Decision Rule
$$\text{RiskLevel} = \begin{cases}
\text{CRITICAL} & \text{if } \|\mathbf{r}_k\| \le 3.5\,\text{m} \lor \text{TTC} \le 1.8\,\text{s} \lor \text{PathBlocked} \\
\text{WARNING}  & \text{if } \|\mathbf{r}_k\| \le 8.0\,\text{m} \lor \text{TTC} \le 3.5\,\text{s} \\
\text{SAFE}     & \text{otherwise}
\end{cases}$$

---

## 5. Pure Pursuit Path Tracking Controller

The geometric curvature $\kappa$ required to intercept the lookahead waypoint $\mathbf{p}_{\text{target}} = (x_r, y_r)$ in vehicle coordinates is:

$$\kappa = \frac{2 y_r}{L_d^2}$$
$$\delta = \text{atan2}(2 L y_r, L_d^2)$$

Where speed-adaptive lookahead distance $L_d$ is:
$$L_d = \max\left(L_{d,\min}, \, \min\left(L_{d,\max}, \, k_{\text{lookahead}} \cdot v\right)\right)$$
$$(L_{d,\min} = 2.5\,\text{m}, \quad L_{d,\max} = 7.0\,\text{m}, \quad k_{\text{lookahead}} = 0.5\,\text{s})$$
