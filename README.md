# Autonomous quadrotor
## Planning
### Bidirectional A*
We use A* Search, a hallmark graph algorithm in robotics, which builds upon Dijkstra's Algorithm by expanding nodes prioritized by the sum of their "cost-to-come" ($g(v): V \rightarrow \mathbb{R}$) and an estimated "cost-to-go" ($h(v): V \rightarrow \mathbb{R}$), $f(v) = g(v) + h(v)$. We take as input a graph $G(V, E)$, a source node $v_s \in V$, and a sink node $v_g \in V$. 

The "cost-to-come" term $g(v)$ defines the cost of following the optimal sub-path from $v_s$ to $v \in V$. The "cost-to-go" term, also known as a heuristic $h(v)$, is an estimate of the cost to follow the optimal path from $v \in V$ to $v_g$. We ensure that A* Search is optimal and complete by using a heuristic $h(v)$ that is both admissible and consistent. 

We consider $h(v)$ admissible if $\forall v \in V$, $h(v) \leq c^*(v, v_g)$, where $c^*(v)$ is the true cost of the optimal path from $v$ to $v_g$. We consider $h(v)$ consistent if $\forall u, v \in V$ such that $e(u, v) \in E$, $h(u) \leq c(u, v) + h(v)$. 

We choose $h$ as the Euclidean ($L_2$) distance. Other candidates include the Manhattan ($L_1$) distance and Diagonal or Chebyshev ($L_{\infty}$) distance. 

By using A* Search with these heuristics, we will return a path $p^* = (v_s, ..., v_g)$ which minimizes the Euclidean distance traversing $e \in E$ from $v_s$ to $v_g$.


We can apply this algorithm from the source node to the sink node and from the sink node to the source node at the same time. When the two paths meet, the algorithm converges to the shortest path. With a good heuristic choice, this will improve convergence time.

### Waypoint pruning
Assuming we have performed bidirectional A* and obtained a list of waypoints, we can use the Ramer–Douglas–Peucker (RDP) algorithm to reduce the number of waypoints before feeding them into a trajectory generator. The RDP algorithm simplifies a path by removing points that do not significantly contribute to the overall shape, thus reducing complexity while maintaining the path's essential form.

The RDP algorithm works as follows:

1. **Initial Step**: Given a set of waypoints $P = \{p_1, p_2, \ldots, p_n\}$, the algorithm starts with the first point $p_1$ and the last point $p_n$ as the initial endpoints of the simplified path.

2. **Finding the Point of Maximum Distance**: For each point $p_i$ in the set $P$, the perpendicular distance $d_i$ from the point to the line segment $p_1p_n$ is calculated. This distance is given by:
   $$
   d_i = \frac{ \left| (p_n - p_1) \times (p_1 - p_i) \right| }{ \| p_n - p_1 \| }
   $$
   where $\times$ denotes the cross product and $\| \cdot \|$ denotes the Euclidean norm.

3. **Recursive Subdivision**: Find the point $p_k$ with the maximum distance $d_k$ from the line segment $p_1p_n$. If $d_k$ is greater than a predefined threshold $\epsilon$, the algorithm keeps $p_k$ and recursively applies the same process to the segments $p_1p_k$ and $p_kp_n$. If $d_k \leq \epsilon$, all points between $p_1$ and $p_n$ are discarded.

4. **Constructing the Simplified Path**: The process is repeated recursively for each sub-segment until all points are processed. The remaining points form the simplified path.

### Minimum-Snap Trajectory Generation
To generate minimum-snap trajectories, we use a $7\text{th}$ order polynomial to represent the position (in one dimension) of the quadrotor:

$$
s_{i}(t)=a_{s, i}t^7+b_{s, i}t^6+c_{s, i}t^5+d_{s, i}t^4+e_{s, i}t^3+f_{s, i}t^2+g_{s, i}t+h_{s, i}
$$

where $s \in \{x, y, z\}$ and $i \in \{1, ..., n-1\}$ for $n$ waypoints. First, we define the continuity constraints between spline segements, i.e.,

$$
\begin{align*}
s_{i}(t_i)&=p_{s, i}\\
s_{i}(t_{i+1})&=p_{s, i+1}
\end{align*}
$$

where $p_{s, i}$ is the $i\text{th}$ waypoint in dimension $s$ and $t_i$ is the time at the $i\text{th}$ waypoint. This implies:

$$
s_i(t_{i+1}) = s_{i+1}(t_i)
$$

which gives us continuity. For smoothness, we consider the following constraints on the spline derivatives:

$$
\begin{align*}
\dot{s}_i(t_{i+1})&=\dot{s}_{i+1}(t_i)\\
\ddot{s}_i(t_{i+1})&=\ddot{s}_{i+1}(t_i)\\
\overset{...}{s}_i(t_{i+1})&=\overset{...}{s}_{i+1}(t_i)\\
\end{align*}
$$

However, we'd like to restrict the derivatives at the start and end, i.e.,

$$
\begin{align*}
\dot{s}_1(t_1)&=0\\
\ddot{s}_1(t_1)&=0\\
\overset{...}{s}_1(t_1)&=0\\
\dot{s}_{n-1}(t_1)&=0\\
\ddot{s}_{n-1}(t_1)&=0\\
\overset{...}{s}_{n-1}(t_1)&=0
\end{align*}
$$

We now formulate the decision variable vector for each of our directions ($s$):

$$
\bold{x}_s = \begin{bmatrix}a_{s, 1} & \cdots & h_{s, 1} & \cdots & a_{s, n-1} & \cdots & h_{s, n-1}\end{bmatrix}^\top
$$

We can now organize all of our constraints into a large equality constraint

$$
\bold{A}_s\bold{x}_s = \bold{b}_s
$$

Moreover, we can formulate corridor constraints using linear inequalities if we find that our splines diverge from our desired path.

Moving on, we formulate the cost on snap. Snap is the fourth derivative of the position, i.e.,

$$
\text{snap} = \overset{....}{s}(t)=840a_{s}t^3+360b_st^2+120c_st+24d_s
$$

For the $i\text{th}$ spline segment, we have a cost of

$$
J_{s, i}=\int_{t_i}^{t_{i+1}}\left(840a_{s, i}t^3+360b_{s, i}t^2+120c_{s, i}t+24d_{s, i} \right)^2dt
$$

To optimize over all segments, we simply sum the cost values for each segment, i.e.,

$$
J_s = \sum_{i=1}^{n-1}J_{s, i}
$$

This can actually be organized into a quadratic cost:

$$
J_s = \bold{x}_s^\top \bold{H}_s\bold{x}_s + \bold{f}_s^\top \bold{x}_s
$$

All together, our minimum-snap trajectory optimization problem looks like:

$$
\begin{align*}
\min_{\bold{x_s}}  \quad & \bold{x}_s^\top \bold{H}_s\bold{x}_s + \bold{f}_s^\top \bold{x}_s \\
\textrm{s.t.} \quad & \bold{A}_s\bold{x}_s = \bold{b}_s\\
\end{align*}
$$

We can also solve for the coefficients for all three dimensions at once by stacking our constraints. For example:

$$
\begin{align*}
\bold{x} &= \begin{bmatrix}\bold{x}_x^\top & \bold{x}_y^\top & \bold{x}_z^\top\end{bmatrix}^\top\\

\bold{A} &= \begin{bmatrix}\bold{A}_x & \bold{0} & \bold{0}\\
\bold{0} & \bold{A}_y & \bold{0}\\
\bold{0} & \bold{0} & \bold{A}_z
\end{bmatrix} \\

\bold{b} &= \begin{bmatrix}
    \bold{b}_x^\top &
    \bold{b}_y^\top &
    \bold{b}_z^\top
\end{bmatrix}\\

\bold{H} &= \begin{bmatrix}\bold{H}_x & \bold{0} & \bold{0}\\
\bold{0} & \bold{H}_y & \bold{0}\\
\bold{0} & \bold{0} & \bold{H}_z
\end{bmatrix} \\

\bold{f} &= \begin{bmatrix}
    \bold{f}_x^\top &
    \bold{f}_y^\top &
    \bold{f}_z^\top
\end{bmatrix}\\
\end{align*}
$$

We reformulate our optimization as

$$
\begin{align*}
\min_{\bold{x}}  \quad & \bold{x}^\top \bold{H}\bold{x} + \bold{f}^\top \bold{x} \\
\textrm{s.t.} \quad & \bold{A}\bold{x} = \bold{b}\\
& \bold{A}_{corr}\bold{x}\leq\bold{b}_{corr}
\end{align*}
$$

where $\bold{A}_{corr}$ and $\bold{b}_{corr}$ are the matrices associated with additional corridor constraints.

## Control
### Nonlinear Geometric Control
To control the robot's motion three-dimensional space, a nonlinear geometric controller is designed, with feedback coming in the form of position ($\textbf{r}$), velocity ($\dot{\textbf{r}}$), orientation $\left({}^WR_B\right)$ and angular velocity ($\boldsymbol\omega$). The desired force necessary to command the robot's movement in $\mathbb{R}^3$, $\textbf{F}_{des}$, is given by the expression

$$
    \textbf{F}_{des} = m\ddot{\textbf{r}}_{des} + \begin{bmatrix}0\\0\\g\end{bmatrix}.
$$

The desired acceleration of the quadrotor is found via a proportional-derivative (PD) controller,

$$
    \ddot{\textbf{r}}_{des} = \ddot{\textbf{r}}_{T} + K_p(\textbf{r}_T - \textbf{r}) + K_d(\dot{\textbf{r}}_T - \dot{\textbf{r}})
$$

where the subscript $T$ represents the variables associated with a generated trajectory, and the gains are given by

$$
K_p = \begin{bmatrix}
    5 & 0 & 0 \\
    0 & 5 & 0 \\
    0 & 0 & 10
\end{bmatrix} \quad\text{and} \quad K_d = \begin{bmatrix}
    4.5 & 0 & 0 \\
    0 & 4.5 & 0 \\
    0 & 0 & 5
\end{bmatrix}
$$


with units of $s^{-2}$ and $s^{-1}$, respectively. The acceleration from the trajectory acts as a feedforward term, where corrections are made via the proportional controllers on position and velocity. Using equations (\ref{eqn:F_des}) and (\ref{eqn:r_ddot_des}), the thrust input $u_1$ to the quadrotor is calculated via

$$
    u_1 = \textbf{b}^T_3\textbf{F}_{des}
$$

where $\textbf{b}_3 = {}^WR_B\begin{bmatrix}
    0 & 0 & 1
\end{bmatrix}^T$ is the quadrotor's body z-axis expressed in the inertial frame. The input $u_1$ only provides the input necessary to keep the robot in the air, therefore a second input $u_2$ is used to change the robot's attitude. To calculate this input, a desired rotation based on a desired trajectory and input thrust is established. The input moments ($\textbf{u}_2$) drive the robot's current orientation to this desired orientation given by the expression 

$$
\left({}^WR_B\right)_{des} = \begin{bmatrix}
    \textbf{b}_{1_{des}} & \textbf{b}_{2_{des}} & \textbf{b}_{1_{des}}
\end{bmatrix}
$$

Since thrust always aligns with the body z-axis of the quadrotor, $\textbf{b}_{3_{des}}$ is defined as the normalized desired force vector:

$$
    \textbf{b}_{3_{des}} = \frac{\textbf{F}_{des}}{||\textbf{F}_{des}||}
$$

The body y-axis should be perpendicular to both the body x-axis and a vector which defines the yaw direction in the plane made up by the world x and y axes. The following yields an expression for the body y-axis, which ensures that the plane formed by the body z-axis and body x-axis contains the the vector defining yaw direction:

$$
    \textbf{b}_{2_{des}} = \frac{\textbf{b}_{3_{des}} \times \begin{bmatrix}
        \cos(\psi_T) & \sin(\psi_T) & 0
    \end{bmatrix}^T}{||\textbf{b}_{3_{des}} \times \begin{bmatrix}
        \cos(\psi_T) & \sin(\psi_T) & 0
    \end{bmatrix}^T||}
$$

where $\psi_T$ is the yaw given by a generated trajectory. The body x-axis is computed by taking the cross product between the body y-axis and body z-axis, i.e., 

$$
    \textbf{b}_{1_{des}} = \textbf{b}_{2_{des}} \times \textbf{b}_{3_{des}}.
$$

Equations (\ref{eqn:b_1_des}), (\ref{eqn:b_2_des}) and (\ref{eqn:b_3_des}) are plugged into equation \ref{eqn:R_des} to generate the desired attitude for the quadrotor. For the sake of simplicity, the superscripts and subscripts are dropped from both ${}^WR_B$ and $\left({}^WR_B\right)_{des}$. The orientation error is given by 

$$
    \textbf{e}_R = \frac{1}{2}\left(R_{des}^TR - R^TR_{des}\right)^{\vee}
$$

where $\vee$ is used to vectorize a skew-symmetric matrix. Moreover, the error in angular velocity $\textbf{e}_\omega = \boldsymbol\omega -\boldsymbol\omega_{des}$ is computed with the desired angular velocity being set to zero (though a trajectory-based desired angular velocity can be computed by exploiting differential flatness). 

The second input set is now generated as follows:

$$
    \textbf{u}_2 = -I(K_R\textbf{e}_R + K_\omega\textbf{e}_\omega)
$$

where $I$ is the inertia tensor of the robot and the gains are given by
$$
K_R = \begin{bmatrix}
    875 & 0 & 0 \\
    0 & 875 & 0 \\
    0 & 0 & 875
\end{bmatrix} \quad\text{and} \quad K_\omega = \begin{bmatrix}
    91 & 0 & 0 \\
    0 & 91 & 0 \\
    0 & 0 & 91
\end{bmatrix}
$$


with units of $s^{-2}$ and $s^{-1}$, respectively. 

The gain matrix $K_p$ provides a proportional constant on the position error of the robot in $\mathbb{R}^3$, which produces large accelerations with high error and less acceleration with low error. The gain matrix $K_d$ provides a proportional constant on the velocity error of the robot $\mathbb{R}^3$, which produces a damping effect on the system's acceleration. This helps to minimize the overshoot the quadrotor experiences when following trajectories or flying towards a point.

## Estimation
### Unscented Kalman Filter
To estimate the quadrotor's orientation and angular velocity, we will use an unscented Kalman filter. Our filter's state is 

$$
x = \begin{bmatrix}q\\ \omega \end{bmatrix} \in \mathbb{R}^7
$$

where $q$ is our quaternion orientation and $\omega$ is our angular velocity. Let's define our estimate at each time-step as the mean of our distribution

$$
\mu_{k|k}\in \mathbb{R}^7
$$

Since our quaternions will always be normalized to unit length, our covariance matrix is actually

$$
\Sigma_{k|k} \in \mathbb{R}^{6\times6}
$$

In our prediction step, we start by generating sigma points via an unscented transform. We generate $2n + 1$ sigma points, where $n$ corresponds to the dimension of our covariance matrix. Due to the difference in dimension between our covariance and our mean, we update the orientation and angular rate portions of our sigma points independently.

We use Cholesky decomposition to get the matrix sqaure root for the the upper left $3\times3$ portion of the covariance, associated with the orientation and do the same for the bottom right $3\times3$ portion, associated with the angular rate. Let's call these matrices $\sqrt{\Sigma_{q}}$ and $\sqrt{\Sigma_{\omega}}$, respectively. 

Since the sigma points associated with the orientation are quaternions, we must convert our representation to quaternion form. Let's define $\psi_s$ as an axis-angle representation given by

$$
\psi_{s, i} = \sqrt{2n}\left(\sqrt{\Sigma_{q, k|k}}_{(i)} \right)^\top 
$$

where $\sqrt{\Sigma_{q, k|k}}_{(i)}$ is the $i\text{th}$ row of the matrix $\sqrt{\Sigma_{q, k|k}}$. We then convert $\psi_s$ into the quaternion $q_{s, i}$. For the orientation portion, the first sigma points are therefore given by:

$$
\begin{align*}
s_{q, i} &= \mu_{q, k|k} * q_{s, i}, \quad \forall i \in [1, n] \\
s_{q, i} &= \mu_{q, k|k} * -q_{s, i}, \quad \forall i \in [n, 2n] \\
s_{q, 2n + 1} &= \mu_{q, k|k}
\end{align*}
$$

where $*$ represents quaternion multiplication. This is done as simply adding quaternions does not properly depict changes in orientation.

The angular velocity portion of the sigma points is easier to construct. The sigma points associated with $\omega$ are:


$$
\begin{align*}
s_{\omega, i} &= \mu_{\omega, k|k} + \sqrt{2n}\left(\sqrt{\Sigma_{\omega, k|k}}_{(i)}\right)^\top, \quad \forall i \in [1, n]\\
s_{\omega, i} &= \mu_{\omega, k|k} - \sqrt{2n}\left(\sqrt{\Sigma_{\omega, k|k}}_{(i)}\right)^\top, \quad \forall i \in [n, 2n]\\
s_{\omega, 2n+1} &= \mu_{\omega, k|k}
\end{align*}
$$

Each sigma point is now propagated through our dynamics function 

$$
x_{k+1} = f(x_k) = \begin{bmatrix}q_k * \Delta q_k\\
\omega_k\end{bmatrix}
$$

where $\Delta q_k$ is the quaternion equivalent of the axis-angle representation of $\omega_k \Delta t$. Due to part of our state being a quaternion, we use the algorithm described in [this paper](https://ieeexplore.ieee.org/document/1257247) to compute the "quaternion mean" and "quaternion covariance". In this case, this corresponds to finding $\mu_{q, k+1|k}$ and $\Sigma_{q, k+1|k}$, respectively. For angular velocity, We simply take find the propagated mean and covariance with:

$$
\begin{align*}
\mu_{\omega, k+1|k} &= \frac{1}{2n+1}\sum_{i=1}^{2n+1}f(s_i)_\omega \\
\Sigma_{\omega, k+1|k} &= R + \frac{1}{2n+1}\sum_{i=1}^{2n+1}\left(f(s_i)_\omega - \mu_{\omega, k+1|k}\right)\left(f(s_i)_\omega - \mu_{\omega, k+1|k}\right)^\top
\end{align*}
$$

where $R$ is a tune-able process noise.

For the update step, we start by taking the propagated sigma points and propagating them through the measurement model (though generating new sigma points should also work). The measurement function is represented by:

$$
y_k = g(x_k) = \begin{bmatrix}\text{vec}\left(q_k^{-1} * q_g * q_k\right)\\
\omega_k \end{bmatrix}
$$

where $q_g$ is the quaternion representation of the acceleration due to gravity vector and $\text{vec}$ represents the conversion of a quaternion object to Euclidean space. This means that $y_k \in \mathbb{R}^6$. Since no portion of our output is in quaternion space, we find

$$
\hat{y} = \frac{1}{2n+1}\sum_{i=1}^{2n+1}g\bigl(f(s_i) \bigr)
$$

We now compute the covariance values

$$
\begin{align*}
\Sigma_{yy} &= Q + \frac{1}{2n+1}\sum_{i=1}^{2n+1}\Bigl(g\bigl(f(s_i) \bigr) - \hat{y}\Bigr)\Bigl(g\bigl(f(s_i) \bigr) - \hat{y}\Bigr)^\top \\
\Sigma_{xy} = \Sigma_{yx} &= \frac{1}{2n+1}\sum_{i=1}^{2n+1}\Bigl(s_i - \mu_{k+1|k}\Bigr)\Bigl(g\bigl(f(s_i) \bigr) - \hat{y}\Bigr)^\top \\
\end{align*}
$$

where $Q$ is the tune-able measurement covariance and the quaternion portion of $s_i - \mu_{k+1|k}$ is calculated using elements from the quaternion covariance algorithm.

Now, we compute the Kalman gain

$$
K = \Sigma_{xy} \Sigma_{yy}^{-1}
$$

and the innovation vector 

$$
\text{innovation} = y_k - \hat{y}
$$

We compute the quaternion portion of $\mu_{k+1|k+1}$ by converting the upper three elements of the Kalman gain multiplied by the innovation vector to a quaternion and doing matrix multiplication. The angular velocity portion of $\mu_{k+1|k+1}$ is computed using

$$
\mu_{k+1|k+1} = \mu_{k+1|k} + (K \cdot \text{innovation})_\omega
$$

Finally, the covariance is calculated as

$$
\Sigma_{k+1|k+1} = \Sigma_{k+1|k} - K\Sigma_{yy}K^\top
$$

