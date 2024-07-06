# Autonomous quadrotor
<p align="center">
  <img src="https://github.com/AlexKoldy/autonomous_quadrotor/assets/52255127/b24c9c50-8568-4496-8354-f5cc870fd3e4" />
</p>

![image](https://github.com/AlexKoldy/autonomous_quadrotor/assets/52255127/7176d963-838a-43c3-9cfa-6f451faeb113)



## Planning
### Bidirectional A*
We use A* Search, a hallmark graph algorithm in robotics, which builds upon Dijkstra's Algorithm by expanding nodes prioritized by the sum of their "cost-to-come" ($g(v): V \rightarrow \mathbb{R}$) and an estimated "cost-to-go" ($h(v): V \rightarrow \mathbb{R}$), $f(v) = g(v) + h(v)$. We take as input a graph $G(V, E)$, a source node $v\_s \in V$, and a sink node $v\_g \in V$. 

The "cost-to-come" term $g(v)$ defines the cost of following the optimal sub-path from $v\_s$ to $v \in V$. The "cost-to-go" term, also known as a heuristic $h(v)$, is an estimate of the cost to follow the optimal path from $v \in V$ to $v\_g$. We ensure that A* Search is optimal and complete by using a heuristic $h(v)$ that is both admissible and consistent. 

We consider $h(v)$ admissible if $\forall v \in V$, $h(v) \leq c^\star(v, v\_g)$, where $c^*(v)$ is the true cost of the optimal path from $v$ to $v\_g$. We consider $h(v)$ consistent if $\forall u, v \in V$ such that $e(u, v) \in E$, $h(u) \leq c(u, v) + h(v)$. 

We choose $h$ as the Euclidean ($L\_2$) distance. Other candidates include the Manhattan ($L\_1$) distance and Diagonal or Chebyshev ($L\_{\infty}$) distance. 

By using A* Search with these heuristics, we will return a path $p^* = (v\_s, ..., v\_g)$ which minimizes the Euclidean distance traversing $e \in E$ from $v\_s$ to $v\_g$.


We can apply this algorithm from the source node to the sink node and from the sink node to the source node at the same time. When the two paths meet, the algorithm converges to the shortest path. With a good heuristic choice, this will improve convergence time.

### Waypoint pruning
Assuming we have performed bidirectional A* and obtained a list of waypoints, we can use the Ramer–Douglas–Peucker (RDP) algorithm to reduce the number of waypoints before feeding them into a trajectory generator. The RDP algorithm simplifies a path by removing points that do not significantly contribute to the overall shape, thus reducing complexity while maintaining the path's essential form.

The RDP algorithm works as follows:

1. **Initial Step**: Given a set of waypoints $P = \{p\_1, p\_2, \ldots, p\_n\}$, the algorithm starts with the first point $p\_1$ and the last point $p\_n$ as the initial endpoints of the simplified path.

2. **Finding the Point of Maximum Distance**: For each point $p\_i$ in the set $P$, the perpendicular distance $d\_i$ from the point to the line segment $p\_1p\_n$ is calculated. This distance is given by:

$$
d\_i = \frac{| (p\_n - p\_1) \times (p\_1 - p\_i) |}{\| p\_n - p\_1 \|}
$$

$\quad \quad$ where $\times$ denotes the cross product and $\| \cdot \|$ denotes the Euclidean norm.

4. **Recursive Subdivision**: Find the point $p\_k$ with the maximum distance $d\_k$ from the line segment $p\_1p\_n$. If $d\_k$ is greater than a predefined threshold $\epsilon$, the algorithm keeps $p\_k$ and recursively applies the same process to the segments $p\_1p\_k$ and $p\_kp\_n$. If $d\_k \leq \epsilon$, all points between $p\_1$ and $p\_n$ are discarded.

5. **Constructing the Simplified Path**: The process is repeated recursively for each sub-segment until all points are processed. The remaining points form the simplified path.

<p align="center">
  <img src="https://github.com/AlexKoldy/autonomous_quadrotor/assets/52255127/1d4a58c6-b6a2-4bb0-85b2-17c92b0a7745" />
</p>

### Minimum-Snap Trajectory Generation
To generate minimum-snap trajectories, we use a $7\text{th}$ order polynomial to represent the position (in one dimension) of the quadrotor:

$$
s\_{i}(t)=a\_{s, i}t^7+b\_{s, i}t^6+c\_{s, i}t^5+d\_{s, i}t^4+e\_{s, i}t^3+f\_{s, i}t^2+g\_{s, i}t+h\_{s, i}
$$

where $s \in \{x, y, z\}$ and $i \in \{1, ..., n-1\}$ for $n$ waypoints. First, we define the continuity constraints between spline segements, i.e.,

$$
\begin{align*}
s\_{i}(t\_i)&=p\_{s, i}\\
s\_{i}(t\_{i+1})&=p\_{s, i+1}
\end{align*}
$$

where $p\_{s, i}$ is the $i\text{th}$ waypoint in dimension $s$ and $t\_i$ is the time at the $i\text{th}$ waypoint. This implies:

$$
s\_i(t\_{i+1}) = s\_{i+1}(t\_i)
$$

which gives us continuity. For smoothness, we consider the following constraints on the spline derivatives:

$$
\begin{align*}
\dot{s}\_i(t\_{i+1})&=\dot{s}\_{i+1}(t\_i)\\
\ddot{s}\_i(t\_{i+1})&=\ddot{s}\_{i+1}(t\_i)\\
\overset{...}{s}\_i(t\_{i+1})&=\overset{...}{s}\_{i+1}(t\_i)\\
\end{align*}
$$

However, we'd like to restrict the derivatives at the start and end, i.e.,

$$
\begin{align*}
\dot{s}\_1(t\_1)&=0\\
\ddot{s}\_1(t\_1)&=0\\
\overset{...}{s}\_1(t\_1)&=0\\
\dot{s}\_{n-1}(t\_1)&=0\\
\ddot{s}\_{n-1}(t\_1)&=0\\
\overset{...}{s}\_{n-1}(t\_1)&=0
\end{align*}
$$

We now formulate the decision variable vector for each of our directions ($s$):

$$
\symbf{x}\_s = \begin{bmatrix}a\_{s, 1} & \cdots & h\_{s, 1} & \cdots & a\_{s, n-1} & \cdots & h\_{s, n-1}\end{bmatrix}^\top
$$

We can now organize all of our constraints into a large equality constraint

$$
\symbf{A}\_s\symbf{x}\_s = \symbf{b}\_s
$$

Moreover, we can formulate corridor constraints using linear inequalities if we find that our splines diverge from our desired path.

Moving on, we formulate the cost on snap. Snap is the fourth derivative of the position, i.e.,

$$
\text{snap} = \overset{....}{s}(t)=840a\_{s}t^3+360b\_st^2+120c\_st+24d\_s
$$

For the $i\text{th}$ spline segment, we have a cost of

$$
J\_{s, i}=\int\_{t\_i}^{t\_{i+1}}\left(840a\_{s, i}t^3+360b\_{s, i}t^2+120c\_{s, i}t+24d\_{s, i} \right)^2dt
$$

To optimize over all segments, we simply sum the cost values for each segment, i.e.,

$$
J\_s = \sum\_{i=1}^{n-1}J\_{s, i}
$$

This can actually be organized into a quadratic cost:

$$
J\_s = \symbf{x}\_s^\top \symbf{H}\_s\symbf{x}\_s + \symbf{f}\_s^\top \symbf{x}\_s
$$

All together, our minimum-snap trajectory optimization problem looks like:

$$
\begin{align*}
\min\_{\symbf{x\_s}}  \quad & \symbf{x}\_s^\top \symbf{H}\_s\symbf{x}\_s + \symbf{f}\_s^\top \symbf{x}\_s \\
\textrm{s.t.} \quad & \symbf{A}\_s\symbf{x}\_s = \symbf{b}\_s\\
\end{align*}
$$

We can also solve for the coefficients for all three dimensions at once by stacking our constraints. For example:

$$
\begin{align*}
\symbf{x} &= \begin{bmatrix}\symbf{x}\_x^\top & \symbf{x}\_y^\top & \symbf{x}\_z^\top\end{bmatrix}^\top\\
\symbf{A} &= \begin{bmatrix}\symbf{A}\_x & \symbf{0} & \symbf{0}\\
\symbf{0} & \symbf{A}\_y & \symbf{0}\\
\symbf{0} & \symbf{0} & \symbf{A}\_z
\end{bmatrix} \\
\symbf{b} &= \begin{bmatrix}
    \symbf{b}\_x^\top &
    \symbf{b}\_y^\top &
    \symbf{b}\_z^\top
\end{bmatrix}\\
\symbf{H} &= \begin{bmatrix}\symbf{H}\_x & \symbf{0} & \symbf{0}\\
\symbf{0} & \symbf{H}\_y & \symbf{0}\\
\symbf{0} & \symbf{0} & \symbf{H}\_z
\end{bmatrix} \\
\symbf{f} &= \begin{bmatrix}
    \symbf{f}\_x^\top &
    \symbf{f}\_y^\top &
    \symbf{f}\_z^\top
\end{bmatrix}\\
\end{align*}
$$

We reformulate our optimization as

$$
\begin{align*}
\min\_{\symbf{x}}  \quad & \symbf{x}^\top \symbf{H}\symbf{x} + \symbf{f}^\top \symbf{x} \\
\textrm{s.t.} \quad & \symbf{A}\symbf{x} = \symbf{b}\\
& \symbf{A}\_{corr}\symbf{x}\leq\symbf{b}\_{corr}
\end{align*}
$$

where $\symbf{A}\_{corr}$ and $\symbf{b}\_{corr}$ are the matrices associated with additional corridor constraints.

<p align="center">
  <img src="https://github.com/AlexKoldy/autonomous_quadrotor/assets/52255127/8e60bd03-b854-449e-b1af-aea0268f555e" />
</p>



## Control
### Nonlinear Geometric Control
To control the robot's motion three-dimensional space, a nonlinear geometric controller is designed, with feedback coming in the form of position ($\textbf{r}$), velocity ($\dot{\textbf{r}}$), orientation $\left({}^WR\_B\right)$ and angular velocity ($\symbf\omega$). The desired force necessary to command the robot's movement in $\mathbb{R}^3$, $\textbf{F}\_{des}$, is given by the expression

$$
    \textbf{F}\_{des} = m\ddot{\textbf{r}}\_{des} + \begin{bmatrix}0 \\\ 0 \\\ g\end{bmatrix}.
$$

The desired acceleration of the quadrotor is found via a proportional-derivative (PD) controller,

$$
    \ddot{\textbf{r}}\_{des} = \ddot{\textbf{r}}\_{T} + K\_p(\textbf{r}\_T - \textbf{r}) + K\_d(\dot{\textbf{r}}\_T - \dot{\textbf{r}})
$$

where the subscript $T$ represents the variables associated with a generated trajectory, and the gains are given by

$$
K\_p = \begin{bmatrix}
    5 & 0 & 0 \\
    0 & 5 & 0 \\
    0 & 0 & 10
\end{bmatrix} \quad\text{and} \quad K\_d = \begin{bmatrix}
    4.5 & 0 & 0 \\
    0 & 4.5 & 0 \\
    0 & 0 & 5
\end{bmatrix}
$$


with units of $s^{-2}$ and $s^{-1}$, respectively. The acceleration from the trajectory acts as a feedforward term, where corrections are made via the proportional controllers on position and velocity. Using equations (\ref{eqn:F\_des}) and (\ref{eqn:r\_ddot\_des}), the thrust input $u\_1$ to the quadrotor is calculated via

$$
    u\_1 = \textbf{b}^T\_3\textbf{F}\_{des}
$$

where $\textbf{b}\_3 = {}^WR\_B\begin{bmatrix}
    0 & 0 & 1
\end{bmatrix}^T$ is the quadrotor's body z-axis expressed in the inertial frame. The input $u\_1$ only provides the input necessary to keep the robot in the air, therefore a second input $u\_2$ is used to change the robot's attitude. To calculate this input, a desired rotation based on a desired trajectory and input thrust is established. The input moments ($\textbf{u}\_2$) drive the robot's current orientation to this desired orientation given by the expression 

$$
\left({}^WR\_B\right)\_{des} = \begin{bmatrix}
    \textbf{b}\_{1\_{des}} & \textbf{b}\_{2\_{des}} & \textbf{b}\_{1\_{des}}
\end{bmatrix}
$$

Since thrust always aligns with the body z-axis of the quadrotor, $\textbf{b}\_{3\_{des}}$ is defined as the normalized desired force vector:

$$
    \textbf{b}\_{3\_{des}} = \frac{\textbf{F}\_{des}}{||\textbf{F}\_{des}||}
$$

The body y-axis should be perpendicular to both the body x-axis and a vector which defines the yaw direction in the plane made up by the world x and y axes. The following yields an expression for the body y-axis, which ensures that the plane formed by the body z-axis and body x-axis contains the the vector defining yaw direction:

$$
    \textbf{b}\_{2\_{des}} = \frac{\textbf{b}\_{3\_{des}} \times \begin{bmatrix}
        \cos(\psi\_T) & \sin(\psi\_T) & 0
    \end{bmatrix}^T}{||\textbf{b}\_{3\_{des}} \times \begin{bmatrix}
        \cos(\psi\_T) & \sin(\psi\_T) & 0
    \end{bmatrix}^T||}
$$

where $\psi\_T$ is the yaw given by a generated trajectory. The body x-axis is computed by taking the cross product between the body y-axis and body z-axis, i.e., 

$$
    \textbf{b}\_{1\_{des}} = \textbf{b}\_{2\_{des}} \times \textbf{b}\_{3\_{des}}.
$$

Equations (\ref{eqn:b\_1\_des}), (\ref{eqn:b\_2\_des}) and (\ref{eqn:b\_3\_des}) are plugged into equation \ref{eqn:R\_des} to generate the desired attitude for the quadrotor. For the sake of simplicity, the superscripts and subscripts are dropped from both ${}^WR\_B$ and $\left({}^WR\_B\right)\_{des}$. The orientation error is given by 

$$
    \textbf{e}\_R = \frac{1}{2}\left(R\_{des}^TR - R^TR\_{des}\right)^{\vee}
$$

where $\vee$ is used to vectorize a skew-symmetric matrix. Moreover, the error in angular velocity $\textbf{e}\_\omega = \symbf\omega -\symbf\omega\_{des}$ is computed with the desired angular velocity being set to zero (though a trajectory-based desired angular velocity can be computed by exploiting differential flatness). 

The second input set is now generated as follows:

$$
    \textbf{u}\_2 = -I(K\_R\textbf{e}\_R + K\_\omega\textbf{e}\_\omega)
$$

where $I$ is the inertia tensor of the robot and the gains are given by

$$
K\_R = \begin{bmatrix}
    875 & 0 & 0 \\
    0 & 875 & 0 \\
    0 & 0 & 875
\end{bmatrix} \quad\text{and} \quad K\_\omega = \begin{bmatrix}
    91 & 0 & 0 \\
    0 & 91 & 0 \\
    0 & 0 & 91
\end{bmatrix}
$$


with units of $s^{-2}$ and $s^{-1}$, respectively. 

The gain matrix $K\_p$ provides a proportional constant on the position error of the robot in $\mathbb{R}^3$, which produces large accelerations with high error and less acceleration with low error. The gain matrix $K\_d$ provides a proportional constant on the velocity error of the robot $\mathbb{R}^3$, which produces a damping effect on the system's acceleration. This helps to minimize the overshoot the quadrotor experiences when following trajectories or flying towards a point.
![image](https://github.com/AlexKoldy/autonomous_quadrotor/assets/52255127/2f2291d3-9dab-4661-804a-aa128e9b220c)


## Estimation
### Unscented Kalman Filter
To estimate the quadrotor's orientation and angular velocity, we will use an unscented Kalman filter. Our filter's state is 

$$
x = \begin{bmatrix}q\\ \omega \end{bmatrix} \in \mathbb{R}^7
$$

where $q$ is our quaternion orientation and $\omega$ is our angular velocity. Let's define our estimate at each time-step as the mean of our distribution

$$
\mu\_{k|k}\in \mathbb{R}^7
$$

Since our quaternions will always be normalized to unit length, our covariance matrix is actually

$$
\Sigma\_{k|k} \in \mathbb{R}^{6\times6}
$$

In our prediction step, we start by generating sigma points via an unscented transform. We generate $2n + 1$ sigma points, where $n$ corresponds to the dimension of our covariance matrix. Due to the difference in dimension between our covariance and our mean, we update the orientation and angular rate portions of our sigma points independently.

We use Cholesky decomposition to get the matrix sqaure root for the the upper left $3\times3$ portion of the covariance, associated with the orientation and do the same for the bottom right $3\times3$ portion, associated with the angular rate. Let's call these matrices $\sqrt{\Sigma\_{q}}$ and $\sqrt{\Sigma\_{\omega}}$, respectively. 

Since the sigma points associated with the orientation are quaternions, we must convert our representation to quaternion form. Let's define $\psi\_s$ as an axis-angle representation given by

$$
\psi\_{s, i} = \sqrt{2n}\left(\sqrt{\Sigma\_{q, k|k}}\_{(i)} \right)^\top 
$$

where $\sqrt{\Sigma\_{q, k|k}}\_{(i)}$ is the $i\text{th}$ row of the matrix $\sqrt{\Sigma\_{q, k|k}}$. We then convert $\psi\_s$ into the quaternion $q\_{s, i}$. For the orientation portion, the first sigma points are therefore given by:

$$
\begin{align*}
s\_{q, i} &= \mu\_{q, k|k} * q\_{s, i}, \quad \forall i \in [1, n] \\
s\_{q, i} &= \mu\_{q, k|k} * -q\_{s, i}, \quad \forall i \in [n, 2n] \\
s\_{q, 2n + 1} &= \mu\_{q, k|k}
\end{align*}
$$

where $*$ represents quaternion multiplication. This is done as simply adding quaternions does not properly depict changes in orientation.

The angular velocity portion of the sigma points is easier to construct. The sigma points associated with $\omega$ are:


$$
\begin{align*}
s\_{\omega, i} &= \mu\_{\omega, k|k} + \sqrt{2n}\left(\sqrt{\Sigma\_{\omega, k|k}}\_{(i)}\right)^\top, \quad \forall i \in [1, n]\\
s\_{\omega, i} &= \mu\_{\omega, k|k} - \sqrt{2n}\left(\sqrt{\Sigma\_{\omega, k|k}}\_{(i)}\right)^\top, \quad \forall i \in [n, 2n]\\
s\_{\omega, 2n+1} &= \mu\_{\omega, k|k}
\end{align*}
$$

Each sigma point is now propagated through our dynamics function 

$$
x\_{k+1} = f(x\_k) = \begin{bmatrix}q\_k * \Delta q\_k\\
\omega\_k\end{bmatrix}
$$

where $\Delta q\_k$ is the quaternion equivalent of the axis-angle representation of $\omega\_k \Delta t$. Due to part of our state being a quaternion, we use the algorithm described in [this paper](https://ieeexplore.ieee.org/document/1257247) to compute the "quaternion mean" and "quaternion covariance". In this case, this corresponds to finding $\mu\_{q, k+1|k}$ and $\Sigma\_{q, k+1|k}$, respectively. For angular velocity, We simply take find the propagated mean and covariance with:

$$
\begin{align*}
\mu\_{\omega, k+1|k} &= \frac{1}{2n+1}\sum\_{i=1}^{2n+1}f(s\_i)\_\omega \\
\Sigma\_{\omega, k+1|k} &= R + \frac{1}{2n+1}\sum\_{i=1}^{2n+1}\left(f(s\_i)\_\omega - \mu\_{\omega, k+1|k}\right)\left(f(s\_i)\_\omega - \mu\_{\omega, k+1|k}\right)^\top
\end{align*}
$$

where $R$ is a tune-able process noise.

For the update step, we start by taking the propagated sigma points and propagating them through the measurement model (though generating new sigma points should also work). The measurement function is represented by:

$$
y\_k = g(x\_k) = \begin{bmatrix}\text{vec}\left(q\_k^{-1} * q\_g * q\_k\right)\\
\omega\_k \end{bmatrix}
$$

where $q\_g$ is the quaternion representation of the acceleration due to gravity vector and $\text{vec}$ represents the conversion of a quaternion object to Euclidean space. This means that $y\_k \in \mathbb{R}^6$. Since no portion of our output is in quaternion space, we find

$$
\hat{y} = \frac{1}{2n+1}\sum\_{i=1}^{2n+1}g\bigl(f(s\_i) \bigr)
$$

We now compute the covariance values

$$
\begin{align*}
\Sigma\_{yy} &= Q + \frac{1}{2n+1}\sum\_{i=1}^{2n+1}\Bigl(g\bigl(f(s\_i) \bigr) - \hat{y}\Bigr)\Bigl(g\bigl(f(s\_i) \bigr) - \hat{y}\Bigr)^\top \\
\Sigma\_{xy} = \Sigma\_{yx} &= \frac{1}{2n+1}\sum\_{i=1}^{2n+1}\Bigl(s\_i - \mu\_{k+1|k}\Bigr)\Bigl(g\bigl(f(s\_i) \bigr) - \hat{y}\Bigr)^\top \\
\end{align*}
$$

where $Q$ is the tune-able measurement covariance and the quaternion portion of $s\_i - \mu\_{k+1|k}$ is calculated using elements from the quaternion covariance algorithm.

Now, we compute the Kalman gain

$$
K = \Sigma\_{xy} \Sigma\_{yy}^{-1}
$$

and the innovation vector 

$$
\text{innovation} = y\_k - \hat{y}
$$

We compute the quaternion portion of $\mu\_{k+1|k+1}$ by converting the upper three elements of the Kalman gain multiplied by the innovation vector to a quaternion and doing matrix multiplication. The angular velocity portion of $\mu\_{k+1|k+1}$ is computed using

$$
\mu\_{k+1|k+1} = \mu\_{k+1|k} + (K \cdot \text{innovation})\_\omega
$$

Finally, the covariance is calculated as

$$
\Sigma\_{k+1|k+1} = \Sigma\_{k+1|k} - K\Sigma\_{yy}K^\top
$$

We compare the roll, pitch yaw values extracted from the quaternion information and compare against those of the Vicon system.

![image](https://github.com/AlexKoldy/autonomous_quadrotor/assets/52255127/96c9134e-10d3-4680-9b7c-2ca2df898592)


