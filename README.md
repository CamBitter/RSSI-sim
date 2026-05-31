### RSSI Simulator

Simulates dining hall line traffic to classify line size instead of general room occupancy.

Uses [DearPyGui](https://github.com/hoffstadt/dearpygui) for sim rendering.

### Setup

`uv sync`

`uv run main.py`

### Methods

Agent movement is a simplified [Social Force Model](https://pedestriandynamics.org/models/social_force_model/). Each frame, every agent accumulates three forces into its velocity, then integrates position.

**Driving force** — steers an agent toward its target at desired speed $v_0$, damped by current velocity over reaction time $\tau$:

$$\vec{F}_\text{drive} = \frac{v_0\,\hat{e} - \vec{v}}{\tau}$$

$\hat{e}$ is the unit vector toward the target. The $-\vec{v}$ term provides built-in damping. A multiplier scales $v_0$ up for the queue head (clears the servery) and for trailing groups.

**Repulsion force** — agents push apart with an exponential that rises sharply on contact, summed over neighbors within radius $R$:

$$\vec{F}_\text{rep} = \sum_{j} A \, \exp\!\left(\frac{r_i + r_j - d_{ij}}{B}\right) \hat{n}_{ij}$$

$d_{ij}$ is the inter-agent distance, $r_i + r_j$ their combined radii, $\hat{n}_{ij}$ the unit vector from neighbor to agent. $A$ sets strength, $B$ sets falloff range.

**Containment force** — a soft spring pulling agents back toward the queue centerline, applied only outside a corridor of half-width $w$:

$$\vec{F}_\text{con} = \begin{cases} -C \, d_\perp \, \hat{p} & \text{if } |d_\perp| > w \\ 0 & \text{otherwise} \end{cases}$$

$d_\perp$ is the signed perpendicular distance to the centerline, $\hat{p}$ the perpendicular unit vector, $C$ the strength.

**Integration** — per frame, in two passes (compute forces from a frozen snapshot, then move):

$$\vec{v} \leftarrow \vec{v} + \vec{F}_\text{drive} + \frac{\vec{F}_\text{rep} + \vec{F}_\text{con}}{m}$$

$$\vec{x} \leftarrow \vec{x} + \vec{v}$$

### Targeting

- **Head** of the front group drives at the end point; brakes on arrival.
- **Front-group followers** chain to the agent ahead, offset one `gap` upstream.
- **Trailing groups** follow the mean position of the group ahead, offset by `gap`.