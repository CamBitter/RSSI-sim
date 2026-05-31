### RSSI Simulator

Simulates dining hall line traffic to classify line size instead of general room occupancy.

Uses [DearPyGui](https://github.com/hoffstadt/dearpygui) for sim rendering.

### Setup

`uv sync`

`uv run main.py`

### Methods

Agent movement is a simplified [Social Force Model](https://pedestriandynamics.org/models/social_force_model/). Each frame, every agent accumulates three forces into its velocity, then integrates position.

**Driving force** — steers an agent toward its target at desired speed `v0`, damped by current velocity over reaction time `τ`:

```
F_drive = (v0 · ê − v) / τ
```

`ê` is the unit vector toward the target. The `−v` term provides built-in damping. A multiplier scales `v0` up for the queue head (clears the servery) and trailing groups.

**Repulsion force** — agents push apart with an exponential that rises sharply on contact, summed over neighbors within radius `R`:

```
F_rep = Σ  A · exp((r_i + r_j − d_ij) / B) · n̂_ij
```

`d_ij` is the inter-agent distance, `r_i + r_j` their combined radii, `n̂_ij` the unit vector from neighbor to agent. `A` sets strength, `B` sets falloff range.

**Containment force** — a soft spring pulling agents back toward the queue centerline, applied only outside a corridor of half-width `w`:

```
F_con = −C · d⊥ · p̂     (if |d⊥| > w, else 0)
```

`d⊥` is the signed perpendicular distance to the centerline, `p̂` the perpendicular unit vector, `C` the strength.

**Integration** (per frame, two passes — forces from a frozen snapshot, then move):

```
v ← v + F_drive + (F_rep + F_con) / m
x ← x + v
```

### Targeting

- **Head** of the front group drives at the end point; brakes on arrival.
- **Front-group followers** chain to the agent ahead, offset one `gap` upstream.
- **Trailing groups** follow the mean position of the group ahead, offset by `gap`.