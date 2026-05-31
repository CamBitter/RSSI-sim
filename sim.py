from agent import Agent
import math


class Sim:
    def __init__(self, agents, params):
        """
        Params:
            - move_speed                Movement speed of agents in pixels per frame
            - proximity_threshold       Distance at which agents are considered to have reached the end point, in pixels
            - wait_time                 Number of frames agents wait at the end point before being removed from the sim
        """

        self.agents = agents
        self.params = params
        self.next_id = len(agents) + 1

    def spawn(self, x, y):
        """Spawn a new agent at (x, y) with velocity towards last agent in queue"""

        agent = Agent(self.next_id, f"00:00:00:00:00:{self.next_id:02x}", x, y, 0, 0)

        # Calculate unit vector towards last agent in queue, or towards end if no agents
        if self.agents:
            xu, yu = self._agent_to_agent_vec(agent, self.agents[-1])
        else:
            xu, yu = self._agent_to_point_vec(agent, self.params["end_point"])

        # Set velocity along unit vector
        agent.xv = xu * self.params["desired_speed"]
        agent.yv = yu * self.params["desired_speed"]

        self.agents.append(agent)
        self.next_id += 1

    def step(self):
        p = self.params
        v0, tau = p["desired_speed"], p["reaction_time"]
        A, B, R = p["A"], p["B"], p["influence_radius"]
        E  = p["end_point"]
        PT = p["proximity_threshold"]
        AS = p["alignment_strength"]

        # Reversed centerline unit vector (points from end back toward start —
        # i.e. the direction "behind the leader" in the queue)
        sx, sy = p["start_point"]
        ex, ey = E
        cdx, cdy = sx - ex, sy - ey            # end -> start
        clen = math.hypot(cdx, cdy)
        back_ux, back_uy = (cdx / clen, cdy / clen) if clen else (0.0, 0.0)
        gap = p["follow_gap"]

        # Check if head of queue has reached end point, and if so, increment checkpoint timer or remove from sim
        head = self.agents[0] if self.agents else None
        if head and self._agent_to_point_dist(head, E) < PT:
            if head.checkpoint_timer < self.params["wait_time"]:
                head.checkpoint_timer += 1

            else:
                self.agents.pop(0)
                print(f"Agent {head.id} reached end point")

        if not self.agents:
            return

        # Update agent velocities
        dvx_arr = []
        dvy_arr = []
        for i, agent in enumerate(self.agents):
            # Get coordinates of target (end point for head of queue, prior agent for others)
            if i == 0:
                target = E
                if self._agent_to_point_dist(agent, E) < PT:
                    agent.xv *= 0.5
                    agent.yv *= 0.5
                    continue  
            else:
                prior = self.agents[i - 1]

                # Where agent is trying to go
                target = (prior.x + back_ux * gap, prior.y + back_uy * gap)

            # Get unit vector towards target
            xu, yu = self._agent_to_point_vec(agent, target)

            # Calculate driving forces
            driving_force_x = (v0 * xu - agent.xv) / tau
            driving_force_y = (v0 * yu - agent.yv) / tau

            # Calculate repulsion forces from nearby agents
            repulsion_force_x, repulsion_force_y = 0, 0
            sum_neighbor_vx, sum_neighbor_vy = 0, 0
            n = 0

            for j, other in enumerate(self.agents):
                if other is agent:
                    continue

                dist = self._agent_to_agent_dist(agent, other)
                if dist < R:
                    # Repulsion force magnitude decreases with distance
                    A_exp = math.exp((agent.radius + other.radius - dist) / B)

                    repulsion_force_x += A * A_exp * (agent.x - other.x) / dist
                    repulsion_force_y += A * A_exp * (agent.y - other.y) / dist

                    # Follow neighbors nearby that are in front of agent
                    if j < i:
                        sum_neighbor_vx += other.xv
                        sum_neighbor_vy += other.yv
                        n += 1

            # Containment force to keep agents in corridor
            containment_force_x, containment_force_y = self._containment_force(agent)

            # Combine forces
            dvx = driving_force_x + (repulsion_force_x + containment_force_x) / agent.mass
            dvy = driving_force_y + (repulsion_force_y + containment_force_y) / agent.mass

            # Add group aligning force to encourage agents to match velocity of neighbors
            if n > 0:
                avg_neighbor_vx = sum_neighbor_vx / n
                avg_neighbor_vy = sum_neighbor_vy / n
                dvx += AS * (avg_neighbor_vx - agent.xv)
                dvy += AS * (avg_neighbor_vy - agent.yv)

            agent.xv += dvx
            agent.yv += dvy

        # Update agent positions
        for agent in self.agents:
            agent.x += agent.xv
            agent.y += agent.yv

    def _containment_force(self, a):
        """Calculate a force that pushes agent a back towards the center line between start and end points, with strength proportional to the perpendicular distance from the line scaled by containment_strength parameter"""

        sx, sy = self.params["start_point"]
        ex, ey = self.params["end_point"]

        dx, dy = ex - sx, ey - sy
        length = math.hypot(dx, dy)
        if length == 0:
            return (0, 0)

        # Unit vector from start to end
        ux, uy = dx / length, dy / length

        # Perpendicular unit vec
        px, py = -uy, ux

        # Signed perpendicular distance from agent to center line
        wx, wy = a.x - sx, a.y - sy
        perp_dist = wx * px + wy * py

        C = self.params["containment_strength"]
        corridor_width = self.params["corridor_width"]

        if abs(perp_dist) < corridor_width:
            return (0.0, 0.0)

        return (-C * perp_dist * px, -C * perp_dist * py)

    def _agent_to_agent_dist(self, a, b):
        """Distance from agent a to agent b"""
        return math.hypot(a.x - b.x, a.y - b.y)

    def _agent_to_point_dist(self, a, pt):
        """Distance from agent a to point pt, where pt is a tuple (x, y)"""
        x, y = pt
        return math.hypot(a.x - x, a.y - y)

    def _agent_to_agent_vec(self, a, b):
        """Unit vector from agent a to agent b"""
        dist = self._agent_to_agent_dist(a, b)
        if dist == 0:
            return (0, 0)
        return (b.x - a.x) / dist, (b.y - a.y) / dist

    def _agent_to_point_vec(self, a, pt):
        """Unit vector from agent a to point pt, where pt is a tuple (x, y)"""
        x, y = pt
        dist = self._agent_to_point_dist(a, pt)
        if dist == 0:
            return (0, 0)
        return (x - a.x) / dist, (y - a.y) / dist
