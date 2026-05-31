from statistics import mean

from agent import Agent
import math
import random


class Sim:
    def __init__(self, queue, params):
        """
        Params:
            - move_speed                Movement speed of agents in pixels per frame
            - proximity_threshold       Distance at which agents are considered to have reached the end point, in pixels
            - wait_time                 Number of frames agents wait at the end point before being removed from the sim
        """

        self.queue = queue
        self.params = params
        self.next_id = 1
        self.next_group_id = 1

    @property
    def agents(self):
        return [a for _, group in self.queue for a in group]

    def spawn(self, x, y):
        num_agents = int(self._expo_sample(1, 8))
        spread = self.params.get("spawn_spread", 15)

        group = []
        for _ in range(num_agents):
            # Randomly spawn agents around click location
            ox, oy = random.gauss(0, spread), random.gauss(0, spread)
            agent = Agent(self.next_id, self.next_group_id, f"00:00:00:00:00:{self.next_id:02x}", x + ox, y + oy, 0, 0)

            group.append(agent)
            self.next_id += 1

        self.queue.append((self.next_group_id, group))
        self.next_group_id += 1

    def step(self):
        p = self.params
        v0, tau = p["desired_speed"], p["reaction_time"]                                # driving: target speed, reaction time
        A, B, R = p["A"], p["B"], p["influence_radius"]                                 # repulsion: strength, falloff, cutoff radius
        E, PT, gap = p["end_point"], p["proximity_threshold"], p["follow_gap"]
        HM, GM = p["head_drive_multiplier"], p["group_drive_multiplier"]                # multipliers for driving force of head of front group and trailing groups

        # Reversed centerline unit vector: points from the end back toward the start used for following gap
        sx, sy = p["start_point"]
        cdx, cdy = sx - E[0], sy - E[1]                      
        clen = math.hypot(cdx, cdy)
        back_ux, back_uy = (cdx / clen, cdy / clen) if clen else (0.0, 0.0)

        if not self.queue:
            return

        # Process the agent physically nearest the end point in group 0
        _, front = self.queue[0]
        if front:
            head = min(front, key=lambda a: self._agent_to_point_dist(a, E))
            if self._agent_to_point_dist(head, E) < PT:
                # Head has arrived, hold it at the servery for wait_time frames
                if head.checkpoint_timer < p["wait_time"]:
                    head.checkpoint_timer += 1
                else:
                    front.remove(head)
                    print(f"Agent {head.id} reached end point")
                    if not front:                 
                        # group fully served
                        self.queue.pop(0)
                    if not self.queue:
                        return
                    
        all_agents = self.agents

        # Compute new velocities for every agent
        for i, (group_id, group_agents) in enumerate(self.queue):
            if i == 0:
                # Order group 0 by closeness to goal 
                ordered = sorted(group_agents, key=lambda a: self._agent_to_point_dist(a, E))
            else:
                # Group n>0 follows group n-1 mean with a gap
                _, prior = self.queue[i - 1]
                mx, my = mean(a.x for a in prior), mean(a.y for a in prior)
                group_target = (mx + back_ux * gap, my + back_uy * gap)

            # Iterate the group 0 in sorted order
            seq = ordered if i == 0 else group_agents
            for k, agent in enumerate(seq):
              
                # Calculate agent target point
                if i == 0:
                    if k == 0:
                        # Head of the front group drives straight at the end point.
                        target = E
                        # On arrival, brake instead of driving through the point.
                        if self._agent_to_point_dist(agent, E) < PT:
                            agent.xv *= 0.5
                            agent.yv *= 0.5            
                    else:
                        # Other front-group members follow the agent ahead of them with a gap
                        lead = seq[k - 1]
                        target = (lead.x + back_ux * gap, lead.y + back_uy * gap)
                else:
                    # Trailing-group members follow the group ahead
                    target = group_target
                    

                # --- Driving force: (v0 * ê - v) / tau ---
                # Steers velocity toward `target` at desired speed v0, damped by current velocity and reaction time tau

                # Get drive force multiplier based on position in queue
                drive_mult = HM if (i== 0 and k == 0) else GM

                xu, yu = self._agent_to_point_vec(agent, target)
                dfx = (v0 * drive_mult * xu - agent.xv) / tau
                dfy = (v0 * drive_mult * yu - agent.yv) / tau

                # --- Repulsion force: sum of exponential pushes from nearby agents ---
                # Each neighbor within R contributes A*exp((r_i+r_j - d)/B) along the
                # unit vector pointing from the neighbor to this agent
                rfx, rfy = 0.0, 0.0
                for other in all_agents:
                    if other is agent:
                        continue
                    dist = self._agent_to_agent_dist(agent, other)
                    if dist < R:
                        a_exp = math.exp((agent.radius + other.radius - dist) / B)
                        rfx += A * a_exp * (agent.x - other.x) / dist
                        rfy += A * a_exp * (agent.y - other.y) / dist

                # --- Containment force: gentle pull back toward the centerline ---
                cfx, cfy = self._containment_force(agent)

                # Combine forces
                agent.xv += dfx + (rfx + cfx) / agent.mass
                agent.yv += dfy + (rfy + cfy) / agent.mass

        # Update positions 
        for agent in all_agents:
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

    def _expo_sample(self, start, end):
        """Sample Exponential distribution between start and end, right-skewed towards start"""

        return min(end, start + int(random.expovariate(0.4)))

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
