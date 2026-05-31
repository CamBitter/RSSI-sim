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
        self.end = params["end_point"]
        self.next_id = len(agents) + 1

    def spawn(self, x, y):
        """Spawn a new agent at (x, y) with velocity towards last agent in queue"""

        agent = Agent(self.next_id, f"00:00:00:00:00:{self.next_id:02x}", x, y, 0, 0)

        # Calculate unit vector towards last agent in queue, or towards end if no agents
        if self.agents:
            xu, yu = self._agent_to_agent_vec(agent, self.agents[-1])
        else:
            xu, yu = self._agent_to_point_vec(agent, self.end)

        # Set velocity along unit vector
        agent.xv = xu * self.params["move_speed"]
        agent.yv = yu * self.params["move_speed"]

        self.agents.append(agent)
        self.next_id += 1

    def step(self):

        # Check if head of queue has reached end point, and if so, increment checkpoint timer or remove from sim
        head = self.agents[0] if self.agents else None
        if (
            head
            and self._agent_to_point_dist(head, self.end)
            < self.params["proximity_threshold"]
        ):
            if head.checkpoint_timer < self.params["wait_time"]:
                head.checkpoint_timer += 1
            else:
                self.agents.pop(0)
                print(f"Agent {head.id} reached end point")

        if not self.agents:
            return

        # Update agent velocities

        # Point new head of queue velocity towards end point
        head = self.agents[0]
        xu, yu = self._agent_to_point_vec(head, self.end)
        head.xv = xu * self.params["move_speed"]
        head.yv = yu * self.params["move_speed"]

        # Point each subsequent agent's velocity towards the agent in front of it
        for i, agent in enumerate(self.agents[1:], start=1):
            prior = self.agents[i - 1]
            xu, yu = self._agent_to_agent_vec(agent, prior)
            agent.xv = xu * self.params["move_speed"]
            agent.yv = yu * self.params["move_speed"]

        # Update agent positions
        for agent in self.agents:
            agent.x += agent.xv
            agent.y += agent.yv

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


if __name__ == "__main__":

    sim_params = {
        "move_speed": 2,
        "proximity_threshold": 50,
        "wait_time": 300,
        "end_point": (50, 100),
    }

    sim = Sim([], sim_params)
    sim.spawn(200, 200)
    sim.spawn(220, 200)

    for _ in range(150):
        sim.step()
        for agent in sim.agents:
            print(f"Agent {agent.id} is at ({agent.x}, {agent.y})")