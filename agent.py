"""
Defines data class for a single agent in the sim.
"""


class Agent:
    def __init__(self, id, group_id, mac, x, y, xv, yv):
        self.id = id
        self.group_id = group_id
        self.mac = mac
        self.x = x
        self.y = y
        self.xv = xv
        self.yv = yv
        self.radius = 14
        self.mass = 1
        self.checkpoint_timer = 0
