"""
Defines data class for a single agent in the sim.
"""


class Agent:
    def __init__(self, id, mac, x, y, xv, yv):
        self.id = id
        self.mac = mac
        self.x = x
        self.y = y
        self.xv = xv
        self.yv = yv
        self.checkpoint_timer = 0
        self.frames_per_checkpoint = 300
