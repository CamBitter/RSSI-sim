from gui import setup_ui, redraw
from sim import Sim
from agent import Agent
import dearpygui.dearpygui as dpg


def main():

    W, H = 1600, 900

    sim_params = {
        "desired_speed": 0.6,               # v0: target cruising speed (px/frame)
        "reaction_time": 0.6,               # tau: lower = snappier driving (higher starves movement)
        "influence_radius": 80,             # cutoff distance for agent/wall repulsion (px)
        "proximity_threshold": 20,          # distance from end point counted as "arrived" (px)
        "A": 1.2,                           # agent repulsion strength
        "B": 40,                            # agent repulsion falloff distance (px)
        "A_wall": 5,                        # wall repulsion strength (stronger than agents)
        "B_wall": 20,                       # wall repulsion falloff distance (px)
        "follow_gap": 25,                   # standoff distance behind leader/group (px)
        "wait_time": 60,                    # frames head waits at servery before removal
        "head_drive_multiplier": 2.0,       # extra driving force on the queue head
        "group_drive_multiplier": 1.5,      # extra driving force on trailing-group agents
        "start_point": (W / 1.2, H / 1.2),  # queue origin (defines centerline + spawn anchor)
        "end_point": (W / 10, H / 10),      # servery / goal point
        "spawn_spread": 20,                 # gaussian jitter radius on group spawn (px)
    }

    sim = Sim(sim_params)
    setup_ui(W, H, sim)

    while dpg.is_dearpygui_running():
        sim.step()
        redraw(sim.agents, sim.walls)
        dpg.render_dearpygui_frame()

    dpg.destroy_context()


if __name__ == "__main__":
    main()
