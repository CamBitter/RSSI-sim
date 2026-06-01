from gui import setup_ui, redraw
from sim import Sim
from agent import Agent
import dearpygui.dearpygui as dpg


def main():

    W, H = 1600, 900

    sim_params = {
        "desired_speed": 0.6,
        "reaction_time": 0.6,
        "influence_radius": 80,
        "proximity_threshold": 20,
        "A": 1.2,
        "B": 40,
        "A_wall": 5,
        "B_wall": 20,
        "containment_strength": 0.002,
        "corridor_width": 80,
        "follow_gap": 25,
        "wait_time": 60,
        "start_point": (W / 1.2, H / 1.2),
        "end_point": (W / 10, H / 10),
        "spawn_spread": 20,
        "head_drive_multiplier": 2.0,
        "group_drive_multiplier": 1.5,
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
